// SPDX-License-Identifier: MPL-2.0

//! Functions to compute warped images with an MLS algorithm.
//! Two warping functions are provided:
//!  - a dense warp where the deformation is computed for each pixel,
//!  - a sparse warp where its only computed on a sparse grid,
//!    and the other pixels locations are interpolated.

#![warn(missing_docs)]

use image::{Rgb, RgbImage};

mod interpolation;

/// Behaves like `RgbImage::from_fn` but will be parallelized if the `rayon` feature is enabled
#[cfg(not(feature = "rayon"))]
fn rgb_image_from_fn<F>(width: u32, height: u32, f: F) -> RgbImage
where
    F: Fn(u32, u32) -> Rgb<u8>,
{
    RgbImage::from_fn(width, height, f)
}

/// Behaves like `RgbImage::from_fn` but will be parallelized if the `rayon` feature is enabled
#[cfg(feature = "rayon")]
fn rgb_image_from_fn<F>(width: u32, height: u32, f: F) -> RgbImage
where
    F: Fn(u32, u32) -> Rgb<u8> + Send + Sync,
{
    use rayon::prelude::*;

    let mut buf = RgbImage::new(width, height);

    buf.par_chunks_exact_mut(3)
        .enumerate()
        .map(|(idx, pixel)| (idx as u32 % width, idx as u32 / width, pixel))
        .for_each(|(x, y, pixel)| {
            pixel.copy_from_slice(&f(x, y).0);
        });

    buf
}

// Dense interpolation #########################################################

/// Compute the warped image with an MLS algorithm.
/// The last argument is the MLS version you choose.
///
/// The new image is back projected as if the source and destination
/// control points were reversed.
///
/// The warp is computed densely, for every pixel.
///
/// Pixels interpolation is done with bilinear interpolation.
#[allow(clippy::type_complexity)]
pub fn reverse_dense(
    img_src: &RgbImage,
    controls_src: &[(f32, f32)],
    controls_dst: &[(f32, f32)],
    deform_function: fn(&[(f32, f32)], &[(f32, f32)], (f32, f32)) -> (f32, f32),
) -> RgbImage {
    let (width, height) = img_src.dimensions();
    let color_outside = Rgb([0, 0, 0]);
    rgb_image_from_fn(width, height, |x, y| {
        let (x2, y2) = deform_function(controls_dst, controls_src, (x as f32, y as f32));
        // nearest_neighbor(img_src, x2, y2).unwrap_or(color_outside)
        interpolation::bilinear(img_src, x2, y2).unwrap_or(color_outside)
    })
}

// Sparse interpolation ########################################################

/// Compute the warped image with an MLS algorithm.
/// The last argument is the MLS version you choose.
///
/// The new image is back projected as if the source and destination
/// control points were reversed.
///
/// Only a sparse grid subset of the pixels are reprojected with the actual MLS function.
/// The other pixels locations are interpolated bilinearly.
/// For example, a subresolution factor of 3 means that only 1 in 3 pixels
/// per row and per column is actually projected with MLS.
/// In the case of a big number of control points (> 100),
/// this can produce a significant speedup (roughly 16x for a subresolution factor of 4),
/// with a minimal impact on the produced image.
///
/// Pixels interpolation is done with bilinear interpolation.
#[allow(clippy::type_complexity)]
pub fn reverse_sparse(
    img_src: &RgbImage,
    controls_src: &[(f32, f32)],
    controls_dst: &[(f32, f32)],
    subresolution_factor: u32,
    deform_function: fn(&[(f32, f32)], &[(f32, f32)], (f32, f32)) -> (f32, f32),
) -> RgbImage {
    let (width, height) = img_src.dimensions();
    let color_outside = Rgb([0, 0, 0]);

    // size of the subresolution matrix for which we actually compute the MLS reprojections
    let sub_width = (width - 1) / subresolution_factor + 2;
    let sub_height = (height - 1) / subresolution_factor + 2;

    // the anchors are the MLS reprojection of the subresolution matrix of points
    let anchors: Vec<Vec<(f32, f32)>> = (0..sub_height)
        .map(|v| {
            let y = (v * subresolution_factor) as f32;
            (0..sub_width)
                .map(|u| {
                    let x = (u * subresolution_factor) as f32;
                    deform_function(controls_dst, controls_src, (x, y))
                })
                .collect()
        })
        .collect();

    // apply bilinear warp to compute the full warp
    rgb_image_from_fn(width, height, |x, y| {
        let sub_left = x / subresolution_factor;
        let sub_top = y / subresolution_factor;
        let top_left_corner = (
            subresolution_factor * sub_left,
            subresolution_factor * sub_top,
        );
        let bot_right_corner = (
            top_left_corner.0 + subresolution_factor,
            top_left_corner.1 + subresolution_factor,
        );
        let sub_left = sub_left as usize;
        let sub_top = sub_top as usize;
        // TODO: should try to avoid retrieving bloc corners for each pixel
        let corners_dst = [
            anchors[sub_top][sub_left],
            anchors[sub_top][sub_left + 1],
            anchors[sub_top + 1][sub_left],
            anchors[sub_top + 1][sub_left + 1],
        ];
        let (x2, y2) = bilinear_warp(top_left_corner, bot_right_corner, corners_dst, (x, y));
        interpolation::bilinear(img_src, x2, y2).unwrap_or(color_outside)
    })
}

/// Perform bilinear warping of the pixel.
/// WARNING: make sure it is within the bloc corners.
fn bilinear_warp(
    top_left_corner: (u32, u32),
    bot_right_corner: (u32, u32),
    corners_dst: [(f32, f32); 4],
    src_pos: (u32, u32),
) -> (f32, f32) {
    let (u, v) = src_pos;
    let (left, top) = top_left_corner;
    let (right, bottom) = bot_right_corner;
    let [dst_tl, dst_tr, dst_bl, dst_br] = corners_dst;

    // compute bilinear coefficients
    let coef_left = right - u;
    let coef_right = u - left;
    let coef_top = bottom - v;
    let coef_bot = v - top;

    let coef_tl = (coef_top * coef_left) as f32;
    let coef_tr = (coef_top * coef_right) as f32;
    let coef_bl = (coef_bot * coef_left) as f32;
    let coef_br = (coef_bot * coef_right) as f32;

    // perform bilinear reprojection
    let area = ((right - left) * (bottom - top)) as f32;
    let x =
        ((coef_tl * dst_tl.0) + (coef_tr * dst_tr.0) + (coef_bl * dst_bl.0) + (coef_br * dst_br.0))
            / area;
    let y =
        ((coef_tl * dst_tl.1) + (coef_tr * dst_tr.1) + (coef_bl * dst_bl.1) + (coef_br * dst_br.1))
            / area;

    (x, y)
}
