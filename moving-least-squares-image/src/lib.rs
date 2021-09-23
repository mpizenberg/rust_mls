use image::{Rgb, RgbImage};

use moving_least_squares as mls;
mod interpolation;

/// Compute the warped image with the affine version of MLS.
///
/// The new image is back projected as if the source and destination
/// control points were reversed.
///
/// Interpolation is done with bilinear interpolation.
pub fn affine_reverse_dense(
    img_src: &RgbImage,
    controls_src: &[(f32, f32)],
    controls_dst: &[(f32, f32)],
) -> RgbImage {
    let (width, height) = img_src.dimensions();
    let color_outside = Rgb([0, 0, 0]);
    RgbImage::from_fn(width, height, |x, y| {
        let (x2, y2) = mls::deform_affine(controls_dst, controls_src, (x as f32, y as f32));
        // nearest_neighbor(img_src, x2, y2).unwrap_or(color_outside)
        interpolation::bilinear(img_src, x2, y2).unwrap_or(color_outside)
    })
}

/// Compute the warped image with the similarity version of MLS.
///
/// The new image is back projected as if the source and destination
/// control points were reversed.
///
/// Interpolation is done with bilinear interpolation.
pub fn similarity_reverse_dense(
    img_src: &RgbImage,
    controls_src: &[(f32, f32)],
    controls_dst: &[(f32, f32)],
) -> RgbImage {
    let (width, height) = img_src.dimensions();
    let color_outside = Rgb([0, 0, 0]);
    RgbImage::from_fn(width, height, |x, y| {
        let (x2, y2) = mls::deform_similarity(controls_dst, controls_src, (x as f32, y as f32));
        // nearest_neighbor(img_src, x2, y2).unwrap_or(color_outside)
        interpolation::bilinear(img_src, x2, y2).unwrap_or(color_outside)
    })
}

/// Compute the warped image with the rigid version of MLS.
///
/// The new image is back projected as if the source and destination
/// control points were reversed.
///
/// Interpolation is done with bilinear interpolation.
pub fn rigid_reverse_dense(
    img_src: &RgbImage,
    controls_src: &[(f32, f32)],
    controls_dst: &[(f32, f32)],
) -> RgbImage {
    let (width, height) = img_src.dimensions();
    let color_outside = Rgb([0, 0, 0]);
    RgbImage::from_fn(width, height, |x, y| {
        let (x2, y2) = mls::deform_rigid(controls_dst, controls_src, (x as f32, y as f32));
        // nearest_neighbor(img_src, x2, y2).unwrap_or(color_outside)
        interpolation::bilinear(img_src, x2, y2).unwrap_or(color_outside)
    })
}

/// Retrieve nearest neighbor pixel for given coordinates.
fn nearest_neighbor(img: &RgbImage, x: f32, y: f32) -> Option<Rgb<u8>> {
    let (width, height) = img.dimensions();
    let u = x.round() as i32;
    let v = y.round() as i32;
    let u_inside = u >= 0 && u < width as i32;
    let v_inside = v >= 0 && v < height as i32;
    if u_inside && v_inside {
        Some(img.get_pixel(u as u32, v as u32).clone())
    } else {
        None
    }
}
