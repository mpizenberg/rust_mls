use image::RgbImage;

use moving_least_squares as mls;

/// Compute the warped image with the affine version of MLS.
///
/// The new image is back projected as if the source and destination
/// control points were reversed.
///
/// Interpolation is done with nearest neighbor for the time being.
pub fn affine_reverse_dense(
    img_src: &RgbImage,
    controls_src: &[(f32, f32)],
    controls_dst: &[(f32, f32)],
) -> RgbImage {
    let (width, height) = img_src.dimensions();
    RgbImage::from_fn(width, height, |x, y| {
        let (x2, y2) = mls::deform_affine(controls_dst, controls_src, (x as f32, y as f32), 1e-10);
        // nearest neighbor appoximation for now.
        let u = x2.round() as i32;
        let v = y2.round() as i32;
        let u_inside = u >= 0 && u < width as i32;
        let v_inside = v >= 0 && v < height as i32;
        if u_inside && v_inside {
            img_src.get_pixel(u as u32, v as u32).clone()
        } else {
            image::Rgb([0, 0, 0])
        }
    })
}
