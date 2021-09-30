use image::{Pixel, RgbImage};
use show_image::create_window;

use moving_least_squares_image as mls_image;

#[show_image::main]
fn main() -> Result<(), Box<dyn std::error::Error>> {
    // Open an image from disk.
    let img = image::open("data/woody.jpg")?;

    // Convert image into RGB.
    let mut img = img.into_rgb8();

    // Define the source control points.
    let controls_src: &[(f32, f32)] = &[
        (20.0, 160.0),
        (170.0, 160.0),
        (330.0, 160.0),
        (130.0, 280.0),
        (220.0, 280.0),
        (117.0, 369.0),
        (250.0, 369.0),
    ];

    // Define the destination control points.
    let controls_dst: &[(f32, f32)] = &[
        (20.0, 250.0),
        (170.0, 160.0),
        (390.0, 50.0),
        (110.0, 280.0),
        (200.0, 280.0),
        (117.0, 369.0),
        (250.0, 369.0),
    ];

    // Draw source control points.
    let red: (u8, u8, u8) = (255, 0, 0);
    controls_src
        .iter()
        .for_each(|&p| draw_point(p, 5.0, red, &mut img));

    // Create new warped image.
    let warped_img_affine = mls_image::affine_reverse_dense(&img, controls_src, controls_dst);
    let warped_img_similarity =
        mls_image::similarity_reverse_dense(&img, controls_src, controls_dst);
    let warped_img_rigid = mls_image::rigid_reverse_dense(&img, controls_src, controls_dst);
    let warped_img_rigid_sparse =
        mls_image::rigid_reverse_sparse(&img, controls_src, controls_dst, 4);

    // Create a window with default options and display the image.
    let window = create_window("image", Default::default())?;
    window.set_image("woody", img)?;

    // Display new warped image in a new window.
    let warped_window_affine = create_window("warped image (affine)", Default::default())?;
    warped_window_affine.set_image("woody_warped_affine", warped_img_affine)?;

    // Display new warped image in a new window.
    let warped_window_similarity = create_window("warped image (similarity)", Default::default())?;
    warped_window_similarity.set_image("woody_warped_similarity", warped_img_similarity)?;

    // Display new warped image in a new window.
    let warped_window_rigid = create_window("warped image (rigid)", Default::default())?;
    warped_window_rigid.set_image("woody_warped_rigid", warped_img_rigid)?;

    // Display new warped image in a new window.
    let warped_window_rigid_sparse =
        create_window("warped image (rigid_sparse)", Default::default())?;
    warped_window_rigid_sparse.set_image("woody_warped_rigid_sparse", warped_img_rigid_sparse)?;

    window.wait_until_destroyed()?;
    warped_window_affine.wait_until_destroyed()?;
    warped_window_similarity.wait_until_destroyed()?;
    warped_window_rigid.wait_until_destroyed()?;
    warped_window_rigid_sparse.wait_until_destroyed()?;
    Ok(())
}

// Helpers #####################################################################

fn draw_point((px, py): (f32, f32), radius: f32, (r, g, b): (u8, u8, u8), img: &mut RgbImage) {
    for y in ((py - radius).floor() as u32)..((py + radius).ceil() as u32 + 1) {
        for x in ((px - radius).floor() as u32)..((px + radius).ceil() as u32 + 1) {
            // local coordinates (dx, dy) of the point inside the small window
            let dx = x as f32 - px;
            let dy = y as f32 - py;
            let d = (dx * dx + dy * dy).sqrt();
            // blend coefficient depending on the distance d and the point radius
            // 1 inside, 0 outside.
            let blend_coef = if d < radius {
                1.0
            } else {
                (1.0 - d + radius).max(0.0)
            };
            // blend the pixel with the new color
            let pix = img.get_pixel_mut(x, y);
            pix.channels_mut()
                .into_iter()
                .zip([r, g, b])
                .for_each(|(p, c)| {
                    *p = (blend_coef * c as f32 + (1.0 - blend_coef) * *p as f32) as u8
                });
        }
    }
}
