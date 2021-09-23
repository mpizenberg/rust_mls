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
    let warped_img = mls_image::affine_reverse_dense(&img, controls_src, controls_dst);

    // Create a window with default options and display the image.
    let window = create_window("image", Default::default())?;
    window.set_image("woody", img)?;

    // Display new warped image in a new window.
    let warped_window = create_window("warped image", Default::default())?;
    warped_window.set_image("woody_warped", warped_img)?;

    window.wait_until_destroyed()?;
    warped_window.wait_until_destroyed()?;
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
