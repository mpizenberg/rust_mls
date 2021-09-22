use image::{GenericImageView, Pixel, RgbImage};
use show_image::create_window;

use moving_least_squares as mls;

#[show_image::main]
fn main() -> Result<(), Box<dyn std::error::Error>> {
    // Open an image from disk.
    let img = image::open("data/woody.jpg")?;
    let (width, height) = img.dimensions();

    // Convert image into RGB.
    let mut img = img.into_rgb8();

    // Define the source control points.
    let controls_src: &[(f32, f32)] = &[
        (40.0, 160.0),
        (170.0, 160.0),
        (320.0, 160.0),
        (130.0, 280.0),
        (220.0, 280.0),
        (117.0, 369.0),
        (250.0, 369.0),
    ];

    // Define the destination control points.
    let controls_dst: &[(f32, f32)] = &[
        (40.0, 190.0),
        (170.0, 160.0),
        (370.0, 120.0),
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
    let warped_img = RgbImage::from_fn(width, height, |x, y| {
        let (x2, y2) = mls::deform_affine(controls_dst, controls_src, (x as f32, y as f32), 1e-10);
        // nearest neighbor appoximation for now.
        let u = x2.round() as i32;
        let v = y2.round() as i32;
        let u_inside = u >= 0 && u < width as i32;
        let v_inside = v >= 0 && v < height as i32;
        if u_inside && v_inside {
            img.get_pixel(u as u32, v as u32).clone()
        } else {
            image::Rgb([0, 0, 0])
        }
    });

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
