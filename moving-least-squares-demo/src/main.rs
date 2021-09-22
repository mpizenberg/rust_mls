use image::{GenericImageView, Pixel, RgbImage};
use show_image::create_window;

#[show_image::main]
fn main() -> Result<(), Box<dyn std::error::Error>> {
    // Open an image from disk.
    let img = image::open("data/woody.jpg")?;
    let (width, height) = img.dimensions();

    // Convert image into RGB.
    let mut img = img.into_rgb8();

    // Draw some points.
    draw_point((100.0, 200.0), 10.0, (255, 0, 0), &mut img);

    // Create a window with default options and display the image.
    let window = create_window("image", Default::default())?;
    window.set_image("woody", img)?;

    window.wait_until_destroyed()?;
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
