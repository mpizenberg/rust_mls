# Image Deformation Using Moving Least Squares

Rust implementation of the paper ["Image Deformation Using Moving Least Squares", Schaefer 2006][pdf].

![mls demo output][img]

[pdf]: https://people.engr.tamu.edu/schaefer/research/mls.pdf
[img]: https://mpizenberg.github.io/resources/moving-least-squares/mls-demo.jpg

This repository contains the MLS warping code inside `moving-least-squares/`, an application to image warping inside `moving-least-squaers-image/` as well as a demo with example usage producing the above figure inside the `moving-least-squares-demo/` directory.
To run the demo:

```sh
cd moving-least-squares-demo/
cargo run --release
```

Here is what using the library looks like:

```rust
// Open an image from disk.
let img = image::open("data/woody.jpg")?.into_rgb8();

// Define the source control points.
let controls_src: &[(f32, f32)] = &[
    (20.0, 160.0),
    ...
    (250.0, 369.0),
];

// Define the destination control points.
let controls_dst: &[(f32, f32)] = &[
    (20.0, 250.0),
    ...
    (250.0, 369.0),
];

// Create new warped image.
let warped_img_affine =
    mls_image::reverse_dense(&img, controls_src, controls_dst, mls::deform_affine);
```
