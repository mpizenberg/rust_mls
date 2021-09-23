// SPDX-License-Identifier: MPL-2.0

//! Helper functions to interpolate / extrapolate warped images.

use image::{ImageBuffer, Pixel, Primitive, Rgb};
use std::ops::{Add, Deref, Mul};

/// Trait for types that can be linearly interpolated with the `linear` function.
///
/// The `Vector` generic type refers to the intermediate type used during interpolations.
/// It usually is the `f32` scalar or a vector of `f32` values.
///
/// The `Output` type is the final type obtained after interpolation.
/// It is obtained via conversion from the interpolated vector.
pub trait CanLinearInterpolate<Vector, Output>
where
    Vector: Add<Output = Vector>,
    f32: Mul<Vector, Output = Vector>,
{
    fn into_vector(self) -> Vector;
    fn from_vector(v: Vector) -> Output;
}

/// Implement CanLinearInterpolate for u8 with f32 outputs.
/// WARNING: beware that interpolating with a f32 output normalizes values from [0-255] to [0.0, 1.0].
impl CanLinearInterpolate<f32, f32> for u8 {
    fn into_vector(self) -> f32 {
        self as f32
    }
    fn from_vector(v: f32) -> f32 {
        (v / 255.0).max(0.0).min(1.0)
    }
}

/// Implement CanLinearInterpolate for u8.
impl CanLinearInterpolate<f32, u8> for u8 {
    fn into_vector(self) -> f32 {
        self as f32
    }
    fn from_vector(v: f32) -> u8 {
        v.max(0.0).min(u8::MAX as f32).round() as u8
    }
}

/// Implement CanLinearInterpolate for u16 with f32 outputs.
/// WARNING: beware that interpolating with a f32 output normalizes values from [0-65535] to [0.0, 1.0].
impl CanLinearInterpolate<f32, f32> for u16 {
    fn into_vector(self) -> f32 {
        self as f32
    }
    fn from_vector(v: f32) -> f32 {
        (v / u16::MAX as f32).max(0.0).min(1.0)
    }
}

/// Implement CanLinearInterpolate for u16.
impl CanLinearInterpolate<f32, u16> for u16 {
    fn into_vector(self) -> f32 {
        self as f32
    }
    fn from_vector(v: f32) -> u16 {
        v.max(0.0).min(u16::MAX as f32).round() as u16
    }
}

/// Implement CanLinearInterpolate for (T,T,T) if T also implements it.
impl<O, T: CanLinearInterpolate<f32, O>> CanLinearInterpolate<Vec3, (O, O, O)> for (T, T, T) {
    fn into_vector(self) -> Vec3 {
        Vec3 {
            x: self.0.into_vector(),
            y: self.1.into_vector(),
            z: self.2.into_vector(),
        }
    }
    fn from_vector(v: Vec3) -> (O, O, O) {
        (
            T::from_vector(v.x),
            T::from_vector(v.y),
            T::from_vector(v.z),
        )
    }
}

/// Implement CanLinearInterpolate for Rgb<T> if T also implements it.
impl<T, O> CanLinearInterpolate<Vec3, Rgb<O>> for Rgb<T>
where
    T: Primitive + CanLinearInterpolate<f32, O>,
    O: Primitive,
{
    fn into_vector(self) -> Vec3 {
        let [x, y, z] = self.0;
        Vec3 {
            x: x.into_vector(),
            y: y.into_vector(),
            z: z.into_vector(),
        }
    }
    fn from_vector(v: Vec3) -> Rgb<O> {
        Rgb([
            T::from_vector(v.x),
            T::from_vector(v.y),
            T::from_vector(v.z),
        ])
    }
}

/// Simple bilinear interpolation of a pixel with floating point coordinates.
#[allow(clippy::many_single_char_names)]
#[allow(clippy::cast_possible_truncation)]
#[allow(clippy::cast_sign_loss)]
#[allow(clippy::cast_precision_loss)]
pub fn bilinear<V, P, Container, O>(img: &ImageBuffer<P, Container>, x: f32, y: f32) -> Option<O>
where
    V: Add<Output = V>,
    f32: Mul<V, Output = V>,
    P: Pixel + 'static,
    Container: Deref<Target = [P::Subpixel]>,
    P: CanLinearInterpolate<V, O>,
{
    let (width, height) = img.dimensions();
    let u = x.floor();
    let v = y.floor();
    if u >= 0.0 && u < (width - 2) as f32 && v >= 0.0 && v < (height - 2) as f32 {
        // Linear interpolation inside boundaries.
        let u_0 = u as u32;
        let v_0 = v as u32;
        let u_1 = u_0 + 1;
        let v_1 = v_0 + 1;
        let a = x - u;
        let b = y - v;
        let uv_00 = img.get_pixel(u_0, v_0).into_vector();
        let uv_10 = img.get_pixel(u_1, v_0).into_vector();
        let uv_01 = img.get_pixel(u_0, v_1).into_vector();
        let uv_11 = img.get_pixel(u_1, v_1).into_vector();
        let interp = Mul::<f32>::mul(1.0 - b, 1.0 - a) * uv_00
            + Mul::<f32>::mul(b, 1.0 - a) * uv_01
            + Mul::<f32>::mul(1.0 - b, a) * uv_10
            + Mul::<f32>::mul(b, a) * uv_11;
        Some(P::from_vector(interp))
    } else {
        None
    }
}

// 3D vector helper ############################################################
// That's to avoid a dependency on a heavy package such as nalgebra

/// Vec3 represented by a 3x1 column vector.
#[derive(Clone, Copy)]
pub struct Vec3 {
    x: f32,
    y: f32,
    z: f32,
}

// Add two vectors
impl Add for Vec3 {
    type Output = Self;
    fn add(self, rhs: Self) -> Self::Output {
        Self {
            x: self.x + rhs.x,
            y: self.y + rhs.y,
            z: self.z + rhs.z,
        }
    }
}

// Scalar multiplication
impl Mul<Vec3> for f32 {
    type Output = Vec3;
    fn mul(self, rhs: Vec3) -> Self::Output {
        Vec3 {
            x: self * rhs.x,
            y: self * rhs.y,
            z: self * rhs.z,
        }
    }
}
