// SPDX-License-Identifier: MPL-2.0

//! Image deformation using moving least squares.

#![warn(missing_docs)]

use core::ops::{Add, Mul, Sub};

/// Move a given point from its original position to its new position
/// according to the deformation that transforms the original control points
/// into their displaced locations.
///
/// The estimated transformation is an affine 2d transformation.
fn deform_affine(
    controls_p: &[(f32, f32)], // p in the paper
    controls_q: &[(f32, f32)], // q in the paper
    point: (f32, f32),         // v in the paper
    proximity_threshold: f32,  // if v is too close to a control point p, we return its associated q
) -> (f32, f32) {
    let v = Point::from(point);
    let sqr_dist_ = |p: Point| (p - v).sqr_norm();
    let vx = point.0;
    let vy = point.1;
    let sqr_dist = |&(x, y): &(f32, f32)| (x - vx).powi(2) + (y - vy).powi(2);

    // The weight of a given control point depends on its distance to the current point.
    // CAREFUL: this w can go to infinity.
    let weight = |pt| 1.0 / sqr_dist(pt);
    let w_all: Vec<_> = controls_p.iter().map(weight).collect();
    let w_sum: f32 = w_all.iter().sum();
    if w_sum.is_infinite() {
        // Most probably, at least one of the weights is infinite,
        // because our point basically coincide with a control point.
        let index = w_all
            .iter()
            .position(|w| w.is_infinite())
            .expect("There is an infinite sum of the weights but none is infinite");
        return controls_q[index];
    }

    // Compute the centroids p* and q*.
    let (wpx_star, wpy_star): (f32, f32) = w_all
        .iter()
        .zip(controls_p)
        .map(|(w, (px, py))| (w * px, w * py))
        .fold((0.0, 0.0), |(sx, sy), (wpx, wpy)| (sx + wpx, sy + wpy));
    let (px_star, py_star) = (wpx_star / w_sum, wpy_star / w_sum);

    let (wqx_star, wqy_star): (f32, f32) = w_all
        .iter()
        .zip(controls_q)
        .map(|(w, (qx, qy))| (w * qx, w * qy))
        .fold((0.0, 0.0), |(sx, sy), (wqx, wqy)| (sx + wqx, sy + wqy));
    let (qx_star, qy_star) = (wqx_star / w_sum, wqy_star / w_sum);

    // Compute the affine matrix M.
    let p_hat: Vec<_> = controls_p
        .iter()
        .map(|(px, py)| (px - px_star, py - py_star))
        .collect();
    // m_p_hat is a 2x2 matrix stored in 4-tuple in column order.
    // actually 3-tuple since the matrix is symmetric.
    let (mp_xx, mp_xy, mp_yy): (f32, f32, f32) = w_all
        .iter()
        .zip(&p_hat)
        .map(|(w, (px, py))| (w * px * px, w * px * py, w * py * py))
        .fold((0.0, 0.0, 0.0), |(s1, s2, s4), (pxx, pxy, pyy)| {
            (s1 + pxx, s2 + pxy, s4 + pyy)
        });
    // Inverse that 2x2 matrix first part of M.
    let det_coef = 1.0 / (mp_xx * mp_yy - mp_xy * mp_xy);
    let mp_inv_xx = det_coef * mp_yy;
    let mp_inv_xy = det_coef * -mp_xy;
    let mp_inv_yy = det_coef * mp_xx;
    // Compute the second part of M.
    let (mq_xx, mq_yx, mq_xy, mq_yy): (f32, f32, f32, f32) = w_all
        .iter()
        .zip(&p_hat)
        .zip(controls_q)
        .map(|((w, (phx, phy)), (qx, qy))| {
            let (qhx, qhy) = (qx - qx_star, qy - qy_star);
            (w * phx * qhx, w * phy * qhx, w * phx * qhy, w * phy * qhy)
        })
        .fold(
            (0.0, 0.0, 0.0, 0.0),
            |(s1, s2, s3, s4), (qxx, qyx, qxy, qyy)| (s1 + qxx, s2 + qyx, s3 + qxy, s4 + qyy),
        );
    // Compute actual coefficients of M.
    let m_xx = mp_inv_xx * mq_xx + mp_inv_xy * mq_yx;
    let m_yx = mp_inv_xy * mq_xx + mp_inv_yy * mq_yx;
    let m_xy = mp_inv_xx * mq_xy + mp_inv_xy * mq_yy;
    let m_yy = mp_inv_xy * mq_xy + mp_inv_yy * mq_yy;

    // Finally compute the projection of our original point.
    let vx_star: f32;
    todo!()
}

// 2D points helper ############################################################
// That's to avoid a dependency on a heavy package such as nalgebra

/// Point represented by a 2x1 column vector.
#[derive(Clone, Copy)]
struct Point {
    x: f32,
    y: f32,
}

impl Point {
    /// Dot product with another point.
    fn dot(self, rhs: Self) -> f32 {
        self.x * rhs.x + self.y * rhs.y
    }

    /// Square norm.
    fn sqr_norm(self) -> f32 {
        self.x * self.x + self.y * self.y
    }
}

// Convert from (x,y) to Point { x, y }
impl From<(f32, f32)> for Point {
    fn from((x, y): (f32, f32)) -> Self {
        Point { x, y }
    }
}

// Add two points
impl Add for Point {
    type Output = Point;
    fn add(self, rhs: Self) -> Self::Output {
        Point {
            x: self.x + rhs.x,
            y: self.y + rhs.y,
        }
    }
}

// Substract a point
impl Sub for Point {
    type Output = Point;
    fn sub(self, rhs: Self) -> Self::Output {
        Point {
            x: self.x - rhs.x,
            y: self.y - rhs.y,
        }
    }
}

// Scalar multiplication
impl Mul<Point> for f32 {
    type Output = Point;
    fn mul(self, rhs: Point) -> Self::Output {
        Point {
            x: self * rhs.x,
            y: self * rhs.y,
        }
    }
}

// 2x2 matrix helper ###########################################################
// That's to avoid a dependency on a heavy package such as nalgebra

/// 2x2 Matrix with the following coefficients.
///
/// | m11  m12 |
/// | m21  m22 |
struct Mat2 {
    m11: f32,
    m21: f32,
    m12: f32,
    m22: f32,
}
