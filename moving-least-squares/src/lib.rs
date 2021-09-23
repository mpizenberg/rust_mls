// SPDX-License-Identifier: MPL-2.0

//! Image deformation using moving least squares.

#![warn(missing_docs)]

use core::iter::Sum;
use core::ops::{Add, Mul, Sub};

/// Move a given point from its original position to its new position
/// according to the deformation that transforms the original control points
/// into their displaced locations.
///
/// The estimated transformation is an affine 2d transformation.
pub fn deform_affine(
    controls_p: &[(f32, f32)], // p in the paper
    controls_q: &[(f32, f32)], // q in the paper
    point: (f32, f32),         // v in the paper
) -> (f32, f32) {
    let v = Point::from(point);
    let sqr_dist = |p: Point| (p - v).sqr_norm();

    // The weight of a given control point depends on its distance to the current point.
    // CAREFUL: this w can go to infinity.
    let weight = |pt| 1.0 / sqr_dist(pt);
    let w_all: Vec<_> = controls_p.iter().map(|&p| weight(p.into())).collect();
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

    // Compute the centroid p*.
    let wp_star_sum: Point = w_all
        .iter()
        .zip(controls_p)
        .map(|(&w, &p)| w * Point::from(p))
        .sum();
    let p_star = (1.0 / w_sum) * wp_star_sum;

    // Compute the centroid q*.
    let wq_star_sum: Point = w_all
        .iter()
        .zip(controls_q)
        .map(|(&w, &q)| w * Point::from(q))
        .sum();
    let q_star = (1.0 / w_sum) * wq_star_sum;

    // Compute the affine matrix M.
    let p_hat: Vec<Point> = controls_p
        .iter()
        .map(|&p| Point::from(p) - p_star)
        .collect();
    // mp is a 2x2 matrix.
    let mp: Mat2 = w_all
        .iter()
        .zip(&p_hat)
        .map(|(&w, &p)| w * p.times_transpose(p))
        .sum();
    // Compute the second part of M.
    let mq: Mat2 = w_all
        .iter()
        .zip(&p_hat)
        .zip(controls_q)
        .map(|((&w, &ph), &q)| {
            let qh = Point::from(q) - q_star;
            (w * ph).times_transpose(qh)
        })
        .sum();

    // Finally compute the projection of our original point.
    ((v - p_star).transpose_mul(mp.inv()).transpose_mul(mq) + q_star).into()
}

/// Move a given point from its original position to its new position
/// according to the deformation that transforms the original control points
/// into their displaced locations.
///
/// The estimated transformation is a 2D similarity.
pub fn deform_similarity(
    controls_p: &[(f32, f32)], // p in the paper
    controls_q: &[(f32, f32)], // q in the paper
    point: (f32, f32),         // v in the paper
) -> (f32, f32) {
    let v = Point::from(point);
    let sqr_dist = |p: Point| (p - v).sqr_norm();

    // The weight of a given control point depends on its distance to the current point.
    // CAREFUL: this w can go to infinity.
    let weight = |pt| 1.0 / sqr_dist(pt);
    let w_all: Vec<_> = controls_p.iter().map(|&p| weight(p.into())).collect();
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

    // Compute the centroid p*.
    let wp_star_sum: Point = w_all
        .iter()
        .zip(controls_p)
        .map(|(&w, &p)| w * Point::from(p))
        .sum();
    let p_star = (1.0 / w_sum) * wp_star_sum;

    // Compute the centroid q*.
    let wq_star_sum: Point = w_all
        .iter()
        .zip(controls_q)
        .map(|(&w, &q)| w * Point::from(q))
        .sum();
    let q_star = (1.0 / w_sum) * wq_star_sum;

    // Compute p_hat.
    let p_hat: Vec<Point> = controls_p
        .iter()
        .map(|&p| Point::from(p) - p_star)
        .collect();

    // Compute q_hat.
    let q_hat: Vec<Point> = controls_q
        .iter()
        .map(|&q| Point::from(q) - q_star)
        .collect();

    // Compute mu_s (eq 6).
    let mu_s: f32 = w_all
        .iter()
        .zip(&p_hat)
        .map(|(wi, pi)| wi * pi.sqr_norm())
        .sum();

    // Compute M (eq 6)
    let m: Mat2 = w_all
        .iter()
        .zip(&p_hat)
        .zip(&q_hat)
        .map(|((&wi, pi), qi)| {
            let p_mat = Mat2 {
                m11: pi.x,
                m12: pi.y,
                m21: pi.y,
                m22: -pi.x,
            };
            let q_mat = Mat2 {
                m11: qi.x,
                m21: qi.y,
                m12: qi.y,
                m22: -qi.x,
            };
            wi * p_mat * q_mat
        })
        .sum();
    let m = (1.0 / mu_s) * m;

    // Finally compute the projection of our original point (eq 3).
    ((v - p_star).transpose_mul(m) + q_star).into()
}

/// Move a given point from its original position to its new position
/// according to the deformation that transforms the original control points
/// into their displaced locations.
///
/// The estimated transformation is a 2D rigid deformation.
pub fn deform_rigid(
    controls_p: &[(f32, f32)], // p in the paper
    controls_q: &[(f32, f32)], // q in the paper
    point: (f32, f32),         // v in the paper
) -> (f32, f32) {
    let v = Point::from(point);
    let sqr_dist = |p: Point| (p - v).sqr_norm();

    // The weight of a given control point depends on its distance to the current point.
    // CAREFUL: this w can go to infinity.
    let weight = |pt| 1.0 / sqr_dist(pt);
    let w_all: Vec<_> = controls_p.iter().map(|&p| weight(p.into())).collect();
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

    // Compute the centroid p*.
    let wp_star_sum: Point = w_all
        .iter()
        .zip(controls_p)
        .map(|(&w, &p)| w * Point::from(p))
        .sum();
    let p_star = (1.0 / w_sum) * wp_star_sum;

    // Compute the centroid q*.
    let wq_star_sum: Point = w_all
        .iter()
        .zip(controls_q)
        .map(|(&w, &q)| w * Point::from(q))
        .sum();
    let q_star = (1.0 / w_sum) * wq_star_sum;

    // Compute p_hat.
    let p_hat: Vec<Point> = controls_p
        .iter()
        .map(|&p| Point::from(p) - p_star)
        .collect();

    // Compute q_hat.
    let q_hat: Vec<Point> = controls_q
        .iter()
        .map(|&q| Point::from(q) - q_star)
        .collect();

    // Compute mu_r.
    let mu_r_vec: Point = w_all
        .iter()
        .zip(&p_hat)
        .zip(&q_hat)
        .map(|((&wi, pi), qi)| {
            let pi_perp = Point { x: -pi.y, y: pi.x };
            Point {
                x: wi * qi.dot(*pi),
                y: wi * qi.dot(pi_perp),
            }
        })
        .sum();
    let mu_r = mu_r_vec.sqr_norm().sqrt();

    // Compute M (eq 6)
    let m: Mat2 = w_all
        .iter()
        .zip(&p_hat)
        .zip(&q_hat)
        .map(|((&wi, pi), qi)| {
            let p_mat = Mat2 {
                m11: pi.x,
                m12: pi.y,
                m21: pi.y,
                m22: -pi.x,
            };
            let q_mat = Mat2 {
                m11: qi.x,
                m21: qi.y,
                m12: qi.y,
                m22: -qi.x,
            };
            wi * p_mat * q_mat
        })
        .sum();
    let m = (1.0 / mu_r) * m;

    // Finally compute the projection of our original point (eq 3).
    ((v - p_star).transpose_mul(m) + q_star).into()
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
    /// 0
    fn zero() -> Self {
        Self { x: 0.0, y: 0.0 }
    }

    /// Dot product with another point.
    fn dot(self, rhs: Self) -> f32 {
        self.x * rhs.x + self.y * rhs.y
    }

    /// Square norm.
    fn sqr_norm(self) -> f32 {
        self.x * self.x + self.y * self.y
    }

    /// Create a 2x2 matrix from a 2x1 point
    fn times_transpose(self, rhs: Self) -> Mat2 {
        Mat2 {
            m11: self.x * rhs.x,
            m21: self.y * rhs.x,
            m12: self.x * rhs.y,
            m22: self.y * rhs.y,
        }
    }

    /// Multiply with a Mat2 on the right.
    /// Returns a Point even though it should be a line vector (no big deal).
    fn transpose_mul(self, rhs: Mat2) -> Self {
        Self {
            x: rhs.m11 * self.x + rhs.m21 * self.y,
            y: rhs.m12 * self.x + rhs.m22 * self.y,
        }
    }
}

// Convert from (x,y) to Point { x, y }
impl From<(f32, f32)> for Point {
    fn from((x, y): (f32, f32)) -> Self {
        Point { x, y }
    }
}

// Convert from Point { x, y } to (x,y)
impl Into<(f32, f32)> for Point {
    fn into(self) -> (f32, f32) {
        (self.x, self.y)
    }
}

// Add two points
impl Add for Point {
    type Output = Self;
    fn add(self, rhs: Self) -> Self::Output {
        Self {
            x: self.x + rhs.x,
            y: self.y + rhs.y,
        }
    }
}

// Substract a point
impl Sub for Point {
    type Output = Self;
    fn sub(self, rhs: Self) -> Self::Output {
        Self {
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

// Sum an iterator of points
impl Sum for Point {
    fn sum<I: Iterator<Item = Self>>(iter: I) -> Self {
        iter.fold(Self::zero(), |s, p| s + p)
    }
}

// 2x2 matrix helper ###########################################################
// That's to avoid a dependency on a heavy package such as nalgebra

/// 2x2 Matrix with the following coefficients.
///
/// | m11  m12 |
/// | m21  m22 |
#[derive(Clone, Copy)]
struct Mat2 {
    m11: f32,
    m21: f32,
    m12: f32,
    m22: f32,
}

impl Mat2 {
    /// 0
    fn zero() -> Self {
        Self {
            m11: 0.0,
            m21: 0.0,
            m12: 0.0,
            m22: 0.0,
        }
    }

    /// Determinant
    fn det(self) -> f32 {
        self.m11 * self.m22 - self.m21 * self.m12
    }

    /// Inverse of a matrix (does not check if det is 0)
    fn inv(self) -> Self {
        1.0 / self.det()
            * Self {
                m11: self.m22,
                m21: -self.m21,
                m12: -self.m12,
                m22: self.m11,
            }
    }
}

// Add two matrices
impl Add for Mat2 {
    type Output = Self;
    fn add(self, rhs: Self) -> Self::Output {
        Self {
            m11: self.m11 + rhs.m11,
            m21: self.m21 + rhs.m21,
            m12: self.m12 + rhs.m12,
            m22: self.m22 + rhs.m22,
        }
    }
}

// Scalar multiplication
impl Mul<Mat2> for f32 {
    type Output = Mat2;
    fn mul(self, rhs: Mat2) -> Self::Output {
        Mat2 {
            m11: self * rhs.m11,
            m21: self * rhs.m21,
            m12: self * rhs.m12,
            m22: self * rhs.m22,
        }
    }
}

// Matrix multiplication
impl Mul for Mat2 {
    type Output = Self;
    fn mul(self, rhs: Self) -> Self::Output {
        Mat2 {
            m11: self.m11 * rhs.m11 + self.m12 * rhs.m21,
            m21: self.m21 * rhs.m11 + self.m22 * rhs.m21,
            m12: self.m11 * rhs.m12 + self.m12 * rhs.m22,
            m22: self.m21 * rhs.m12 + self.m22 * rhs.m22,
        }
    }
}

// Sum an iterator of matrices
impl Sum for Mat2 {
    fn sum<I: Iterator<Item = Self>>(iter: I) -> Self {
        iter.fold(Self::zero(), |s, m| s + m)
    }
}
