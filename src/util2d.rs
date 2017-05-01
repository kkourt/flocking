// vim: set expandtab softtabstop=4 tabstop=4 shiftwidth=4:

// Code heavily based on https://www.geometrictools.com/
// which is distributed under the Boost Software License, Version 1.0.
//
//  Relevant files:
//   Include/Mathematics/GteVector.h
//   Include/Mathematics/GteVector2.h
//   Include/Mathematics/GteLine.h
//   Include/Mathematics/GteIntrLine2Line2.h
//
// Kornilios Kourtis <kornilios@gmail.com>
//
// NB: not sure what the implication of not having a complete order for floating
// points in rust is.
//
// In general, the code is not very robust wrt floating point operations and
// geometry.

use std;
use std::f64::consts::PI;

trait Vec2 {
    fn get_xy(&self) -> (&f64, &f64);
    fn get_xy_mut(&mut self) -> (&mut f64, &mut f64);
    fn from_xy(x: f64, y: f64) -> Self;

    fn dot<T>(&self, vec2: &T) -> f64 where T: Vec2 {
        let (a0, a1) = self.get_xy();
        let (b0, b1) = vec2.get_xy();
        a0*b0 + a1*b1
    }

    // x.dot(x)
    fn dot_self(&self) -> f64 {
        let (x, y) = self.get_xy();
        x.powi(2) + x.powi(2)
    }

    fn apply<F>(&mut self, mut f: F) where F: FnMut(&mut f64) {
        let (x,y) = self.get_xy_mut();
        f(x);
        f(y);
    }

    fn normalize(&mut self) {
        // NB: In the GTE source code there is a robust parameter which follows
        // a different path
        let len = self.dot_self().sqrt();
        if len > 0.0 {
            self.apply(|x| *x = *x / len)
        } else {
            self.apply(|x| *x = 0.)
        }
    }


    // Compute the perpendicular using the formal determinant,
    //   perp = det{{e0,e1},{x0,x1}} = (x1,-x0)
    // where e0 = (1,0), e1 = (0,1), and v = (x0,x1).
    fn perp(&self) -> Self where Self: std::marker::Sized {
        let (x, y) = self.get_xy();
        Self::from_xy(y.clone(), -x.clone())
    }

    // Compute Dot((x0,x1),Perp(y0,y1)) = x0*y1 - x1*y0, where v0 = (x0,x1) and
    // v1 = (y0,y1).
    fn dot_perp<T: Vec2>(&self, v1: T) -> f64
        where Self: std::marker::Sized, T: Vec2 {
        self.dot(&v1.perp())
    }

    // NB: we could probably do better here
    fn close<T: Vec2>(&self, v: &T) -> bool {
        let eps = 1E-12_f64;
        let (a0, a1) = self.get_xy();
        let (b0, b1) = v.get_xy();
        if (a0 - a1).abs() > eps {
            false
        } else if (b0 - b1).abs() > eps {
            false
        } else {
            true
        }
    }
}

#[derive(Clone,Copy,Debug,PartialEq)]
pub struct Point {
    x: f64,
    y: f64,
}

impl Vec2 for Point {
    fn get_xy(&self) -> (&f64, &f64) { (&self.x, &self.y) }
    fn get_xy_mut(&mut self) -> (&mut f64, &mut f64) { (&mut self.x, &mut self.y) }
    fn from_xy(x: f64, y: f64) -> Self { Point { x: x, y: y} }
}

impl Vec2 for [f64; 2] {

    fn get_xy(&self) -> (&f64, &f64) {
        (&self[0], &self[1])
    }

    fn get_xy_mut(&mut self) -> (&mut f64, &mut f64) {
        unsafe {
            let ptr = self.as_mut_ptr();
            (&mut *ptr.offset(0), &mut *ptr.offset(1))
        }
    }

    fn from_xy(x: f64, y: f64) -> Self { [x,y] }
}

impl std::ops::Add for Point {
    type Output = Point;

    fn add(self, other: Point) -> Point {
        Point {x: self.x + other.x, y: self.y + other.y}
    }
}

impl std::ops::Sub for Point {
    type Output = Point;

    fn sub(self, other: Point) -> Point {
        Point {x: self.x - other.x, y: self.y - other.y}
    }
}

impl std::ops::Mul<f64> for Point {
    type Output = Point;
    fn mul(self, s: f64) -> Point { Point {x: self.x*s, y: self.y*s} }
}

impl std::ops::Mul<Point> for f64 {
    type Output = Point;
    fn mul(self, p: Point) -> Point { p*self }
}

// The line is represented by P + t*D, where:
//   P is an origin point
//   D is a unit-length direction vector
//   t is any real number
#[derive(Debug,PartialEq)]
pub struct Line {
    origin: Point,
    direction: [f64; 2],
}

#[derive(Debug,PartialEq)]
pub enum LineIntersection {
    Point(Point),
    Parallel,
    Same,
}

impl LineIntersection {
    pub fn close(&self, x: &LineIntersection) -> bool {
        match (self, x) {
            (&LineIntersection::Parallel, &LineIntersection::Parallel) => true,
            (&LineIntersection::Same, &LineIntersection::Same) => true,
            (&LineIntersection::Point(ref p1), &LineIntersection::Point(ref p2)) => p1.close(p2),
            _ => false,
        }
    }
}

impl Line {
    pub fn get_point(&self, t: f64) -> Point {
        Point {
            x: self.origin.x + t*self.direction[0],
            y: self.origin.y + t*self.direction[1],
        }
    }

    pub fn from_p_angle(p: Point, rads: f64) -> Self {
        Line { origin: p, direction: [rads.cos(), rads.sin()] }
    }

    pub fn from_angle(rads: f64) -> Self {
        Self::from_p_angle(Point{ x: 0., y: 0.}, rads)
    }


    // based on Include/Mathematics/GteIntrLine2Line2.h::FIQuery
    //
    // The intersection of the lines is the solution to:
    //    p0 + s0*d0 = p1 + s1*d1
    // => s0*d0 - s1*d1 = p1 - p0 = Q
    //
    // if DotPerp(d0,d1) == 0  => the lines are parallel
    // if DotPerp(d0,d1) == 0 && DotPerp(Q, d1) == 0 => the lines are the same
    // if DotPerp(d0,d1) != 0 => then the intersection point is:
    //    s0 = DotPerp(Q, D1))/DotPerp(D0, D1))
    // or
    //    s1 = DotPerp(Q, D0))/DotPerp(D0, D1))
    pub fn intersection(&self, line1: &Self) -> LineIntersection {
        let line0 = self;
        let mut diff = line1.origin - line0.origin;
        let dp_d0_d1 = line0.direction.dot_perp(line1.direction);
        if dp_d0_d1 != 0. { // lines are not parallel
            let dp_diff_d1 = diff.dot_perp(line1.direction);
            let s0 = dp_diff_d1 / dp_d0_d1;
            return LineIntersection::Point(line0.get_point(s0))
        }

        // Lines are parallel
        diff.normalize();
        let dp_diffN_d1 = diff.dot_perp(line1.direction);
        if dp_diffN_d1 != 0. {
            LineIntersection::Parallel
        } else {
            LineIntersection::Same
        }
    }
}

#[test]
pub fn t_line_intersection() {
    {
        let l1 = Line::from_p_angle(Point { x: 1., y: 0. }, 0.);
        let l2 = Line::from_p_angle(Point { x: 0., y: 0. }, 0.);
        let res = l1.intersection(&l2);
        println!("l1:{:?}\nl2:{:?}\nintersection:{:?}", l1, l2, res);
        assert_eq!(res, LineIntersection::Same);
    }

    {
        let l1 = Line::from_p_angle(Point { x: 1., y: 1. }, 0.);
        let l2 = Line::from_p_angle(Point { x: 0., y: 0. }, 0.);
        let res = l1.intersection(&l2);
        println!("l1:{:?}\nl2:{:?}\nintersection:{:?}", l1, l2, res);
        assert_eq!(res, LineIntersection::Parallel);
    }

    {
        let l1 = Line::from_p_angle(Point { x: 0., y: 0. }, 0.);
        let l2 = Line::from_p_angle(Point { x: 0., y: 0. }, PI / 2.0);
        let res = l1.intersection(&l2);
        println!("l1:{:?}\nl2:{:?}\nintersection:{:?}", l1, l2, res);
        assert_eq!(res, LineIntersection::Point(Point{x: 0., y: 0.}));
    }

    {
        let l1 = Line::from_p_angle(Point { x: -1., y: 0. }, 0.);
        let l2 = Line::from_p_angle(Point { x: 0., y: 1. }, PI / 2.0);
        let res = l1.intersection(&l2);
        println!("l1:{:?}\nl2:{:?}\nintersection:{:?}", l1, l2, res);
        assert!(LineIntersection::Point(Point{x: 0., y: 0.}).close(&res));
    }
}

#[derive(Debug)]
pub struct Segment {
    points: [Point; 2]
}


pub enum SegmentIntersection {
    None,
    Point(Point),
    Overlap(Segment),
}

