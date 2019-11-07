// vim: set expandtab softtabstop=4 tabstop=4 shiftwidth=4:

// Code heavily based on https://www.geometrictools.com/
// which is distributed under the Boost Software License, Version 1.0.
//
//  Relevant files:
//   Include/Mathematics/GteVector.h
//   Include/Mathematics/GteVector2.h
//   Include/Mathematics/GteLine.h
//   Include/Mathematics/GteIntrIntervals.h
//   Include/Mathematics/GteIntrLine2Line2.h
//   Include/Mathematics/GteIntrSegment2Segment2.h
//
// Kornilios Kourtis <kornilios@gmail.com>
//
// NB #1: Not sure what the implication of not having a full order for
// floating points in rust is.
//
// NB #2: In general, the code is not very robust wrt floating point operations
// and geometry.

use std;
use std::f64::consts::PI;

pub trait Vec2 {
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
        x.powi(2) + y.powi(2)
    }

    fn apply<F>(&mut self, mut f: F) where F: FnMut(&mut f64) {
        let (x,y) = self.get_xy_mut();
        f(x);
        f(y);
    }

    fn xform<T>(&self) -> T where T: Vec2 {
        let (x,y) = self.get_xy();
        T::from_xy(*x, *y)
    }

    fn distance<T>(&self, p: T) -> f64 where T: Vec2 {
        let (x1, y1) = self.get_xy();
        let (x2, y2) = p.get_xy();
        return ((x2 - x1).powi(2) + (y2 - y1).powi(2)).sqrt();
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

// GteIntrIntervals.h:
// The intervals are [u0,u1] and [v0,v1], where u0 <= u1 and v0 <= v1, and
// where the endpoints are any finite floating-point numbers.  Degenerate
// intervals are allowed (u0 = u1 or v0 = v1).  The queries do not perform
// validation on the input intervals.
pub struct Interval(f64, f64);

pub enum IntervalIntersection {
    None,
    Interval(Interval),
}

impl Interval {
    pub fn intersection(&self, int: &Interval) -> IntervalIntersection {
        let &Interval(ref x0, ref x1) = self;
        let &Interval(ref y0, ref y1) = self;

        // x0       x1  y0      y1
        // |--------|   |-------|
        if x1 < y0 {
            return IntervalIntersection::None
        }

        // y0       y1  x0      x1
        // |--------|   |-------|
        if y1 < x0 {
            return IntervalIntersection::None
        }

        //    x0        x1
        //    |---------|
        //            |---------|
        //            y0        y1
        //
        //    y0        y1
        //    |---------|
        //         |---------|
        //        x0        x1
        let i0 = x0.max(*y0);
        let i1 = x1.max(*y1);
        IntervalIntersection::Interval(Interval(i0, i1))
    }
}

#[derive(Clone,Copy,Debug,PartialEq)]
pub struct Point {
    pub x: f64,
    pub y: f64,
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

impl std::ops::Div<f64> for Point {
    type Output = Point;
    fn div(self, s: f64) -> Point { Point {x: self.x / s, y: self.y / s} }
}

impl std::ops::Div<Point> for f64 {
    type Output = Point;
    fn div(self, p: Point) -> Point { p/self }
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
    Point(Point, f64, f64), // point and line parameters
    Parallel,
    Same,
}

impl LineIntersection {
    pub fn close(&self, x: &LineIntersection) -> bool {
        match (self, x) {
            (&LineIntersection::Parallel, &LineIntersection::Parallel) => true,
            (&LineIntersection::Same, &LineIntersection::Same) => true,
            (&LineIntersection::Point(ref p1, _, _), &LineIntersection::Point(ref p2, _, _)) => p1.close(p2),
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
            let dp_diff_d0 = diff.dot_perp(line0.direction);
            let dp_diff_d1 = diff.dot_perp(line1.direction);
            let s0 = dp_diff_d1 / dp_d0_d1;
            let s1 = dp_diff_d0 / dp_d0_d1;
            return LineIntersection::Point(line0.get_point(s0), s0, s1)
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
        match res {
            LineIntersection::Point(p, _, _) => assert_eq!(p, Point{x: 0.0, y: 0.0}),
            _ => assert!(false),
        };
    }

    {
        let l1 = Line::from_p_angle(Point { x: -1., y: 0. }, 0.);
        let l2 = Line::from_p_angle(Point { x: 0., y: 1. }, PI / 2.0);
        let res = l1.intersection(&l2);
        println!("l1:{:?}\nl2:{:?}\nintersection:{:?}", l1, l2, res);
        match res {
            LineIntersection::Point(p, _, _) => assert!(p.close(&Point{x: 0.0, y: 0.0})),
            _ => assert!(false),
        };
    }
}

// A segment is represented by:
//  (1-t)*P0 + t*P1, 0 <= t <= 1
// Where P0, P1 are the endpoints of the segment
#[derive(Debug,Clone,PartialEq)]
pub struct Segment(pub Point,pub Point);

// from Include/Mathematics/GteIntrSegment2Segment2.h:
// Some algorithms prefer a centered representation that is similar to how
// oriented bounding boxes are defined.  This representation is:
//
// C + s*D, where:
//  C = (P0 + P1)/2 is the center of the segment,
//  D = (P1 - P0)/|P1 - P0| is a unit-length direction vector for the segment, and
//  |t| <= e.  The value e = |P1 - P0|/2 is the extent (or radius or half-length) of the segment.
#[derive(Debug)]
pub struct SegmentCentered {
    center: Point,
    direction: [f64; 2],
    extent: f64,
}

impl SegmentCentered {
    pub fn get_line(&self) -> Line {
        Line { origin: self.center, direction: self.direction }
    }
}

#[derive(Debug,PartialEq)]
pub enum SegmentIntersection {
    None,
    Point(Point),
    Overlap(Segment),
}

impl Segment {

    pub fn angle(&self) -> f64 {
        let &Segment(ref a, ref b) = self;
        (b.y - a.y).atan2(b.x - a.x)
    }

    pub fn to_vector<T>(&self) -> T where T: Vec2 {
        let &Segment(ref a, ref b) = self;
        T::from_xy(b.x - a.x, b.y - a.y)
    }

    pub fn get_centered(&self) -> SegmentCentered {
        let &Segment(p0,p1) = self;
        let cent = (p0 + p1) / 2.;
        let mut diff = p1 - p0;
        let size = diff.dot_self().sqrt();
        diff.normalize();
        SegmentCentered {
            center: cent,
            direction: diff.xform(),
            extent: size / 2.
        }
    }

    pub fn intersection_line(&self, line: &Line) -> SegmentIntersection {
        let seg0_c = self.get_centered();
        let line0 = seg0_c.get_line();

        match line0.intersection(&line) {
            LineIntersection::Parallel => SegmentIntersection::None,
            LineIntersection::Same => SegmentIntersection::Overlap(self.clone()),
            LineIntersection::Point(ref p, ref s0, ref s1) => {
                if s0.abs() <= seg0_c.extent {
                    SegmentIntersection::Point(*p)
                } else {
                    SegmentIntersection::None
                }
            },

        }
    }


    pub fn intersection(&self, seg1: &Segment) -> SegmentIntersection {
        let seg0_c = self.get_centered();
        let seg1_c = seg1.get_centered();
        let line0 = seg0_c.get_line();
        let line1 = seg1_c.get_line();

        match line0.intersection(&line1) {
            LineIntersection::Parallel => SegmentIntersection::None,
            LineIntersection::Point(ref p, ref s0, ref s1) => {
                if s0.abs() <= seg0_c.extent && s1.abs() <= seg1_c.extent {
                    SegmentIntersection::Point(*p)
                } else {
                    SegmentIntersection::None
                }
            },
            LineIntersection::Same => {
                // Compute the location of segment1 endpoints relative to segment0.
                let diff = seg1_c.center - seg0_c.center;
                let t = seg0_c.direction.dot(&diff);

                // Get the parameter intervals of the segments relative to segment0.
                let interval0 = Interval(-seg0_c.extent, seg0_c.extent);
                let interval1 = Interval(t - seg1_c.extent, t + seg1_c.extent);
                match interval0.intersection(&interval1) {
                    IntervalIntersection::None => SegmentIntersection::None,
                    IntervalIntersection::Interval(Interval(p1, p2)) => {
                        if p1 == p2 {
                            let x : Point = seg0_c.direction.xform();
                            let sp = seg0_c.center + p1*x;
                            SegmentIntersection::Point(sp)
                        } else {
                            let x : Point = seg0_c.direction.xform();
                            let sp1 = seg0_c.center + p1*x;
                            let sp2 = seg0_c.center + p2*x;
                            SegmentIntersection::Overlap(Segment(sp1,sp2))
                        }
                    },
                }
            },
        }
    }
}


#[test]
pub fn t_segment_intersection() {
    let seg1 = Segment(Point{ x: -1.0, y: 0.0}, Point{ x: 1.0, y: 0.0});
    let seg2 = Segment(Point{ y: -1.0, x: 0.0}, Point{ y: 1.0, x: 0.0});
    let i = seg1.intersection(&seg2);
    assert!(i.eq(&SegmentIntersection::Point(Point{ x: 0.0, y: 0.0 })) == true);
}
