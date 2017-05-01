// vim: set expandtab softtabstop=4 tabstop=4 shiftwidth=4:

use std;

#[derive(Clone,Copy,Debug)]
pub struct Point {
    x: f64,
    y: f64,
}

impl Point {
    pub fn dot(&self, p: &Point) -> f64 {
        self.x*p.x + self.y*p.y
    }
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

#[derive(Debug)]
pub struct Segment {
    points: [Point; 2]
}

#[derive(Debug)]
pub enum SegmentIntersection {
    None,
    Point(Point),
    Overlap(Segment),
}

impl Segment {
    pub fn new(p0: Point, p1: Point) -> Self {
        Segment { points: [p0, p1] }
    }

    pub fn from_tuples(a: (f64, f64), b: (f64, f64)) -> Self {
        Segment { points: [
            Point { x: a.0, y: a.1},
            Point { x: b.0, y: b.1}
        ]}
    }

    // see: Geometric tools for computer graphics, Schneider and Eberly, 7.1
    //
    // Segments:
    //   s1.points[0] + s*s1.points[1], s in [0,1]
    //   s2.points[0] + t*s2.points[1], t in [0,1]
    //
    // NB: not sure what the implication of not having a complete order for
    // floating points in rust here is.
    pub fn intersection(s1: &Segment, s2: &Segment) -> SegmentIntersection {

        // Rename to the variables name used in the book, to avoid errors
        // segments:
        //  p0 + s*d0, s in [0,1]
        //  p1 + t*d1, t in [0,1]
        let p0 = s1.points[0]; let d0 = s1.points[1];
        let p1 = s2.points[0]; let d1 = s2.points[1];

        let e = p1 - p0;
        let kross = d0.x*d1.y - d0.y*d1.x;
        let sqr_kross = kross.powi(2);
        let sqr_len_0 = d0.x.powi(2) + d0.y.powi(2);
        let sqr_len_1 = d1.x.powi(2) + d1.y.powi(2);
        // if kross != 0, the lines intersect
        let sqr_epsillon = 1E-20_f64;
        if sqr_kross > sqr_epsillon*sqr_len_0*sqr_len_1 {
            // Lines of segments are not parallel
            let s = (e.x*d1.y - e.y*d1.x) / kross;
            if s < 0. || s > 1. { // lines intersection not a point on (p0 + s*d0)
                return SegmentIntersection::None
            }

            let t = (e.x*d0.y - e.y*d0.x) / kross;
            if t < 0. || t > 1. { // lines intersection not a point on (p1 + s*d1)
                return SegmentIntersection::None
            }

            return SegmentIntersection::Point(p0 + s*d0)
        }

        // Lines of segments are parallel
        let sqr_len_e = e.x.powi(2) + e.y.powi(2);
        let kross_e = e.x*d0.y - e.y*d0.x;
        let sqr_kross_e = kross_e.powi(2);
        if sqr_kross_e > sqr_epsillon*sqr_len_0*sqr_len_e {
                return SegmentIntersection::None
        }
        // lines of the segments are the same.
        // Need to test for overlap of segments
        let s0 = d0.dot(&e) / sqr_len_0;
        let s1 = s0 + d0.dot(&d1) / sqr_len_0;
        let (smin, smax) = if s0 < s1 { (s0,s1) } else { (s1,s0) };
        {
            // Rename to the variables name used in the book, to avoid errors
            // The intersection of two intervals [u0,u1] where:
            //  u0 < u1
            //  v0 < v1
            let (u0,u1) = (0., 1.);
            let (v0,v1) = (smin, smax);
            if u1 < v0 || u0 > v1 {
                SegmentIntersection::None
            } else if u1 > v0 {
                if u0 < v1 {
                    let w0 = if u0 < v0 {v0} else {u0};
                    let w1 = if u1 > v1 {v1} else {u1};
                    SegmentIntersection::Overlap(Segment { points: [p0 + w0*d0, p0 + w1*d0] })
                } else {
                    let w0 = u0;
                    SegmentIntersection::Point(p0 + w0*d0)
                }
            } else {
                let w0 = u1;
                SegmentIntersection::Point(p0 + w0*d0)
            }
        }
    }
}

type P = Point;
type S = Segment;

fn intersection_test(l1: ((f64,f64), (f64, f64)),
                     l2: ((f64, f64), (f64,f64))) {
    let s0 = S::from_tuples(l1.0,l1.1);
    let s1 = S::from_tuples(l2.0,l2.1);
    let res = S::intersection(&s0, &s1);
    println!("s0:{:?}\ns1:{:?}\nintersection:{:?}", s0, s1, res);
}

#[test]
fn t() {

    intersection_test(
        ((0.,-1.),(0.,1.)),
        ((-1.,0.),(1.,0.))
    );

    intersection_test(
        ((0.,0.),(0.,2.)),
        ((0.,0.),(0.,1.))
    );

    intersection_test(
        ((1.,0.),(1.,2.)),
        ((0.,0.),(0.,1.))
    );
}
