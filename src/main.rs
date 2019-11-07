// vim: set expandtab softtabstop=4 tabstop=4 shiftwidth=4 nowrap:
extern crate piston;
extern crate graphics;
extern crate glutin_window;
extern crate opengl_graphics;
extern crate rand;

use piston::window::WindowSettings;
use piston::event_loop::*;
use piston::input::*;
use glutin_window::GlutinWindow as Window;
use opengl_graphics::{ GlGraphics, OpenGL };
use std::f64::consts::PI;

mod util2d;

use util2d::Vec2;

const SCREEN_X: u32 = 800; //1024;
const SCREEN_Y: u32 = 600; //768;
const BOIDS_NR: usize = 100;

const MAX_V:        f64 = 100.0;
const ACCELERATION: f64 = 1.0;
const ROT_SPEED:    f64 = 1.0*PI;

pub struct Body {
    x: f64,
    y: f64,
    v: f64, // velocity
    a: f64, // angle
    // input: acceleration and rotation (-1, 0, 1)
    in_acc: i32,
    in_rot: i32
}

pub struct Boid { b: Body }
pub struct User { b: Body }

impl Body {

    pub fn accel(&mut self, x_: bool)  {
        let x = !x_; // up is lower
        self.in_acc = match self.in_acc {
            1  => if x {1} else {0},
            0  => if x {1} else {-1},
           -1  => if x {0} else {-1},
            _  => { panic!("Unexpected error") }
        };
    }

    pub fn rotate(&mut self, x: bool)  {
        self.in_rot = match self.in_rot {
            1  => if x {1} else {0},
            0  => if x {1} else {-1},
           -1  => if x {0} else {-1},
            _  => { panic!("Unexpected error") }
        };
    }

    pub fn update(&mut self, dt: f64) {

        self.v += (self.in_acc as f64)*ACCELERATION*dt;
        if self.v > MAX_V {
            self.v = MAX_V
        }
        self.a += (self.in_rot as f64)*ROT_SPEED*dt;

        self.x += self.a.cos()*self.v;
        self.y += self.a.sin()*self.v;
    }

    // is body out of bounds?
    pub fn is_oob(&self) -> bool {
        self.x < 0.0 ||
        self.x > (SCREEN_X as f64) ||
        self.y < 0.0 ||
        self.y > (SCREEN_Y as f64)
    }
}

pub struct State {
    rng: rand::ThreadRng,
    user: User,
    boids: Vec<Boid>,
    reset: bool,
    pause: bool,
}


impl Boid {

    /// adjust Boid to avoid walls.
    fn avoid_walls(&mut self) {
        // first we try to find out, which wall are we heading to.
        // http://stackoverflow.com/questions/7586063/how-to-calculate-the-angle-between-a-line-and-the-horizontal-axis
        let sx = SCREEN_X as f64;
        let sy = SCREEN_Y as f64;

        // Get all the walls points.(Currently, the walls are rectangular, but I
        // think this should work for any convex shape.)
        let wall_points = vec![(0.0, sy), (0.0, 0.0), (sx, 0.0), (sx, sy)];
        let mut wall_angles = wall_points.iter().map(|&(ref px, ref py)|
           // for each point, compute the angle from the boid
           ((px,py), (py - self.b.y).atan2(px - self.b.x))
        ).map(|(p,r)| {
            // make the angle positive if it's not
            (p, if r < 0.0 { r + 2.0*PI} else { r })
        }).collect::<Vec<_>>();

        wall_angles.sort_by(|&(_, ref r1), &(_, ref r2)|
            match r1.partial_cmp(r2) {
                Some(o) => o,
                None    => panic!("Die a horrible death!")
            }
        );

        let wpoints_nr = wall_angles.len();
        // find the position of the first angle that's larger than the boid's
        // angle
        let line_points = match wall_angles.iter().position(|&(_, ref a)|
            match a.partial_cmp(&self.b.a) {
                None => panic!("Die a horrible death, part 2!"),
                Some(std::cmp::Ordering::Greater) => true,
                _ => false,
            }
        ) {
            Some(0) => (wall_angles[wpoints_nr-1], wall_angles[0]),
            None    => (wall_angles[wpoints_nr-1], wall_angles[0]),
            Some(i) => (wall_angles[i-1], wall_angles[i]),
        };

        let wall = util2d::Segment(
            util2d::Point { x: *((line_points.0).0).0, y: *((line_points.0).0).1 },
            util2d::Point { x: *((line_points.1).0).0, y: *((line_points.1).0).1 },
        );

        // OK now we have the wall that we are going to hit (wall), next we need
        // to compute our distance from it.
        let boid_p = util2d::Point {x: self.b.x, y: self.b.y };
        let direction = util2d::Line::from_p_angle(boid_p, self.b.a);
        let wall_p = match wall.intersection_line(&direction) {
                util2d::SegmentIntersection::Point(p) => p,
                _ => panic!("Unexpected enum variant"),
        };

        let wall_distance = wall_p.distance(boid_p);
        // println!("wall={:?} wall_p={:?} boid_p={:?} SCREEN:{:?} wall distance={:?}", wall, wall_p, boid_p, (SCREEN_X, SCREEN_Y), wall_distance);
        if wall_distance < 100.0 {
            // We need to determine the direction to turn
            let dir_v : util2d::Point = wall_p - boid_p;
            let wall_v : util2d::Point = wall.to_vector();
            // println!("dir_v={:?} wall_v={:?}" , dir_v, wall_v);
            let dot = dir_v.dot(&wall_v);
            // println!("Angle={:?} dot={:?}", self.b.a*180.0/PI, dot);
            let turn = {
                if dot < 0.0 {
                    -20.0*PI/180.0
                } else {
                    -20.0*PI/180.0
                }
            };
            // println!("Turning={:?}", turn);
            self.b.a += turn;
        }

        while self.b.a < 0.0 {
            self.b.a += 2.0*PI;
        }

        while self.b.a > 2.0*PI {
            self.b.a -= 2.0*PI;
        }
    }



    pub fn run_ai(&mut self) {
        self.b.in_rot = 0;

        self.avoid_walls();

        let d_top   = self.b.y;
        let d_bot   = (SCREEN_Y as f64) - self.b.y;
        let d_left  = self.b.x;
        let d_right = (SCREEN_X as f64) - self.b.x;
        let d_ver   = if d_top < d_bot { d_top } else { d_bot };
        let d_hor   = if d_left < d_right { d_left } else { d_right };
        let d_min   = if d_hor < d_ver { d_hor } else { d_ver };

        let rads = self.b.a;
        assert!(rads >= 0.0 && rads <= 2.0*PI);
        let q = (rads / (PI/2.0)).round() as u32;

        self.b.in_rot = if d_min > 100.0 {
            0
        } else if d_min == d_top {
            //println!("q={:?} rads={:?}", q, rads);
            match q {
                1 => -1,
                2 => 1,
                _ => 0
            }
        } else if d_min == d_bot {
            0
        } else if d_min == d_left {
            0
        } else if d_min == d_right {
            0
        } else { 0 }
    }
}

pub fn getrand<R,T>(rng: &mut R, s: T, e: T) -> T
    where R: rand::Rng,
          T: std::cmp::PartialOrd + rand::distributions::range::SampleRange {
    use rand::distributions::IndependentSample;
    let range = rand::distributions::Range::new(s,e);
    range.ind_sample(rng)
}

pub fn rand_body<R: rand::Rng>(rng: &mut R) -> Body {

    let rand_a = getrand(rng, 0.0, 2.0*std::f64::consts::PI);
    // let rand_a = 0.6318703870433586;
    // println!("rand_a={:?}", rand_a);
    Body { x: (SCREEN_X / 2) as f64 //getrand(rng, 0, SCREEN_X -10) as f64
         , y: (SCREEN_Y / 2) as f64 //getrand(rng, 0, SCREEN_Y -10) as f64
         , v: 1.0 // getrand(rng, 0.0, 5.5)
         // , a: PI*1.0/180.0 //getrand(rng, 0.0, 2.0*std::f64::consts::PI)
         , a: rand_a
         , in_rot:  0
         , in_acc:  0 }
}

pub fn user_initial_body() -> Body {
    Body { x: (SCREEN_X / 2) as f64
         , y: (SCREEN_Y / 2) as f64
         , v:     0.0
         , a:     0.0
         , in_rot:  0
         , in_acc:  0 }
}

impl State {
    pub fn new() -> State {
        let mut rng = rand::thread_rng();

        let mut boids = Vec::new();
        for _ in 0..BOIDS_NR {
            boids.push(Boid { b: rand_body(&mut rng) });
        };

        State {
            rng: rng,
            user: User { b: user_initial_body() },
            boids: boids,
            reset: false,
            pause: false,
        }
    }

    pub fn reset(&mut self) {
        for boid in self.boids.iter_mut() {
            boid.b = rand_body(&mut self.rng);
        }
        self.user.b = user_initial_body();
    }

    pub fn update(&mut self, dt: f64) {

        if self.reset {
            self.reset();
            self.reset = false;
            return
        }

        if self.pause {
            return
        }

        self.user.b.update(dt);
        for boid in self.boids.iter_mut().filter(|x| !x.b.is_oob() ) {
            boid.run_ai();
            boid.b.update(dt)
        }
    }

}

pub fn render(c: graphics::Context, gl: &mut GlGraphics, state: &State) {
    use graphics::*;
    let bg_c   = color::hex("2b114c");
    let usr_c  = color::hex("f49842");
    let boid_c = color::hex("89f442");
    const USER_POLY: &'static [[f64; 2]] = &[
        [ 0.0,  6.67],
        [-2.5, -3.34],
        [ 2.5, -3.34]
    ];
    // Clear the screen.
    clear(bg_c, gl);

    // render user
    let ref user = &state.user;
    let user_trans = c.transform
                      .trans(user.b.x, user.b.y)
                      .rot_rad(user.b.a - PI/2.0)
                      .scale(1.0,1.0);
    polygon(usr_c, USER_POLY, user_trans, gl);

    for boid in state.boids.iter() {
        let boid_trans = c.transform
                          .trans(boid.b.x, boid.b.y)
                          .rot_rad(boid.b.a - PI/2.0)
                          .scale(1.0,1.0);
        polygon(boid_c, USER_POLY, boid_trans, gl);
    }
}


pub fn main() {
    let opengl = OpenGL::V3_2;

    // Create an Glutin window.
    let mut window: Window = WindowSettings::new("_flock_", [SCREEN_X, SCREEN_Y])
        .opengl(opengl)
        .exit_on_esc(true)
        .build()
        .unwrap();

    let mut st = State::new();
    let mut gl = GlGraphics::new(opengl);
    let mut events = Events::new(EventSettings::new());

    while let Some(e) = events.next(&mut window) {
        match e {
            Input::Render(r) => { gl.draw(r.viewport(), |c, g| render(c, g, &st)) }
            Input::Update(u) => { st.update(u.dt) }
            Input::Press(b)  => {
                match b {
                    Button::Keyboard(Key::Up)    => { st.user.b.accel(false)  },
                    Button::Keyboard(Key::Down)  => { st.user.b.accel(true)   },
                    Button::Keyboard(Key::Right) => { st.user.b.rotate(true)  },
                    Button::Keyboard(Key::Left)  => { st.user.b.rotate(false) },
                    Button::Keyboard(Key::Left)  => { st.user.b.rotate(false) },
                    Button::Keyboard(Key::R)     => { st.reset = true },
                    Button::Keyboard(Key::Space) => { st.pause = !st.pause },
                    _ => {}
                }
            }
            Input::Release(b)  => {
                match b {
                    Button::Keyboard(Key::Up)    => { st.user.b.in_acc = 0 },
                    Button::Keyboard(Key::Down)  => { st.user.b.in_acc = 0 },
                    Button::Keyboard(Key::Right) => { st.user.b.in_rot = 0 },
                    Button::Keyboard(Key::Left)  => { st.user.b.in_rot = 0 },
                    _ => {}
                }
            }
            _ => {}
        }
    }
}
