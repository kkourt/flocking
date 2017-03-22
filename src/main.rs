// vim: set expandtab softtabstop=4 tabstop=4 shiftwidth=4:
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

const SCREEN_X: u32 = 1024;
const SCREEN_Y: u32 = 768;
const BOIDS_NR: usize = 1;

const MAX_V:        f64 = 100.0;
const ACCELERATION: f64 = 1.0;
const ROT_SPEED:    f64 = 1.0*std::f64::consts::PI;

pub struct Body { x: f64
                , y: f64
                , v: f64 // velocity
                , a: f64 // angle
                // input: acceleration and rotation (-1, 0, 1)
                , in_acc: i32
                , in_rot: i32 }

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

        self.x -= self.a.sin()*self.v;
        self.y += self.a.cos()*self.v;
    }
}

pub struct State {
    rng: rand::ThreadRng,
    user: User,
    boids: Vec<Boid>,
}

impl Boid {
    pub fn run_ai(&mut self) {
        self.b.in_rot = 0;

        let d_top   = self.b.y;
        let d_bot   = (SCREEN_Y as f64) - self.b.y;
        let d_left  = self.b.x;
        let d_right = (SCREEN_X as f64) - self.b.x;
        let d_ver   = if d_top < d_bot { d_top } else { d_bot };
        let d_hor   = if d_left < d_right { d_left } else { d_right };
        let d_min   = if d_hor < d_ver { d_hor } else { d_ver };

        let rads = self.b.a;
        assert!(rads >= 0.0 && rads <= 2.0*std::f64::consts::PI);
        let q = (rads / (std::f64::consts::PI/2.0)).round() as u32;

        self.b.in_rot = if d_min > 100.0 {
            0
        } else if d_min == d_top {
            println!("q={:?} rads={:?}", q, rads);
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
    Body { x: getrand(rng, 0, SCREEN_X -10) as f64
         , y: getrand(rng, 0, SCREEN_Y -10) as f64
         , v: 1.0 // getrand(rng, 0.0, 5.5)
         , a: 3.14 //getrand(rng, 0.0, 2.0*std::f64::consts::PI)
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
            user: User { b: Body { x: (SCREEN_X / 2) as f64
                                 , y: (SCREEN_Y / 2) as f64
                                 , v:     0.0
                                 , a:     0.0
                                 , in_rot:  0
                                 , in_acc:  0 } },
            boids: boids
        }
    }

    pub fn update(&mut self, dt: f64) {
        self.user.b.update(dt);
        for boid in self.boids.iter_mut() {
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
                      .rot_rad(user.b.a)
                      .scale(1.0,1.0);
    polygon(usr_c, USER_POLY, user_trans, gl);

    for boid in state.boids.iter() {
        let boid_trans = c.transform
                          .trans(boid.b.x, boid.b.y)
                          .rot_rad(boid.b.a)
                          .scale(1.0,1.0);
        polygon(boid_c, USER_POLY, boid_trans, gl);
    }
}


pub fn main() {
    let opengl = OpenGL::V3_2;

    // Create an Glutin window.
    let mut window: Window = WindowSettings::new("flock", [SCREEN_X, SCREEN_Y])
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