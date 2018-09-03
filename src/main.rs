extern crate bvh;
extern crate nalgebra;
extern crate png;
extern crate rand;

use std::path::Path;
use std::fs::File;
use std::io::BufWriter;
use png::HasParameters;
use nalgebra::{Vector3};
use bvh::nalgebra::{Point2, Point3};
use bvh::ray::Ray;
use std::f64::consts::PI;

const TWO_PI: f32 = 2.0 * PI as f32;

fn main() {
    let path = Path::new(r"test.png");
    let file = File::create(path).unwrap();
    let w = &mut BufWriter::new(file);

    let width = 512;
    let height = 512;

    let mut encoder = png::Encoder::new(w, width as u32, height as u32);

    encoder.set(png::ColorType::RGBA).set(png::BitDepth::Eight);
    let mut writer = encoder.write_header().unwrap();


    let mut out_vec = vec![0; 4 * width * height];
    make_image(width, height, &mut out_vec);
    writer.write_image_data(&out_vec).unwrap();
}

fn make_image(width: usize, height: usize, out_vec: &mut [u8]) {
    let sphere_1 = Sphere::new(
        Point3::new(0.0, 0.0, 0.0),
        2.0,
    );

    let camera = Point3::new(0.0, 0.0, -5.0);

    let light = Sphere::new(
        Point3::new(0.0, 2.0, -5.0),
        1.0,
    );

    let wf = width as f32;
    let hf = height as f32;

    for i in 0..width * height {
        let offset = 4 * i;
        let x = i % width;
        let y = i / width;

        let mut r = 0.0;
        let mut g = 0.0;
        let mut b = 0.0;

        // Repeat 400 times for 400 samples.
        // Each sample is offset slightly for supersampling.
        for ss in 0..400 {
            let xoffs = 0.0025 * (ss % 20) as f32;
            let yoffs = 0.0025 * (ss / 20) as f32;
            let x_ratio = (x as f32 + xoffs - (wf / 2.0)) / wf;
            let y_ratio = (y as f32 + yoffs - (hf / 2.0)) / hf;
            let ray_dir = Vector3::new(x_ratio, y_ratio, 1.0);
            let ray = Ray::new(camera, ray_dir);
            let (rr, gg, bb) = trace_iterative(ray, &sphere_1, &light);
            r += rr;
            g += gg;
            b += bb;
        }
        let (r, g, b) = (r / 400.0, g / 400.0, b / 400.0);
        out_vec[offset]   = (r * 255.0) as u8;
        out_vec[offset+1] = (g * 255.0) as u8;
        out_vec[offset+2] = (b * 255.0) as u8;
        out_vec[offset+3] = 255;
    }
}

// c ranges from 0 to 1, output should range from 
fn scale_color(c: f32) -> f32 {
   (c * 100.0 + 1.0).log10() * 127.0
}

fn trace_iterative(ray: Ray, sphere: &Sphere, light: &Sphere) -> (f32, f32, f32) {

    let mut ray = ray;

    let mut frag_color = (1.0, 1.0, 1.0);

    for i in 0..2 {
        let hit = intersect(&ray, &sphere);
        let hit_light = intersect(&ray, &light);

        if let Some(idata) = hit_light {
            // We're done, return the light color
            return (
                frag_color.0 * 10.0,
                frag_color.1 * 10.0,
                frag_color.2 * 10.0);
        } else if let Some(idata) = hit {
            // Multiply by the surface color and bounce

            // Compute the surface color
            let u_tex = (idata.uv.x * 10.0).floor() as u8 % 2;
            let v_tex = (idata.uv.y * 10.0).floor() as u8 % 2;
            let mut color = (u_tex ^ v_tex) as f32;
            if color < 0.0 {
                color = 0.0;
            }
            
            // Compute the bounced ray
            ray = Ray::new(
                idata.point + 0.00001 * idata.normal, 
                random_vector_in_hemisphere(&idata.normal)
            );

            // Multiply
            frag_color = 
                (frag_color.0 * color,
                 frag_color.1 * color,
                 frag_color.2 * color);
        } else {
            // Light escaped into oblivion, give up
            break;
        }
    }

    return (
        frag_color.0 * 0.0, 
        frag_color.1 * 0.0, 
        frag_color.2 * 0.0);
}

fn random_vector_in_hemisphere(vec: &Vector3<f32>) -> Vector3<f32> {
    let u  = rand::random::<f32>();
    let v  = rand::random::<f32>();
    let th = TWO_PI * u;
    let ph = (2.0 * v - 1.0).acos();

    let mut new_vec = Vector3::new(
        th.cos() * ph.sin(),
        th.sin() * ph.sin(),
        ph.cos()
    );

    if new_vec.dot(vec) < 0.0 {
        new_vec *= -1.0;
    }
    new_vec
}

struct IntersectData {
    point: Point3<f32>,
    normal: Vector3<f32>,
    uv: Point2<f32>,
    dist: f32,
}

struct Sphere {
    center: Point3<f32>,
    radius: f32,
    radius2: f32,
}

impl Sphere {
    fn new(center: Point3<f32>, radius: f32) -> Self {
        Sphere {
            center: center,
            radius: radius,
            radius2: radius * radius
        }
    }
}

fn intersect(ray: &Ray, sphere: &Sphere) -> Option<IntersectData> {
    let mut t0;
    let mut t1;
    let dir = ray.direction;
    let radius2 = sphere.radius2;
    let l = sphere.center - ray.origin;
    let tca = l.dot(&dir);
    let d2 = l.dot(&l) - tca * tca;
    if d2 > radius2 { return None; };
    let thc = (radius2 - d2).sqrt();
    t0 = tca - thc;
    t1 = tca + thc;

    if t0 > t1 { ::std::mem::swap(&mut t0, &mut t1); };

    if t0 < 0.0 {
        t0 = t1;
        if t0 < 0.0 { return None; };
    }
    let t = t0;
    let phit = ray.origin + t * ray.direction;
    let normal = phit - sphere.center;

    // compute uv
    let u = normal.z.atan2(normal.x);
    let v = (normal.y / sphere.radius).acos();

    Some(IntersectData {
        point: phit,
        normal: normal.normalize(),
        uv: Point2::new(u / TWO_PI, v / TWO_PI),
        dist: t,
    })
}
