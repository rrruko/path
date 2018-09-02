extern crate bvh;
extern crate nalgebra;
extern crate png;

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

    let width = 1024;
    let height = 1024;

    let mut encoder = png::Encoder::new(w, width as u32, height as u32);

    encoder.set(png::ColorType::RGBA).set(png::BitDepth::Eight);
    let mut writer = encoder.write_header().unwrap();


    let mut out_vec = vec![0; 4 * width * height];
    make_image(width, height, &mut out_vec);
    writer.write_image_data(&out_vec).unwrap();
}

fn make_image(width: usize, height: usize, out_vec: &mut [u8]) {
    let sphere_1 = Sphere {
        center: Point3::new(0.0, 0.0, 1.0),
        radius: 1.0,
    };

    let camera = Point3::new(0.0, 0.0, -1.0);
    
    let wf = width as f32;
    let hf = height as f32;

    for i in 0..width * height {
        let offset = 4 * i;
        let x = i % width;
        let y = i / width;
        let x_ratio = (x as f32 - (wf / 2.0)) / wf;
        let y_ratio = (y as f32 - (hf / 2.0)) / hf;
        let ray_dir = Vector3::new(x_ratio, y_ratio, 0.5);
        let ray = Ray::new(camera, ray_dir);
        let hit = intersect(&ray, &sphere_1);
        let (r, g, b) = 
            match hit {
                Some(idata) => {
                    let u_tex = 255 * ((idata.uv.x * 100.0).floor() as u8 % 2);
                    let v_tex = 255 * ((idata.uv.y * 100.0).floor() as u8 % 2);
                    (u_tex ^ v_tex, u_tex ^ v_tex, u_tex ^ v_tex)
                },
                None => (0, 0, 0),
            };
        out_vec[offset]   = r as u8;
        out_vec[offset+1] = g as u8;
        out_vec[offset+2] = b as u8;
        out_vec[offset+3] = 255;
    }
}

struct IntersectData {
    point: Point3<f32>,
    normal: Vector3<f32>,
    uv: Point2<f32>,
}

struct Sphere {
    center: Point3<f32>,
    radius: f32,
}

fn intersect(ray: &Ray, sphere: &Sphere) -> Option<IntersectData> {
    let mut t0;
    let mut t1;
    let dir = ray.direction;
    let radius2 = sphere.radius * sphere.radius;
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
    
    // compute uv
    let u = (phit - sphere.center).y.atan2((phit - sphere.center).x);
    let v = ((phit - sphere.center).z / sphere.radius).acos();
    Some(IntersectData {
        point: phit,
        normal: (phit - sphere.center).normalize(),
        uv: Point2::new(u / TWO_PI, v / TWO_PI),
    })
}
