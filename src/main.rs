#![allow(unknown_lints)]

extern crate alga;
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
use alga::linear::EuclideanSpace;

const TWO_PI: f32 = 2.0 * PI as f32;

const SAMPLE_COUNT_SQRT: i32 = 50;
const SAMPLE_COUNT: i32 = SAMPLE_COUNT_SQRT * SAMPLE_COUNT_SQRT;
const INV_SAMPLE_COUNT: f32 = 1.0 / (SAMPLE_COUNT as f32);

fn main() {
    let width = 512;
    let height = 512;
    let mut out_vec = vec![0; 4 * width * height];
    make_image(width, height, &mut out_vec);
    write_image(&out_vec, width as u32, height as u32, r"test.png");
}

fn write_image(pixel_buffer: &[u8], width: u32, height: u32, path: &str) {
    assert_eq!(pixel_buffer.len() as u32, width * height * 4);
    
    let path = Path::new(path);
    let file = File::create(path).unwrap();
    let w = &mut BufWriter::new(file);
    let mut encoder = png::Encoder::new(w, width, height);
    encoder.set(png::ColorType::RGBA).set(png::BitDepth::Eight);
    let mut writer = encoder.write_header().unwrap();
    writer.write_image_data(&pixel_buffer).unwrap();
}

#[allow(many_single_char_names)]
fn make_image(width: usize, height: usize, out_vec: &mut [u8]) {
    let sphere = Sphere::new(
        Point3::new(0.0, 0.0, 4.0),
        2.0,
        Material::Mirror,
    );

    let sphere_2 = Sphere::new(
        Point3::new(0.0, 0.0, -8.0),
        2.0,
        Material::Mirror,
    );

    let sphere_3 = Sphere::new(
        Point3::new(-1.8, 1.0, 1.0),
        1.0,
        Material::Candy(0.9, 0.2, 0.3),
    );

    let sphere_4 = Sphere::new(
        Point3::new(1.8, 1.0, -2.0),
        1.0,
        Material::Candy(0.2, 0.3, 0.9),
    );

    let camera = Point3::new(0.0, 0.0, -5.0);

    let light_left = Sphere::new(
        Point3::new(-20.0, -10.0, 0.0),
        1.0,
        Material::Light(500.0),
    );

    let plane = Plane::new(
        Point3::new(0.0, 2.0, 0.0),
        Vector3::new(0.0, -1.0, 0.0),
        Material::Candy(0.95, 0.95, 0.95),
    );

    let sky = Plane::new(
        Point3::new(0.0, -50.0, 0.0),
        Vector3::new(0.0, 1.0, 0.0),
        Material::Candy(0.95, 0.95, 0.95),
    );

    let back = Plane::new(
        Point3::new(0.0, 0.0, 50.0),
        Vector3::new(0.0, 0.0, -1.0),
        Material::Candy(0.95, 0.95, 0.95),
    );

    let front = Plane::new(
        Point3::new(0.0, 0.0, -50.0),
        Vector3::new(0.0, 0.0, 1.0),
        Material::Candy(0.95, 0.95, 0.95),
    );

    let left = Plane::new(
        Point3::new(-50.0, 0.0, 0.0),
        Vector3::new(1.0, 0.0, 0.0),
        Material::Light(10.0),
    );

    let right = Plane::new(
        Point3::new(50.0, 0.0, 0.0),
        Vector3::new(-1.0, 0.0, 0.0),
        Material::Candy(0.95, 0.95, 0.95),
    );

    let mut back_light_1 = Disc::new(
        back.clone(),
        1_f32,
    );
    back_light_1.plane.center.y -= 20.0;
    back_light_1.plane.center.x -= 5.0;
    back_light_1.plane.center.z -= 0.01;
    back_light_1.plane.material = Material::Light(100.0);

    let mut back_light_2 = Disc::new(
        back.clone(),
        1_f32,
    );
    back_light_2.plane.center.y -= 10.0;
    back_light_2.plane.center.x -= 5.0;
    back_light_2.plane.center.z -= 0.01;
    back_light_2.plane.material = Material::Light(100.0);
    
    let mut back_light_3 = Disc::new(
        back.clone(),
        1_f32,
    );
    back_light_3.plane.center.y -= 10.0;
    back_light_3.plane.center.x += 5.0;
    back_light_3.plane.center.z -= 0.01;
    back_light_3.plane.material = Material::Light(100.0);
    
    let mut back_light_4 = Disc::new(
        back.clone(),
        1_f32,
    );
    back_light_4.plane.center.y -= 20.0;
    back_light_4.plane.center.x += 5.0;
    back_light_4.plane.center.z -= 0.01;
    back_light_4.plane.material = Material::Light(100.0);

    let mut is: Vec<&Intersectable> = vec![
        &sphere, &sphere_2, &sphere_3, &sphere_4,
        &light_left,
        &back_light_1, &back_light_2, &back_light_3, &back_light_4,
        &plane, &sky, &back, &left, &right, &front
    ];

    let wf = width as f32;
    let hf = height as f32;

    for i in 0..width * height {
        let offset = 4 * i;
        let x = i % width;
        let y = i / width;

        let mut r = 0.0;
        let mut g = 0.0;
        let mut b = 0.0;

        // Repeat N times for N samples.
        // Each sample is offset slightly for supersampling.
        for _ in 0..SAMPLE_COUNT {
            //let mut xoffs = INV_SAMPLE_COUNT * (ss % SAMPLE_COUNT_SQRT) as f32;
            //let mut yoffs = INV_SAMPLE_COUNT * (ss / SAMPLE_COUNT_SQRT) as f32;
            let xoffs = rand::random::<f32>();// * 0.5;//INV_SAMPLE_COUNT;
            let yoffs = rand::random::<f32>();// * 0.5;//INV_SAMPLE_COUNT;
            let x_ratio = (x as f32 + xoffs - (wf / 2.0)) * 7.0 / wf;
            let y_ratio = (y as f32 + yoffs - (hf / 2.0)) * 7.0 / hf;
            let mut ray_dir_from_camera = Point3::new(x_ratio, y_ratio, 7.0);
            let camera_xoffs = rand::random::<f32>() * 0.5 - 0.25;
            let camera_yoffs = rand::random::<f32>() * 0.5 - 0.25;
            let aperture_sample_point = Point3::new(
                camera.x + camera_xoffs,
                camera.y + camera_yoffs,
                camera.z
            );
            let ray_dir = Vector3::new(
                ray_dir_from_camera.x - camera_xoffs,
                ray_dir_from_camera.y - camera_yoffs,
                ray_dir_from_camera.z
            );
                /*
                camera.x + (ray_target.x + camera.x - aperture_sample_point.x),
                camera.y + (ray_target.y + camera.y - aperture_sample_point.y),
                camera.z + (ray_target.z + camera.z - aperture_sample_point.z)
                */
            let ray = Ray::new(aperture_sample_point, ray_dir);
            let (rr, gg, bb) = trace_iterative(ray, &is);
            r += rr;
            g += gg;
            b += bb;
        }
        let (r, g, b) = (r * INV_SAMPLE_COUNT, g * INV_SAMPLE_COUNT, b * INV_SAMPLE_COUNT);
        out_vec[offset]   = oof(r) as u8;//(r * 255.0).min(255.0) as u8;
        out_vec[offset+1] = oof(g) as u8;//(g * 255.0).min(255.0) as u8;
        out_vec[offset+2] = oof(b) as u8;//(b * 255.0).min(255.0) as u8;
        out_vec[offset+3] = 255;
    }
}

fn oof(x: f32) -> f32 {
    (x * 25.0).min(255.0)
    /*
    let max_lightness = 100.0_f32;
    let scale = 10.0_f32;
    let counter = (max_lightness * scale + 1.0).log10();
    let exposure = 1.0_f32;
    (x * scale + 1.0).log10() * 255.0 * exposure / counter
    */
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

fn mirror(incident: &Vector3<f32>, normal: &Vector3<f32>) -> Vector3<f32> {
    incident - 2.0 * normal.dot(&incident) * normal
}

struct IntersectData {
    point: Point3<f32>,
    normal: Vector3<f32>,
    uv: Point2<f32>,
    dist: f32,
    incident: Vector3<f32>,
    material: Material
}

fn trace_iterative(ray: Ray, thing: &Intersectable) -> (f32, f32, f32) {
    let mut ray = ray;
    let mut frag_color = (1.0, 1.0, 1.0);
    for _ in 0..4 {
        let hit = thing.intersect(&ray);
        if let Some(idata) = hit {
            match idata.material {
                Material::Light(intensity) => {
                    return (
                        frag_color.0 * intensity,
                        frag_color.1 * intensity,
                        frag_color.2 * intensity,
                    );
                },
                Material::Mirror => {
                    frag_color = (
                        frag_color.0 * 0.9,
                        frag_color.1 * 0.9,
                        frag_color.2 * 0.9,
                    );
                    ray = Ray::new(
                        idata.point + 0.00001 * idata.normal,
                        mirror(&idata.incident, &idata.normal),
                    );
                },
                Material::Diffuse => {
                    ray = Ray::new(
                        idata.point + 0.00001 * idata.normal,
                        random_vector_in_hemisphere(&idata.normal)
                    );

                    let u_tex = (idata.uv.x * 3.0).floor() as u8 % 2;
                    let v_tex = (idata.uv.y * 3.0).floor() as u8 % 2;
                    let mut color = f32::from(u_tex ^ v_tex);
                    color += 1.0;
                    color *= 0.5;

                    let u_tex_2 = (idata.uv.x * 0.25).floor() as u8 % 2;
                    let v_tex_2 = (idata.uv.y * 0.25).floor() as u8 % 2;
                    let mut color_2 = f32::from(u_tex_2 ^ v_tex_2);
                    color_2 += 1.0;
                    color_2 *= 0.25;

                    frag_color = (
                        frag_color.0 * color * color_2,
                        frag_color.1 * color * color_2,
                        frag_color.2 * color * color_2
                    );
                },
                Material::Candy(r, g, b) => {
                    let will_reflect = rand::random::<f32>();
                    if will_reflect > 0.05 {
                        ray = Ray::new(
                            idata.point + 0.00001 * idata.normal,
                            random_vector_in_hemisphere(&idata.normal)
                        );
                    } else {
                        ray = Ray::new(
                            idata.point + 0.00001 * idata.normal,
                            mirror(&idata.incident, &idata.normal),
                        );
                    }
                    frag_color = (
                        frag_color.0 * r,
                        frag_color.1 * g,
                        frag_color.2 * b,
                    );
                },
            }
        } else {
            break;
        }
    }

    ( frag_color.0 * 0.5
    , frag_color.1 * 0.5
    , frag_color.2 * 0.5
    )
}

#[derive(Debug, Copy, Clone)]
enum Material {
    Mirror,
    Light(f32),
    Diffuse,
    Candy(f32, f32, f32)
}

trait Intersectable {
    fn intersect(&self, ray: &Ray) -> Option<IntersectData>;
}

#[derive(Debug, Copy, Clone)]
struct Plane {
    center: Point3<f32>,
    normal: Vector3<f32>,
    material: Material,
}

impl Plane {
    fn new(center: Point3<f32>, normal: Vector3<f32>, material: Material) -> Self {
        Plane {
            center,
            normal,
            material,
        }
    }
}

impl Intersectable for Plane {
    fn intersect(&self, ray: &Ray) -> Option<IntersectData> {
        let denom = self.normal.dot(&ray.direction);
        if denom.abs() > 0.0001 {
            let t = (self.center - ray.origin).dot(&self.normal) / denom;
            if t >= 0.0001 {
                let point = ray.origin + t * ray.direction;
                return Some(IntersectData {
                    point,
                    normal: self.normal,
                    uv: Point2::new(point.x, point.z),
                    dist: t,
                    incident: ray.direction,
                    material: self.material,
                });
            }
        }
        None
    }
}

struct Sphere {
    center: Point3<f32>,
    radius: f32,
    radius2: f32,
    material: Material,
}

impl Sphere {
    fn new(center: Point3<f32>, radius: f32, material: Material) -> Self {
        Sphere {
            center,
            radius,
            radius2: radius * radius,
            material,
        }
    }
}

impl Intersectable for Sphere {
    fn intersect(&self, ray: &Ray) -> Option<IntersectData> {
        let mut t0;
        let mut t1;
        let dir = ray.direction;
        let l = self.center - ray.origin;
        let tca = l.dot(&dir);
        let d2 = l.dot(&l) - tca * tca;
        if d2 > self.radius2 { return None; };
        let thc = (self.radius2 - d2).sqrt();
        t0 = tca - thc;
        t1 = tca + thc;

        if t0 > t1 { ::std::mem::swap(&mut t0, &mut t1); };

        if t0 < 0.0 {
            t0 = t1;
            if t0 < 0.0 { return None; };
        }
        let t = t0;
        let phit = ray.origin + t * ray.direction;
        let normal = phit - self.center;

        // compute uv
        let u = normal.z.atan2(normal.x);
        let v = (normal.y / self.radius).acos();

        Some(IntersectData {
            point: phit,
            normal: normal.normalize(),
            uv: Point2::new(u / TWO_PI, v / TWO_PI),
            dist: t,
            incident: ray.direction,
            material: self.material,
        })
    }
}

struct Disc {
    plane: Plane,
    radius: f32,
}

impl Disc {
    fn new(plane: Plane, radius: f32) -> Self {
        Disc {
            plane,
            radius,
        }
    }
}

impl Intersectable for Disc {
    fn intersect(&self, ray: &Ray) -> Option<IntersectData> {
        self.plane.intersect(ray).filter(|idata|
            idata.point.distance(&self.plane.center) < self.radius
        )
    }
}

impl<'a> Intersectable for Vec<&'a dyn Intersectable> {
    fn intersect(&self, ray: &Ray) -> Option<IntersectData> {
        let mut nearest_dist = ::std::f32::INFINITY;
        let mut nearest_hit = None;
        for intersectable in self {
            let hit = intersectable.intersect(ray);
            if let Some(h) = hit {
                if h.dist < nearest_dist {
                    nearest_dist = h.dist;
                    nearest_hit = Some(h);
                }
            }
        }
        nearest_hit
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn plane_intersect_distance_is_correct() {
        let plane = Plane {
            center: Point3::new(0.0, 0.0, 0.0),
            normal: Vector3::new(0.0, 1.0, 0.0),
            material: Material::Light,
        };
        let ray = Ray::new(
            Point3::new(0.0, 2.0, 0.0),
            Vector3::new(0.0, -50.0, 0.0),
        );
        let hit = plane.intersect(&ray).unwrap();
        assert_eq!(hit.dist, 2.0);

        let ray_2 = Ray::new(
            Point3::new(0.0, 1.0, -1.0),
            Vector3::new(0.0, -1.0, 1.0),
        );
        let hit_2 = plane.intersect(&ray_2).unwrap();
        assert!(hit_2.dist < 1.414214 && hit_2.dist > 1.414213);

        let ray_3 = Ray::new(
            Point3::new(0.0, 0.1, 0.0),
            Vector3::new(0.0, -1.0, 0.0),
        );
        let hit_3 = plane.intersect(&ray_3).unwrap();
        assert_eq!(hit_3.dist, 0.1);
    }
}
