#![allow(unknown_lints)]

extern crate alga;
extern crate bvh;
extern crate nalgebra;
extern crate png;
extern crate rand;

use std::ops;
use std::path::Path;
use std::fs::File;
use std::io::BufWriter;
use png::HasParameters;
use nalgebra::{Vector3};
use bvh::nalgebra::{Point2, Point3};
use bvh::ray::Ray;
use std::f32::INFINITY;
use std::f32::consts::FRAC_PI_2;
use std::f64::consts::PI;
use alga::linear::EuclideanSpace;

const TWO_PI: f32 = 2.0 * PI as f32;

const SAMPLE_COUNT_SQRT: i32 = 5;
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
    let mirror = Material::Mirror(Color::new(0.9, 0.9, 0.9));
    let red = Material::Candy(Color::new(0.9, 0.2, 0.3));
    let blue = Material::Candy(Color::new(0.2, 0.3, 0.9));
    let bright_light = Material::Light(Color::new(500.0, 500.0, 500.0));
    let dim_light = Material::Light(Color::new(10.0, 10.0, 10.0));
    let white = Material::Candy(Color::new(0.95, 0.95, 0.95));

    let sphere = Sphere::new(
        Point3::new(0.0, 0.0, 4.0),
        2.0,
        mirror,
    );

    let sphere_2 = Sphere::new(
        Point3::new(0.0, 0.0, -8.0),
        2.0,
        mirror,
    );

    let sphere_3 = Sphere::new(
        Point3::new(-1.8, 1.0, 1.0),
        1.0,
        red,
    );

    let sphere_4 = Sphere::new(
        Point3::new(1.8, 1.0, -2.0),
        1.0,
        blue,
    );

    let camera = Point3::new(0.0, 0.0, -5.0);
    let focal_distance = 7.0; // Focus is on the plane 7 units in front of the camera.
    let aperture_width = 0.5;

    let light_left = Sphere::new(
        Point3::new(-20.0, -10.0, 0.0),
        1.0,
        bright_light,
    );

    let plane = Plane::new(
        Point3::new(0.0, 2.0, 0.0),
        Vector3::new(0.0, -1.0, 0.0),
        white,
    );

    let sky = Plane::new(
        Point3::new(0.0, -50.0, 0.0),
        Vector3::new(0.0, 1.0, 0.0),
        white,
    );

    let back = Plane::new(
        Point3::new(0.0, 0.0, 50.0),
        Vector3::new(0.0, 0.0, -1.0),
        white,
    );

    let front = Plane::new(
        Point3::new(0.0, 0.0, -50.0),
        Vector3::new(0.0, 0.0, 1.0),
        white,
    );

    let left = Plane::new(
        Point3::new(-50.0, 0.0, 0.0),
        Vector3::new(1.0, 0.0, 0.0),
        dim_light,
    );

    let right = Plane::new(
        Point3::new(50.0, 0.0, 0.0),
        Vector3::new(-1.0, 0.0, 0.0),
        white,
    );

    let mut back_light_1 = Disc::new(
        back,
        1_f32,
    );
    back_light_1.plane.center.y -= 20.0;
    back_light_1.plane.center.x -= 5.0;
    back_light_1.plane.center.z -= 0.01;
    back_light_1.plane.material = bright_light;

    let mut back_light_2 = Disc::new(
        back,
        1_f32,
    );
    back_light_2.plane.center.y -= 10.0;
    back_light_2.plane.center.x -= 5.0;
    back_light_2.plane.center.z -= 0.01;
    back_light_2.plane.material = bright_light;

    let mut back_light_3 = Disc::new(
        back,
        1_f32,
    );
    back_light_3.plane.center.y -= 10.0;
    back_light_3.plane.center.x += 5.0;
    back_light_3.plane.center.z -= 0.01;
    back_light_3.plane.material = bright_light;

    let mut back_light_4 = Disc::new(
        back,
        1_f32,
    );
    back_light_4.plane.center.y -= 20.0;
    back_light_4.plane.center.x += 5.0;
    back_light_4.plane.center.z -= 0.01;
    back_light_4.plane.material = bright_light;

    let is: Vec<&Intersectable> = vec![
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

        let mut rgb = Color::new(0.0, 0.0, 0.0);

        // Repeat N times for N samples.
        // Each sample is offset slightly for supersampling.
        for _ in 0..SAMPLE_COUNT {
            // Pick an offset between 0 and 1 from the top-left corner of the
            //   pixel on the image plane, where 1.0 = one pixel width.
            let (xoffs, yoffs) = sample_pixel_random();

            // The x and y coordinates at which this sample pierces the image plane,
            //   ranging from the top left at (0, 0) to the bottom right at (1, 1).
            let x_ratio = (x as f32 + xoffs - (wf / 2.0)) * focal_distance / wf;
            let y_ratio = (y as f32 + yoffs - (hf / 2.0)) * focal_distance / hf;

            // The direction from the exact center of the camera through which
            //   this sample should be taken. For depth of field, we want to
            //   nudge this slightly since the ray's origin will be at a point
            //   sampled within the aperture, rather than the exact center.
            let mut ray_dir_from_camera = Vector3::new(x_ratio, y_ratio, focal_distance);

            let (camera_xoffs, camera_yoffs) = sample_aperture_square(aperture_width);

            // The point that is sampled within the aperture.
            let aperture_sample_point = Point3::new(
                camera.x + camera_xoffs,
                camera.y + camera_yoffs,
                camera.z
            );

            // The actual direction from the sampled aperture point to the
            //   sampled point within the current pixel in the image plane.
            let ray_dir = Vector3::new(
                ray_dir_from_camera.x - camera_xoffs,
                ray_dir_from_camera.y - camera_yoffs,
                ray_dir_from_camera.z
            );

            let ray = Ray::new(aperture_sample_point, ray_dir);
            let sample_rgb = trace_iterative(ray, &is);
            rgb = rgb + sample_rgb;
        }
        let Color { red, green, blue } = rgb * INV_SAMPLE_COUNT;
        out_vec[offset]   = scale_radiance(red) as u8;
        out_vec[offset+1] = scale_radiance(green) as u8;
        out_vec[offset+2] = scale_radiance(blue) as u8;
        out_vec[offset+3] = 255;
    }
}

// A fairly inefficient way to choose a sample (sobol or jittered sampling
//   would be better) but this works well enough.
fn sample_pixel_random() -> (f32, f32) {
    (
        rand::random::<f32>(),
        rand::random::<f32>()
    )
}

// Samples the aperture as if it were shaped like a square. This can look a
//   little weird, since I don't think square apertures are common.
fn sample_aperture_square(aperture_width: f32) -> (f32, f32) {
    (
        rand::random::<f32>() * aperture_width - (aperture_width / 2.0),
        rand::random::<f32>() * aperture_width - (aperture_width / 2.0)
    )
}

fn scale_radiance(x: f32) -> f32 {
    255.0 * (x/4.0).atan() / FRAC_PI_2
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

fn trace_iterative(ray: Ray, thing: &Intersectable) -> Color<f32> {
    let mut ray = ray;
    let mut frag_color = Color::new(1.0, 1.0, 1.0);
    for _ in 0..4 {
        let hit = thing.intersect(&ray);
        if let Some(idata) = hit {
            match idata.material {
                Material::Light(intensity) => {
                    return frag_color * intensity;
                },
                Material::Mirror(mirror_color) => {
                    frag_color = frag_color * mirror_color;
                    ray = Ray::new(
                        idata.point + 0.00001 * idata.normal,
                        mirror(&idata.incident, &idata.normal),
                    );
                },
                Material::Candy(candy_color) => {
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
                    frag_color = frag_color * candy_color;
                },
            }
        } else {
            break;
        }
    }

    frag_color * 0.5
}

#[derive(Debug, Copy, Clone)]
enum Material {
    Mirror(Color<f32>),
    Light(Color<f32>),
    Candy(Color<f32>)
}

#[derive(Debug, Copy, Clone)]
struct Color<T> {
    red: T,
    green: T,
    blue: T,
}

impl<T> Color<T> {
    fn new(red: T, green: T, blue: T) -> Self {
        Color {
            red,
            green,
            blue,
        }
    }
}

impl ops::Add<Color<f32>> for Color<f32> {
    type Output = Color<f32>;
    fn add(self, rhs: Color<f32>) -> Color<f32> {
        Color::new(
            self.red + rhs.red,
            self.green + rhs.green,
            self.blue + rhs.blue,
        )
    }
}

impl ops::Mul<Color<f32>> for Color<f32> {
    type Output = Color<f32>;
    fn mul(self, rhs: Color<f32>) -> Color<f32> {
        Color::new(
            self.red * rhs.red,
            self.green * rhs.green,
            self.blue * rhs.blue,
        )
    }
}

impl ops::Mul<f32> for Color<f32> {
    type Output = Color<f32>;
    fn mul(self, rhs: f32) -> Color<f32> {
        Color::new(
            self.red * rhs,
            self.green * rhs,
            self.blue * rhs,
        )
    }
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
        let mut nearest_dist = INFINITY;
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
            material: Material::Mirror(Color::new(0.0, 0.0, 0.0)),
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
