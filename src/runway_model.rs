use apricot::mesh::{MeshData, Vertex};
use nalgebra_glm::{vec2, vec3, DVec2, Vec2, Vec3};

/// How far the runway floats above the ground it's graded into [m]. Depth bias does the rest
const LIFT: f32 = 0.05;

/// Asphalt, and white markings, for a runway from world `a` to `b` (x, z) [m] that's `width` wide.
/// `ground` gives the height and normal at a world (x, z)
pub fn runway_meshes(
    a: DVec2,
    b: DVec2,
    width: f32,
    ground: impl Fn(f64, f64) -> (f32, Vec3),
) -> (MeshData, MeshData) {
    let len = (b - a).norm() as f32;
    let fwd: Vec2 = ((b - a) / (b - a).norm()).cast();
    let right = vec2(-fwd.y, fwd.x); // world x/z, so north (0, -1) gives east (1, 0)
    let half = 0.5 * width;

    // A rectangle from `t0` to `t1` along the runway and `s0` to `s1` across it (right is +)
    let rect = |t0: f32, t1: f32, s0: f32, s1: f32| {
        [(t0, s0), (t0, s1), (t1, s0), (t1, s1)].map(|(t, s)| {
            let p = fwd * t + right * s;
            let world = a + p.cast::<f64>();
            let (h, n) = ground(world.x, world.y);
            (vec3(p.x, h + LIFT, p.y), n)
        })
    };

    // Asphalt, in small pieces so it follows the ground (it matters where runways cross)
    let mut asphalt = MeshData::default();
    let (nt, ns) = ((len / 15.0).ceil() as usize, 4);
    for i in 0..nt {
        for k in 0..ns {
            let t = |i: usize| len * i as f32 / nt as f32;
            let s = |k: usize| -half + width * k as f32 / ns as f32;
            push_quad(&mut asphalt, rect(t(i), t(i + 1), s(k), s(k + 1)));
        }
    }

    let mut marks = MeshData::default();

    // Threshold bars at both ends
    let bars = match width {
        w if w < 21.0 => 4,
        w if w < 26.0 => 6,
        w if w < 35.0 => 8,
        w if w < 52.0 => 12,
        _ => 16,
    };
    for (t0, t1) in [(6.0, 51.0), (len - 51.0, len - 6.0)] {
        for k in 0..bars / 2 {
            let s = 2.625 + k as f32 * 3.5;
            push_quad(&mut marks, rect(t0, t1, s, s + 1.75));
            push_quad(&mut marks, rect(t0, t1, -s - 1.75, -s));
        }
    }

    // Dashed centerline 36 m stripes with 24 m gaps, between the thresholds
    let mut t = 60.0;
    while t + 36.0 <= len - 60.0 {
        push_quad(&mut marks, rect(t, t + 36.0, -0.45, 0.45));
        t += 60.0;
    }

    (asphalt, marks)
}

/// Two triangles from corners [(t0, s0), (t0, s1), (t1, s0), (t1, s1)], facing up
fn push_quad(mesh: &mut MeshData, corners: [(Vec3, Vec3); 4]) {
    let base = mesh.vertices.len() as u32;
    for (pos, normal) in corners {
        mesh.vertices.push(Vertex::new(pos, normal, vec2(0.0, 0.0)));
    }
    mesh.indices
        .extend_from_slice(&[base, base + 1, base + 2, base + 2, base + 1, base + 3]);
}
