///! Defines the plane mesh
///! TODO: The mesh should be taken from a .obj, and idk about the contact points but probably a toml or something per plane.
///!       Maybe the toml specifies the .obj, the textures, and the contact points? And the engine config? And the aero config?
use apricot::mesh::{MeshData, Vertex};
use nalgebra_glm::{vec2, vec3, DVec3, Vec3};

/// Where the propeller hub sits, in body space [m]
pub const PROP_HUB: Vec3 = Vec3::new(2.42, -0.02, 0.0);

/// Something that can collide with the ground
#[derive(Clone, Copy, PartialEq, Eq)]
pub enum ContactKind {
    /// Smoothly roll across the ground
    Wheel {
        /// Whether this wheel has brakes
        brakes: bool,
    },
    /// Agressively scrape against the ground
    Airframe,
}

/// A sphere on the airplane that collides with the ground, in body space
#[derive(Clone, Copy)]
pub struct ContactPoint {
    /// Center of the sphere [m]
    pub pos: DVec3,
    /// Radius of the sphere
    pub radius: f64,
    pub kind: ContactKind,
}

const fn wheel(x: f64, y: f64, z: f64, radius: f64, brakes: bool) -> ContactPoint {
    ContactPoint {
        pos: DVec3::new(x, y, z),
        radius,
        kind: ContactKind::Wheel { brakes },
    }
}

const fn airframe(x: f64, y: f64, z: f64) -> ContactPoint {
    ContactPoint {
        pos: DVec3::new(x, y, z),
        radius: 0.1,
        kind: ContactKind::Airframe,
    }
}

pub const CONTACTS: [ContactPoint; 10] = [
    // Gear
    wheel(-0.5, -1.1, -1.2, 0.25, true),
    wheel(-0.5, -1.1, 1.2, 0.25, true),
    wheel(1.9, -1.05, 0.0, 0.2, false),
    // Wingtips
    airframe(-0.3, 0.9, -5.5), // left
    airframe(-0.3, 0.9, 5.5),  // right
    // Tail
    airframe(-5.3, 0.0, 0.0),  // bottom of the fuselage
    airframe(-5.1, 0.2, -1.7), // left stab
    airframe(-5.1, 0.2, 1.7),  // right stab
    airframe(-5.3, 1.55, 0.0), // top of fin
    // Prop tip at its lowest
    airframe(2.42, -0.97, 0.0),
];

/// Get the height of the lowest wheel [m]
pub fn gear_height() -> f64 {
    CONTACTS
        .iter()
        .map(|c| c.radius - c.pos.y)
        .fold(0.0, f64::max)
}

/// Two meshes so they can be drawn in different colors
pub struct AirplaneModel {
    /// Fuselage, wings, tail, struts and gear legs
    pub body: MeshData,
    /// Windows, wheels and propeller
    pub trim: MeshData,
    /// Propeller blades, centered on the origin. Draw at `PROP_HUB`
    pub prop: MeshData,
}

pub fn airplane_model() -> AirplaneModel {
    let mut body = Builder::default();
    let mut trim = Builder::default();

    // Fuselage, nose to tail, as cross-sections: (x, bottom y, top y, half width)
    let fuselage = [
        (2.30, -0.30, 0.25, 0.35),  // front of cowling
        (1.50, -0.55, 0.45, 0.50),  // firewall
        (0.80, -0.55, 0.75, 0.55),  // top of windshield
        (-1.00, -0.50, 0.75, 0.55), // back of cabin
        (-5.30, 0.00, 0.35, 0.12),  // tail
    ];
    for pair in fuselage.windows(2) {
        body.section(pair[0], pair[1]);
    }

    // Wing on top of the cabin, slight dihedral, tapering toward the tips
    for side in [-1.0, 1.0] {
        let root = slab_z(0.35, -1.15, 0.75, 0.90, 0.0);
        let tip = slab_z(0.25, -0.85, 0.90, 1.00, side * 5.5);
        body.hexa(cat(root, tip));
    }

    // Horizontal stabilizer
    for side in [-1.0, 1.0] {
        let root = slab_z(-4.55, -5.45, 0.20, 0.27, 0.0);
        let tip = slab_z(-4.85, -5.40, 0.20, 0.26, side * 1.7);
        body.hexa(cat(root, tip));
    }

    // Vertical fin, swept back
    let fin_root = slab_y(-4.20, -5.50, 0.30, 0.05);
    let fin_top = slab_y(-5.00, -5.60, 1.55, 0.03);
    body.hexa(cat(fin_root, fin_top));

    // Wing struts, lower fuselage to under the wing
    for side in [-1.0, 1.0] {
        body.strut(
            vec3(-0.3, -0.4, side * 0.5),
            vec3(-0.3, 0.77, side * 2.6),
            0.08,
            0.025,
        );
    }

    // Landing gear
    // TODO: Make this read from the contact points?!
    for side in [-1.0, 1.0] {
        body.strut(
            vec3(-0.5, -0.5, side * 0.45),
            vec3(-0.5, -1.05, side * 1.15),
            0.06,
            0.025,
        );
        trim.wheel(vec3(-0.5, -1.1, side * 1.2), 0.25, 0.09);
    }
    body.strut(vec3(1.9, -0.35, 0.0), vec3(1.9, -0.95, 0.0), 0.05, 0.04);
    trim.wheel(vec3(1.9, -1.05, 0.0), 0.2, 0.07);

    // Spinner and a two-blade propeller
    trim.section((2.30, -0.20, 0.15, 0.17), (2.55, -0.04, 0.00, 0.02));

    let mut prop = Builder::default();
    prop.cuboid(Vec3::zeros(), vec3(0.025, 0.95, 0.07));

    // Side windows, sitting just proud of the cabin sides
    for side in [-1.0, 1.0] {
        let inner = side * 0.555;
        let outer = side * 0.575;
        let pane = |z| {
            [
                vec3(0.75, 0.25, z),
                vec3(-0.90, 0.25, z),
                vec3(-0.90, 0.70, z),
                vec3(0.75, 0.70, z),
            ]
        };
        trim.hexa(cat(pane(inner), pane(outer)));
    }

    // Windshield, on the sloped top-front of the cabin
    let slope_normal = vec3(0.3, 0.7, 0.0).normalize();
    let windshield = |offset: f32| {
        let o = slope_normal * offset;
        [
            vec3(1.45, 0.471, -0.42) + o,
            vec3(1.45, 0.471, 0.42) + o,
            vec3(0.85, 0.729, 0.47) + o,
            vec3(0.85, 0.729, -0.47) + o,
        ]
    };
    trim.hexa(cat(windshield(0.005), windshield(0.025)));

    AirplaneModel {
        body: body.data,
        trim: trim.data,
        prop: prop.data,
    }
}

/// A wing-like cross-section at spanwise position `z`: leading/trailing edge x, bottom/top y
fn slab_z(le: f32, te: f32, bottom: f32, top: f32, z: f32) -> [Vec3; 4] {
    [
        vec3(le, bottom, z),
        vec3(te, bottom, z),
        vec3(te, top, z),
        vec3(le, top, z),
    ]
}

/// A fin-like cross-section at height `y`: leading/trailing edge x, half thickness
fn slab_y(le: f32, te: f32, y: f32, half_thickness: f32) -> [Vec3; 4] {
    [
        vec3(le, y, -half_thickness),
        vec3(te, y, -half_thickness),
        vec3(te, y, half_thickness),
        vec3(le, y, half_thickness),
    ]
}

fn cat(a: [Vec3; 4], b: [Vec3; 4]) -> [Vec3; 8] {
    [a[0], a[1], a[2], a[3], b[0], b[1], b[2], b[3]]
}

/// Accumulates flat-shaded convex pieces into one mesh
#[derive(Default)]
struct Builder {
    data: MeshData,
}

impl Builder {
    /// One flat face. `inside` is any point inside the solid, used to point the face outward,
    /// so the corners can be given in either winding order
    fn polygon(&mut self, corners: &[Vec3], inside: Vec3) {
        // Newell's method: a robust normal even if a corner is degenerate
        let mut n = Vec3::zeros();
        for (i, a) in corners.iter().enumerate() {
            let b = corners[(i + 1) % corners.len()];
            n += a.cross(&b);
        }
        if n.norm_squared() < 1e-12 {
            return; // zero-area face
        }
        let centroid = corners.iter().sum::<Vec3>() / corners.len() as f32;

        let mut pts = corners.to_vec();
        if n.dot(&(centroid - inside)) < 0.0 {
            pts.reverse();
            n = -n;
        }
        let n = n.normalize();

        let base = self.data.vertex_count() as u32;
        for p in &pts {
            self.data.vertices.push(Vertex::new(*p, n, vec2(0.0, 0.0)));
        }
        // Triangle fan
        for i in 1..pts.len() as u32 - 1 {
            self.data
                .indices
                .extend_from_slice(&[base, base + i, base + i + 1]);
        }
    }

    /// A six-sided solid. Corners 0..4 are one end and 4..8 the other, in matching order
    fn hexa(&mut self, c: [Vec3; 8]) {
        let inside = c.iter().sum::<Vec3>() / 8.0;
        for f in [
            [0, 1, 2, 3],
            [4, 5, 6, 7],
            [0, 1, 5, 4],
            [1, 2, 6, 5],
            [2, 3, 7, 6],
            [3, 0, 4, 7],
        ] {
            self.polygon(&[c[f[0]], c[f[1]], c[f[2]], c[f[3]]], inside);
        }
    }

    /// A fuselage segment between two cross-sections: (x, bottom y, top y, half width)
    fn section(&mut self, a: (f32, f32, f32, f32), b: (f32, f32, f32, f32)) {
        let ring = |(x, bottom, top, hw): (f32, f32, f32, f32)| {
            [
                vec3(x, bottom, -hw),
                vec3(x, bottom, hw),
                vec3(x, top, hw),
                vec3(x, top, -hw),
            ]
        };
        self.hexa(cat(ring(a), ring(b)));
    }

    /// Axis-aligned box
    fn cuboid(&mut self, center: Vec3, half: Vec3) {
        let end = |z: f32| {
            [
                center + vec3(-half.x, -half.y, z),
                center + vec3(half.x, -half.y, z),
                center + vec3(half.x, half.y, z),
                center + vec3(-half.x, half.y, z),
            ]
        };
        self.hexa(cat(end(-half.z), end(half.z)));
    }

    /// A flat bar from `a` to `b`, wide along X like a streamlined strut
    fn strut(&mut self, a: Vec3, b: Vec3, half_chord: f32, half_thickness: f32) {
        let x = vec3(1.0, 0.0, 0.0);
        let n = (b - a).cross(&x).normalize();
        let ring = |p: Vec3| {
            [
                p - x * half_chord - n * half_thickness,
                p + x * half_chord - n * half_thickness,
                p + x * half_chord + n * half_thickness,
                p - x * half_chord + n * half_thickness,
            ]
        };
        self.hexa(cat(ring(a), ring(b)));
    }

    /// An octagonal wheel with its axle along Z
    fn wheel(&mut self, center: Vec3, radius: f32, half_width: f32) {
        const SIDES: usize = 8;
        let rim = |z: f32| -> Vec<Vec3> {
            (0..SIDES)
                .map(|i| {
                    let a = i as f32 / SIDES as f32 * std::f32::consts::TAU;
                    center + vec3(a.cos() * radius, a.sin() * radius, z)
                })
                .collect()
        };
        let near = rim(-half_width);
        let far = rim(half_width);

        self.polygon(&near, center);
        self.polygon(&far, center);
        for i in 0..SIDES {
            let j = (i + 1) % SIDES;
            self.polygon(&[near[i], near[j], far[j], far[i]], center);
        }
    }
}
