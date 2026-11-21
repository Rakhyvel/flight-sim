//! Flight instruments

use apricot::{
    geometry::rectangle::Rectangle,
    render::{assets::TextureId, RenderContext},
};
use nalgebra_glm::{vec2, Vec2, Vec4};

// Layout
pub const MARGIN: f32 = 6.0; // screen edge to panel
pub const PAD: f32 = 6.0; // panel edge to contents
pub const GAP: f32 = 6.0; // between items on a panel
pub const INSTRUMENT_R: f32 = 75.0;

// Colors
const PANEL: Vec4 = Vec4::new(0.0, 0.0, 0.0, 0.3);
const OUTLINE: Vec4 = Vec4::new(1.0, 1.0, 1.0, 0.25);
const TEXT: Vec4 = Vec4::new(1.0, 1.0, 1.0, 0.9);
const DIM: Vec4 = Vec4::new(1.0, 1.0, 1.0, 0.5);
const ACCENT: Vec4 = Vec4::new(1.0, 0.75, 0.2, 1.0);
const SKY: Vec4 = Vec4::new(0.25, 0.5, 0.9, 0.35);
const GROUND: Vec4 = Vec4::new(0.6, 0.4, 0.2, 0.35);
const WAYPOINT: Vec4 = Vec4::new(0.9, 0.4, 1.1, 1.0); // we are all children of the thin magenta line...
const FPV: Vec4 = Vec4::new(0.3, 1.0, 0.4, 1.0);

/// Translucent backing panel with a thin outline
pub fn draw_plate(renderer: &RenderContext, rect: Rectangle) {
    renderer.fill_rect(rect, PANEL);
    renderer.draw_rect(rect, 1.0, OUTLINE);
}

/// `label` on the left, `value` right-aligned, vertically centered in `rect`
pub fn draw_readout(renderer: &RenderContext, rect: Rectangle, label: &str, value: &str) {
    let label_size = renderer.measure_text(label);
    let value_size = renderer.measure_text(value);
    let mid_y = rect.pos.y + rect.size.y / 2.0;

    renderer.draw_text(vec2(rect.pos.x, mid_y - label_size.y / 2.0), label, TEXT);
    renderer.draw_text(
        vec2(
            rect.pos.x + rect.size.x - value_size.x,
            mid_y - value_size.y / 2.0,
        ),
        value,
        TEXT,
    );
}

/// Outlined box filling up from the bottom. `fraction` is 0 to 1
pub fn draw_vertical_bar(renderer: &RenderContext, rect: Rectangle, fraction: f32) {
    renderer.draw_rect(rect, 1.0, OUTLINE);

    let inset = 2.0;
    let inner_h = rect.size.y - 2.0 * inset;
    let fill_h = inner_h * fraction.clamp(0.0, 1.0);
    renderer.fill_rect(
        Rectangle::new(
            rect.pos.x + inset,
            rect.pos.y + inset + inner_h - fill_h,
            rect.size.x - 2.0 * inset,
            fill_h,
        ),
        ACCENT,
    );
}

/// Artificial horizon in a square. `pitch` and `roll` in radians
pub fn draw_attitude_indicator(
    renderer: &RenderContext,
    center: Vec2,
    r: f32,
    pitch: f32,
    roll: f32,
    // (right, up) [deg]
    fpv: Option<Vec2>,
    // In degrees above the horizon
    target_path: Option<f32>,
) {
    let pitch = pitch.to_degrees();
    let px_per_deg = r / 30.0; // 30deg visible

    // Screen-space directions (y down). Rolling right turns the horizon counterclockwise
    let along = vec2(roll.cos(), -roll.sin());
    let sky_dir = vec2(-roll.sin(), -roll.cos());
    let horizon = center - sky_dir * (pitch * px_per_deg);

    let square = Rectangle::new(center.x - r, center.y - r, 2.0 * r, 2.0 * r);
    renderer.push_clip(square);

    renderer.fill_rect(square, SKY);

    // Faint ground fill below the horizon, as a huge rotated rect
    let big = 5.0 * r;
    let g = horizon - sky_dir * (big / 2.0);
    renderer.draw_line(g - along * big, g + along * big, big, GROUND);
    renderer.draw_line(horizon - along * big, horizon + along * big, 1.0, TEXT);

    // Pitch ladder every 10deg
    for deg in (-90_i32..=90).step_by(10) {
        if deg == 0 {
            continue;
        }
        let p = horizon + sky_dir * (deg as f32 * px_per_deg);
        let half_w = 0.15 * r;
        renderer.draw_line(p - along * half_w, p + along * half_w, 1.0, DIM);
    }

    // Draw the target pitch bar, if there is one
    if let Some(deg) = target_path {
        let p = horizon + sky_dir * (deg * px_per_deg);
        for side in [-1.0, 1.0] {
            renderer.draw_line(
                p + along * (side * 0.12 * r),
                p + along * (side * 0.4 * r),
                2.0,
                WAYPOINT,
            );
        }
    }

    renderer.pop_clip();
    renderer.draw_rect(square, 1.0, OUTLINE);

    // Draw the velocity marker
    // TODO: it's a bit garish...
    if let Some(fpv) = fpv {
        let lim = r - 10.0;
        let p = center + vec2(fpv.x, -fpv.y) * px_per_deg;
        let p = vec2(
            p.x.clamp(center.x - lim, center.x + lim),
            p.y.clamp(center.y - lim, center.y + lim),
        );

        // draw the outline of the circle
        // TODO: Should this be an apricot fn?
        let rr = 5.0;
        for i in 0..8 {
            let a0 = i as f32 / 8.0 * std::f32::consts::TAU;
            let a1 = (i + 1) as f32 / 8.0 * std::f32::consts::TAU;
            renderer.draw_line(
                p + vec2(a0.cos(), a0.sin()) * rr,
                p + vec2(a1.cos(), a1.sin()) * rr,
                1.5,
                FPV,
            );
        }

        // draw wings and tail
        renderer.draw_line(p - vec2(rr + 9.0, 0.0), p - vec2(rr, 0.0), 1.5, FPV);
        renderer.draw_line(p + vec2(rr, 0.0), p + vec2(rr + 9.0, 0.0), 1.5, FPV);
        renderer.draw_line(p - vec2(0.0, rr), p - vec2(0.0, rr + 6.0), 1.5, FPV);
    }

    // Fixed aircraft reference
    renderer.draw_line(
        center + vec2(-0.5 * r, 0.0),
        center + vec2(-0.15 * r, 0.0),
        1.0,
        ACCENT,
    );
    renderer.draw_line(
        center + vec2(0.15 * r, 0.0),
        center + vec2(0.5 * r, 0.0),
        1.0,
        ACCENT,
    );
    renderer.fill_rect(
        Rectangle::new(center.x - 1.5, center.y - 1.5, 3.0, 3.0),
        ACCENT,
    );

    // Numbers in the corner
    let text = format!("P {pitch:+.0}\nR {:+.0}", roll.to_degrees());
    renderer.draw_text(square.pos + vec2(4.0, 2.0), &text, DIM);
}

/// Heading tape drawn into `window`
pub fn draw_compass(
    renderer: &RenderContext,
    window: Rectangle,
    heading_deg: f32,
    bug_deg: Option<f32>,
) {
    let px_per_deg = window.size.x / 90.0; // 45deg visible
    let direction = -1.0;

    let center = window.pos + window.size / 2.0;
    let bottom = window.pos.y + window.size.y;

    renderer.push_clip(window);

    for deg in (0..360).step_by(10) {
        // Offset from the current heading, -180..180
        let a = (deg as f32 - heading_deg + 540.0).rem_euclid(360.0) - 180.0;
        let x = center.x + direction * a * px_per_deg;

        renderer.draw_line(vec2(x, bottom - 6.0), vec2(x, bottom), 1.0, DIM);

        if deg % 30 == 0 {
            let label = match deg {
                0 => "N".to_string(),
                90 => "E".to_string(),
                180 => "S".to_string(),
                270 => "W".to_string(),
                d => (d / 10).to_string(),
            };
            let size = renderer.measure_text(&label);
            renderer.draw_text(
                vec2(x - size.x / 2.0, center.y - size.y / 2.0),
                &label,
                TEXT,
            );
        }
    }

    // Lubber line
    renderer.draw_line(
        vec2(center.x, window.pos.y),
        vec2(center.x, bottom),
        1.0,
        ACCENT,
    );

    // Draw the bug
    if let Some(bug) = bug_deg {
        let a = (bug - heading_deg + 540.0).rem_euclid(360.0) - 180.0;
        let x = (center.x + direction * a * px_per_deg)
            .clamp(window.pos.x + 5.0, window.pos.x + window.size.x - 5.0);
        let top = window.pos.y;
        let (l, r, tip) = (vec2(x - 5.0, top), vec2(x + 5.0, top), vec2(x, top + 7.0));
        for (p, q) in [(l, r), (r, tip), (tip, l)] {
            // TODO: this is pretty common for drawing the outlines of stuff, extract to fn
            renderer.draw_line(p, q, 2.0, WAYPOINT);
        }
    }

    renderer.pop_clip();
    renderer.draw_rect(window, 1.0, OUTLINE);
}

pub fn draw_minimap(
    renderer: &RenderContext,
    center: Vec2,
    r: f32,
    map: TextureId,
    map_px: f32,
    heading: f32,
    range: f32,
    route: &[Vec2],
    active: usize,
) {
    let square = Rectangle::new(center.x - r, center.y - r, 2.0 * r, 2.0 * r);
    renderer.draw_texture(
        map,
        square,
        Rectangle::new(0.0, map_px, map_px, -map_px),
        Vec4::new(1.0, 1.0, 1.0, 1.0),
    );

    // Draw the route
    let (s, c) = heading.sin_cos();
    let to_screen = |d: Vec2| center + vec2(d.x * c + d.y * s, d.y * c - d.x * s) * (r / range);

    renderer.push_clip(square);
    let ahead = &route[active.min(route.len())..];
    let mut prev = center;
    for (i, &d) in ahead.iter().enumerate() {
        let p = to_screen(d);
        renderer.draw_line(prev, p, if i == 0 { 2.0 } else { 1.0 }, WAYPOINT);
        renderer.fill_rect(Rectangle::new(p.x - 3.0, p.y - 3.0, 6.0, 6.0), WAYPOINT);
        prev = p;
    }
    renderer.pop_clip();

    // If the active waypoint is off the map, put a marker on the edge pointing at it
    if let Some(&d) = ahead.first() {
        let v = to_screen(d) - center;
        let edge = (r - 6.0) / v.x.abs().max(v.y.abs());
        if edge < 1.0 {
            let p = center + v * edge;
            renderer.fill_rect(Rectangle::new(p.x - 4.0, p.y - 4.0, 8.0, 8.0), WAYPOINT);
        }
    }

    // The plane is always pointing up
    let tip = center + vec2(0.0, -7.0);
    let left = center + vec2(-6.0, 6.0);
    let notch = center + vec2(0.0, 3.0);
    let right = center + vec2(6.0, 6.0);
    for (a, b) in [(tip, left), (left, notch), (notch, right), (right, tip)] {
        renderer.draw_line(a, b, 2.0, ACCENT);
    }

    // North turns the opposite way to the plane
    let north = vec2(-heading.sin(), -heading.cos()); // screen space, y down
    let n_pos = center + north * (r - 10.0);
    let n_size = renderer.measure_text("N");
    renderer.draw_text(n_pos - n_size / 2.0, "N", TEXT);
}
