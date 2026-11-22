use apricot::{geometry::rectangle::Rectangle, render::RenderContext};
use nalgebra_glm::{vec2, Vec2, Vec4};

use crate::hud::Theme;

use super::{DIM, FPV, HALO, OUTLINE, PANEL, TEXT, WAYPOINT};

/// Flight path marker with a circle with wings and a tail
pub fn draw_fpv(renderer: &RenderContext, t: &Theme, p: Vec2) {
    let rr = 0.45 * t.em;
    let wing = 0.8 * t.em;
    let tail = 0.5 * t.em;
    renderer.draw_poly(p, 16, rr, 0.0, 1.5, FPV);
    renderer.draw_line(p - vec2(rr + wing, 0.0), p - vec2(rr, 0.0), 1.5, FPV);
    renderer.draw_line(p + vec2(rr, 0.0), p + vec2(rr + wing, 0.0), 1.5, FPV);
    renderer.draw_line(p - vec2(0.0, rr), p - vec2(0.0, rr + tail), 1.5, FPV);
}

/// Outline through a list of points, back to the start
pub fn draw_outline(renderer: &RenderContext, points: &[Vec2], thickness: f32, color: Vec4) {
    for (i, &a) in points.iter().enumerate() {
        renderer.draw_line(a, points[(i + 1) % points.len()], thickness, color);
    }
}

/// A scrolling vertical scale
pub struct Tape {
    /// What's in the middle
    pub value: f32,
    pub px_per_unit: f32,
    /// Tick spacing
    pub minor: f32,
    /// Numbered tick spacing, a multiple of `minor`
    pub major: f32,
    /// Nothing is drawn below this
    pub min: f32,
    /// Which side the ticks are on, the side facing the attitude indicator
    pub ticks_on_right: bool,
    /// A target marker, in the same units as `value`
    pub bug: Option<f32>,
}

pub fn thousands(n: f32) -> String {
    let n = n.round() as i64;
    let digits = n.unsigned_abs().to_string();
    let mut out = String::new();
    for (i, c) in digits.chars().enumerate() {
        if i > 0 && (digits.len() - i) % 3 == 0 {
            out.push(',');
        }
        out.push(c);
    }
    if n < 0 {
        format!("-{out}")
    } else {
        out
    }
}

/// Translucent backing panel with a thin outline
pub fn draw_plate(renderer: &RenderContext, rect: Rectangle) {
    renderer.fill_rect(rect, PANEL);
    renderer.draw_rect(rect, 1.0, OUTLINE);
}

/// How wide a tape has to be for its boxed value to fit `widest`, like "88,888"
pub fn tape_width(renderer: &RenderContext, t: &Theme, widest: &str) -> f32 {
    renderer.set_font(t.value);
    (renderer.measure_text(widest).x + 1.2 * t.em).round()
}

pub fn draw_tape(renderer: &RenderContext, t: &Theme, rect: Rectangle, tape: &Tape) {
    let major_len = 0.6 * t.em;
    let minor_len = 0.3 * t.em;

    let mid = rect.pos.y + rect.size.y / 2.0;
    let to_y = |v: f32| mid - (v - tape.value) * tape.px_per_unit;
    let (edge, inward) = if tape.ticks_on_right {
        (rect.pos.x + rect.size.x, -1.0)
    } else {
        (rect.pos.x, 1.0)
    };

    renderer.fill_rect(rect, PANEL);
    renderer.push_clip(rect);
    renderer.set_font(t.body);

    let half = rect.size.y / 2.0 / tape.px_per_unit;
    let per_major = (tape.major / tape.minor).round() as i64;
    let first = ((tape.value - half) / tape.minor).floor() as i64;
    let last = ((tape.value + half) / tape.minor).ceil() as i64;
    for i in first..=last {
        let v = i as f32 * tape.minor;
        if v < tape.min {
            continue;
        }
        let y = to_y(v);
        let is_major = i.rem_euclid(per_major) == 0;
        let len = if is_major { major_len } else { minor_len };
        renderer.draw_line(vec2(edge, y), vec2(edge + inward * len, y), 1.0, DIM);

        if is_major {
            let text = thousands(v);
            let s = renderer.measure_text(&text);
            let inset = major_len + 0.3 * t.em;
            let x = if tape.ticks_on_right {
                edge - inset - s.x
            } else {
                edge + inset
            };
            renderer.draw_text(vec2(x, y - s.y / 2.0), &text, TEXT);
        }
    }
    renderer.draw_rect(rect, 1.0, OUTLINE);

    // Current value, boxed across the middle
    renderer.set_font(t.value);
    let text = thousands(tape.value.max(tape.min));
    let s = renderer.measure_text(&text);
    let box_pad = 0.2 * t.em;
    let boxed = Rectangle::new(
        rect.pos.x,
        mid - s.y / 2.0 - box_pad,
        rect.size.x,
        s.y + 2.0 * box_pad,
    );
    renderer.fill_rect(boxed, HALO);
    renderer.draw_rect(boxed, 1.0, TEXT);
    renderer.draw_text(
        vec2(boxed.pos.x + (boxed.size.x - s.x) / 2.0, mid - s.y / 2.0),
        &text,
        TEXT,
    );

    if let Some(bug) = tape.bug {
        let b = 0.4 * t.em;
        let y = to_y(bug).clamp(rect.pos.y + b, rect.pos.y + rect.size.y - b);
        let tip = vec2(edge + inward * 1.6 * b, y);
        draw_outline(
            renderer,
            &[vec2(edge, y - b), tip, vec2(edge, y + b)],
            2.0,
            WAYPOINT,
        );
    }
    renderer.pop_clip();
}
