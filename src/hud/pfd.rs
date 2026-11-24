use apricot::{geometry::rectangle::Rectangle, render::RenderContext};
use nalgebra_glm::{vec2, Vec2};

use super::{
    widgets::{self, Tape},
    Theme, ACCENT, DIM, GROUND, OUTLINE, SKY, TEXT, WAYPOINT,
};
use crate::{
    flight_data::FlightData,
    hud::FPV,
    units::{FT_PER_M, KT_PER_MS},
};

/// Half the attitude indicator's size [em]
const R: f32 = 7.0;
/// Heading tape height [em]
const HEADING_H: f32 = 1.6;

/// Pixel sizes of the pieces, for the current theme
struct Layout {
    r: f32,
    speed_w: f32,
    alt_w: f32,
    heading_h: f32,
}

fn layout(renderer: &RenderContext, t: &Theme) -> Layout {
    Layout {
        r: (R * t.em).round(),
        speed_w: widgets::tape_width(renderer, t, "888"),
        alt_w: widgets::tape_width(renderer, t, "88,888"),
        heading_h: (HEADING_H * t.em).round(),
    }
}

pub fn size(renderer: &RenderContext, t: &Theme) -> Vec2 {
    let l = layout(renderer, t);
    vec2(
        t.pad + l.speed_w + t.gap + 2.0 * l.r + t.gap + l.alt_w + t.pad,
        t.pad + 2.0 * l.r + t.gap + l.heading_h + t.pad,
    )
}

/// Speed tape, attitude with heading under it, altitude tape
pub fn draw(renderer: &RenderContext, t: &Theme, rect: Rectangle, data: &FlightData) {
    let l = layout(renderer, t);
    let tape_h = 2.0 * l.r;
    widgets::draw_plate(renderer, rect);
    let top = rect.pos.y + t.pad;
    let mut x = rect.pos.x + t.pad;

    let speed = Tape {
        value: data.airspeed * KT_PER_MS as f32,
        px_per_unit: 0.15 * t.em, // about +-33 kt visible
        minor: 5.0,
        major: 20.0,
        min: 0.0,
        ticks_on_right: true,
        bug: None,
    };
    widgets::draw_tape(
        renderer,
        t,
        Rectangle::new(x, top, l.speed_w, tape_h),
        &speed,
    );
    x += l.speed_w + t.gap;

    let nav = data.nav.as_ref();
    draw_attitude_indicator(
        renderer,
        t,
        vec2(x + l.r, top + l.r),
        l.r,
        data.pitch,
        data.roll,
        data.slip,
        nav.and_then(|n| n.target_path).map(f32::to_degrees),
    );
    draw_compass(
        renderer,
        t,
        Rectangle::new(x, top + tape_h + t.gap, 2.0 * l.r, l.heading_h),
        data.heading.to_degrees(),
        data.turn_rate.to_degrees(),
        nav.map(|n| n.bearing.to_degrees()),
    );
    x += 2.0 * l.r + t.gap;

    let altitude = Tape {
        value: data.altitude * FT_PER_M as f32,
        px_per_unit: 0.01 * t.em, // about +-500 ft visible
        minor: 100.0,
        major: 200.0,
        min: f32::NEG_INFINITY,
        ticks_on_right: false,
        bug: nav
            .and_then(|n| n.target_altitude)
            .map(|a| a * FT_PER_M as f32),
    };
    widgets::draw_tape(
        renderer,
        t,
        Rectangle::new(x, top, l.alt_w, tape_h),
        &altitude,
    );
}

/// Artificial horizon in a square. `pitch` and `roll` in radians, `target_path` in degrees above
/// the horizon
fn draw_attitude_indicator(
    renderer: &RenderContext,
    t: &Theme,
    center: Vec2,
    r: f32,
    pitch: f32,
    roll: f32,
    slip: f32,
    target_path: Option<f32>,
) {
    let pitch = pitch.to_degrees();
    let px_per_deg = r / 20.0; // 20deg visible each way

    // Screen-space directions (y down). Rolling right turns the horizon counterclockwise
    let along = vec2(roll.cos(), -roll.sin());
    let sky_dir = vec2(-roll.sin(), -roll.cos());
    let horizon = center - sky_dir * (pitch * px_per_deg);

    let square = Rectangle::new(center.x - r, center.y - r, 2.0 * r, 2.0 * r);
    renderer.push_clip(square);

    // Draw the sky and horizon
    renderer.fill_rect(square, SKY);
    let big = 5.0 * r;
    let g = horizon - sky_dir * (big / 2.0);
    renderer.draw_line(g - along * big, g + along * big, big, GROUND);
    renderer.draw_line(horizon - along * big, horizon + along * big, 1.0, TEXT);

    // Draw the ladder
    renderer.set_font(t.label);
    for i in -36_i32..=36 {
        if i == 0 {
            continue;
        }
        let deg = i as f32 * 2.5;
        let p = horizon + sky_dir * (deg * px_per_deg);
        let half_w = r * match i.rem_euclid(4) {
            0 => 0.25,
            2 => 0.12,
            _ => 0.05,
        };
        renderer.draw_line(p - along * half_w, p + along * half_w, 1.0, TEXT);

        if i % 4 == 0 {
            let text = (deg.abs() as i32).to_string();
            let s = renderer.measure_text(&text);
            for side in [-1.0, 1.0] {
                let c = p + along * (side * (half_w + 0.3 * t.em + s.x / 2.0));
                renderer.draw_text(c - s / 2.0, &text, TEXT);
            }
        }
    }

    // The flight path angle that reaches the next waypoint's altitude
    if let Some(deg) = target_path {
        let p = horizon + sky_dir * (deg * px_per_deg);
        for side in [-1.0, 1.0] {
            renderer.draw_line(
                p + along * (side * 0.3 * r),
                p + along * (side * 0.55 * r),
                2.0,
                WAYPOINT,
            );
        }
    }

    renderer.pop_clip();
    renderer.draw_rect(square, 1.0, OUTLINE);

    // Draw bank angle
    let arc_r = 0.85 * r;
    for deg in [
        -60.0_f32, -45.0, -30.0, -20.0, -10.0, 10.0, 20.0, 30.0, 45.0, 60.0,
    ] {
        let dir = vec2(deg.to_radians().sin(), -deg.to_radians().cos());
        let len = if deg.abs() == 30.0 || deg.abs() == 60.0 {
            0.12 * r
        } else {
            0.07 * r
        };
        renderer.draw_line(
            center + dir * arc_r,
            center + dir * (arc_r + len),
            1.0,
            TEXT,
        );
    }
    let s = 0.3 * t.em;
    let top = center + vec2(0.0, -arc_r);
    widgets::draw_outline(
        renderer,
        &[top, top + vec2(-s, -1.4 * s), top + vec2(s, -1.4 * s)],
        1.0,
        TEXT,
    );

    // Bank pointer, always pointing at the sky
    let tip = center + sky_dir * arc_r;
    let base = tip - sky_dir * (1.4 * s);
    widgets::draw_outline(
        renderer,
        &[tip, base - along * s, base + along * s],
        1.5,
        ACCENT,
    );

    // Slip bar under the pointer, slides out the same side as the ball.
    let slide = (slip.to_degrees() / 1.0).clamp(-1.0, 1.0) * 2.0 * s;
    let near = base - sky_dir * (0.25 * s) + along * slide;
    let far = near - sky_dir * (0.5 * s);
    widgets::draw_outline(
        renderer,
        &[
            near - along * s,
            near + along * s,
            far + along * (1.1 * s),
            far - along * (1.1 * s),
        ],
        1.5,
        ACCENT,
    );

    // Draw the aircraft reference
    for side in [-1.0, 1.0] {
        let outer = center + vec2(side * 0.55 * r, 0.0);
        let inner = center + vec2(side * 0.2 * r, 0.0);
        renderer.draw_line(outer, inner, 2.0, ACCENT);
        renderer.draw_line(inner, inner + vec2(0.0, 0.08 * r), 2.0, ACCENT);
    }
    let dot = 0.15 * t.em;
    renderer.fill_rect(
        Rectangle::new(center.x - dot, center.y - dot, 2.0 * dot, 2.0 * dot),
        ACCENT,
    );
}

/// Heading tape drawn into `window`, with an optional bug at `bug_deg`
fn draw_compass(
    renderer: &RenderContext,
    t: &Theme,
    window: Rectangle,
    heading_deg: f32,
    turn_rate_deg: f32,
    bug_deg: Option<f32>,
) {
    let px_per_deg = window.size.x / 90.0; // 45deg visible
    let direction = -1.0;
    let tick = 0.35 * t.em;

    let center = window.pos + window.size / 2.0;
    let bottom = window.pos.y + window.size.y;

    renderer.push_clip(window);
    renderer.set_font(t.body);

    for deg in (0..360).step_by(10) {
        // Offset from the current heading, -180..180
        let a = (deg as f32 - heading_deg + 540.0).rem_euclid(360.0) - 180.0;
        let x = center.x + direction * a * px_per_deg;

        renderer.draw_line(vec2(x, bottom - tick), vec2(x, bottom), 1.0, DIM);

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

    // Draw the rate marks, 3 deg/s for 6 seconds
    for side in [-1.0, 1.0] {
        let x = center.x + side * 18.0 * px_per_deg;
        renderer.draw_line(vec2(x, bottom - 1.8 * tick), vec2(x, bottom), 1.0, TEXT);
    }

    // Where the heading will be in 6 seconds
    let trend = (direction * turn_rate_deg * 6.0).clamp(-40.0, 40.0) * px_per_deg;
    renderer.draw_line(
        vec2(center.x, bottom - 1.5),
        vec2(center.x + trend, bottom - 1.5),
        3.0,
        FPV,
    );

    // Lubber line
    renderer.draw_line(
        vec2(center.x, window.pos.y),
        vec2(center.x, bottom),
        1.0,
        ACCENT,
    );

    if let Some(bug) = bug_deg {
        let b = 0.35 * t.em;
        let a = (bug - heading_deg + 540.0).rem_euclid(360.0) - 180.0;
        let x = (center.x + direction * a * px_per_deg)
            .clamp(window.pos.x + b, window.pos.x + window.size.x - b);
        let top = window.pos.y;
        widgets::draw_outline(
            renderer,
            &[vec2(x - b, top), vec2(x + b, top), vec2(x, top + 1.4 * b)],
            2.0,
            WAYPOINT,
        );
    }

    renderer.pop_clip();
    renderer.draw_rect(window, 1.0, OUTLINE);
}
