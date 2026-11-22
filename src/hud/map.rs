use apricot::{
    geometry::rectangle::Rectangle,
    render::{assets::TextureId, RenderContext},
};
use nalgebra_glm::{vec2, Vec2, Vec4};

use crate::{
    flight_data::FlightData,
    hud::Theme,
    minimap::{Minimap, MINIMAP_PX, MINIMAP_RANGE},
};

use super::{widgets, ACCENT, HALO, TEXT, WAYPOINT};

/// Half the map's size [em]
const R: f32 = 8.0;

/// The map doesn't change all that much, only redraw it every this many frames to reduce lag
const REDRAW_EVERY: u32 = 4;

fn radius(t: &Theme) -> f32 {
    (R * t.em).round()
}

pub fn size(t: &Theme) -> f32 {
    2.0 * (t.pad + radius(t))
}

pub fn draw(
    renderer: &RenderContext,
    t: &Theme,
    rect: Rectangle,
    data: &FlightData,
    minimap: &Minimap,
) {
    widgets::draw_plate(renderer, rect);
    renderer.set_font(t.body);
    let route = data.nav.as_ref().map_or(&[][..], |n| &n.route[..]);
    draw_minimap(
        renderer,
        t,
        rect.pos + rect.size / 2.0,
        radius(t),
        minimap.minimap.texture,
        MINIMAP_PX as f32,
        data.heading,
        MINIMAP_RANGE,
        route,
    );
}

/// Heading-up map with the route on it. `route` is the waypoints still to fly, as world (x, z)
/// offsets from the plane [m]
fn draw_minimap(
    renderer: &RenderContext,
    t: &Theme,
    center: Vec2,
    r: f32,
    map: TextureId,
    map_px: f32,
    heading: f32,
    range: f32,
    route: &[Vec2],
) {
    let square = Rectangle::new(center.x - r, center.y - r, 2.0 * r, 2.0 * r);
    renderer.draw_texture(
        map,
        square,
        Rectangle::new(0.0, map_px, map_px, -map_px),
        Vec4::new(1.0, 1.0, 1.0, 1.0),
    );

    // Rotate world offsets so the plane's heading points up
    let (s, c) = heading.sin_cos();
    let to_screen = |d: Vec2| center + vec2(d.x * c + d.y * s, d.y * c - d.x * s) * (r / range);

    let dot = 0.25 * t.em;

    renderer.push_clip(square);
    let mut prev = center;
    for (i, &d) in route.iter().enumerate() {
        let p = to_screen(d);
        renderer.draw_line(prev, p, if i == 0 { 2.0 } else { 1.0 }, WAYPOINT);
        renderer.fill_rect(
            Rectangle::new(p.x - dot, p.y - dot, 2.0 * dot, 2.0 * dot),
            WAYPOINT,
        );
        prev = p;
    }
    renderer.pop_clip();

    // If the active waypoint is off the map, put a marker on the edge pointing at it
    if let Some(&d) = route.first() {
        let v = to_screen(d) - center;
        let edge = (r - 0.6 * t.em) / v.x.abs().max(v.y.abs());
        if edge < 1.0 {
            let p = center + v * edge;
            renderer.fill_rect(
                Rectangle::new(p.x - 0.35 * t.em, p.y - 0.35 * t.em, 0.7 * t.em, 0.7 * t.em),
                WAYPOINT,
            );
        }
    }

    // The plane is always pointing up
    let s = 0.55 * t.em;
    widgets::draw_outline(
        renderer,
        &[
            center + vec2(0.0, -s),
            center + vec2(-0.85 * s, 0.85 * s),
            center + vec2(0.0, 0.4 * s),
            center + vec2(0.85 * s, 0.85 * s),
        ],
        2.0,
        ACCENT,
    );

    // North turns the opposite way to the plane
    let north = vec2(-heading.sin(), -heading.cos()); // screen space, y down
    let n_pos = center + north * (r - 0.9 * t.em);
    renderer.fill_circle(n_pos, 0.75 * s, HALO);
    let n_size = renderer.measure_text("N");
    renderer.draw_text(n_pos - n_size / 2.0, "N", TEXT);
}
