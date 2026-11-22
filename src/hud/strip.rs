use apricot::{geometry::rectangle::Rectangle, render::RenderContext};
use nalgebra_glm::{vec2, Vec4};

use super::{
    widgets::thousands, ACCENT, DIM, FT_PER_M, KT_PER_MS, M_PER_NM, OUTLINE, STRIP, TEXT, WAYPOINT,
};
use crate::{flight_data::FlightData, hud::Theme};

struct Cell {
    label: &'static str,
    value: String,
    color: Vec4,
    /// Characters to make room for, so cells don't jump around as the value changes
    chars: usize,
}

const fn cell(label: &'static str, value: String, color: Vec4, chars: usize) -> Cell {
    Cell {
        label,
        value,
        color,
        chars,
    }
}

/// Bar of label-over-value cells across the top of the screen. Gives back its height
pub fn draw(renderer: &RenderContext, t: &Theme, screen_w: f32, data: &FlightData) -> f32 {
    let cells = cells(data);

    renderer.set_font(t.label);
    let label_char = renderer.measure_text("0");
    renderer.set_font(t.value);
    let value_char = renderer.measure_text("0");
    let height = (t.pad + label_char.y + value_char.y + t.pad).round();

    let widths: Vec<f32> = cells
        .iter()
        .map(|c| {
            let label = c.label.len() as f32 * label_char.x;
            let value = c.chars as f32 * value_char.x;
            (label.max(value) + 1.5 * t.em).round()
        })
        .collect();

    renderer.fill_rect(Rectangle::new(0.0, 0.0, screen_w, height), STRIP);

    let mut x = ((screen_w - widths.iter().sum::<f32>()) / 2.0).max(0.0);
    for (i, (c, &w)) in cells.iter().zip(&widths).enumerate() {
        if i > 0 {
            renderer.draw_line(vec2(x, t.pad), vec2(x, height - t.pad), 1.0, OUTLINE);
        }
        renderer.set_font(t.label);
        let ls = renderer.measure_text(c.label);
        renderer.draw_text(vec2(x + (w - ls.x) / 2.0, t.pad), c.label, DIM);

        renderer.set_font(t.value);
        let vs = renderer.measure_text(&c.value);
        renderer.draw_text(
            vec2(x + (w - vs.x) / 2.0, t.pad + label_char.y),
            &c.value,
            c.color,
        );
        x += w;
    }
    height
}

fn cells(data: &FlightData) -> Vec<Cell> {
    let mut cells = vec![
        cell("GS kt", thousands(data.ground_speed * KT_PER_MS), TEXT, 3),
        cell("VS fpm", vertical_speed(data.vertical_speed), TEXT, 6),
        cell("RA ft", radar_altitude(data.height_above_ground), TEXT, 5),
        cell(
            "HDG",
            format!("{:03.0}", data.heading.to_degrees()),
            TEXT,
            4,
        ),
    ];
    if let Some(nav) = &data.nav {
        cells.extend([
            cell("NEXT", nav.name.clone(), WAYPOINT, 11),
            cell("DIS nm", format!("{:.1}", nav.distance / M_PER_NM), TEXT, 5),
            cell(
                "ETE",
                nav.time_to_go.map_or("--:--".into(), minutes_seconds),
                TEXT,
                5,
            ),
            cell(
                "TGT ft",
                nav.target_altitude
                    .map_or("---".into(), |a| thousands(a * FT_PER_M)),
                WAYPOINT,
                6,
            ),
        ]);
    }
    let braking = data.brakes > 0.5;
    if let Some((lever, actual)) = data.flaps {
        let moving = (lever - actual).abs() > 0.5;
        cells.push(cell(
            "FLAP",
            format!("{actual:.0}"),
            if moving { ACCENT } else { TEXT },
            2,
        ));
    }
    cells.extend([
        cell("THR", format!("{:.0}%", data.throttle * 100.0), TEXT, 4),
        cell(
            "BRK",
            if braking { "ON" } else { "OFF" }.into(),
            if braking { ACCENT } else { DIM },
            3,
        ),
    ]);
    cells
}

/// Rounded to 10 fpm so it doesn't flicker
fn vertical_speed(ms: f32) -> String {
    let fpm = (ms * FT_PER_M * 60.0 / 10.0).round() * 10.0;
    if fpm > 0.0 {
        format!("+{}", thousands(fpm))
    } else {
        thousands(fpm)
    }
}

/// Like a real radar altimeter, only reads below 2,500 ft
fn radar_altitude(m: f32) -> String {
    let ft = m * FT_PER_M;
    if ft < 2500.0 {
        thousands(ft.max(0.0))
    } else {
        "---".into()
    }
}

fn minutes_seconds(s: f32) -> String {
    let s = s as u32;
    format!("{}:{:02}", s / 60, s % 60)
}
