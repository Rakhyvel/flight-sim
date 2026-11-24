mod map;
mod pfd;
mod strip;
mod widgets;

use std::fmt::Display;

use apricot::{
    error::Error,
    geometry::rectangle::Rectangle,
    render::{assets::FontId, RenderContext},
};
use nalgebra_glm::{vec2, Vec4};
use sdl2::ttf::FontStyle;

use crate::{flight_data::FlightData, minimap::Minimap};

const EM_PER_SCREEN: f32 = 1.0 / 54.0;
const MIN_EM: f32 = 9.0;
const MAX_EM: f32 = 28.0;

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
const HALO: Vec4 = Vec4::new(0.0, 0.0, 0.0, 0.8);
const STRIP: Vec4 = Vec4::new(0.0, 0.0, 0.0, 0.75);
const BAD: Vec4 = Vec4::new(1.0, 0.35, 0.3, 1.0);

const LIGHT: &[u8] = include_bytes!("../../assets/fonts/IBMPlexMono-Light.ttf");
const REGULAR: &[u8] = include_bytes!("../../assets/fonts/IBMPlexMono-Regular.ttf");

/// Sizes and fonts, all scaled to the window
struct Theme {
    /// Body text size [px]. Every other size is a multiple of this
    em: f32,
    /// Screen edge to panel [px]
    margin: f32,
    /// Panel edge to contents [px]
    pad: f32,
    /// Between items on a panel [px]
    gap: f32,
    /// Small captions
    label: FontId,
    /// Tape numbers, ladder, compass
    body: FontId,
    /// Readouts
    value: FontId,
}

impl Theme {
    fn new(renderer: &RenderContext, em: f32) -> Result<Self, Error> {
        // Adding a font under a name that's taken replaces it, so the ids don't change on resize
        let font = |bytes: &[u8], name: &'static str, ems: f32| {
            renderer.add_font_from_bytes(bytes, name, (em * ems).round() as u16, FontStyle::NORMAL)
        };
        Ok(Self {
            em,
            margin: (0.5 * em).round(),
            pad: (0.4 * em).round(),
            gap: (0.4 * em).round(),
            label: font(LIGHT, "hud_label", 0.8)?,
            body: font(LIGHT, "hud_body", 1.0)?,
            value: font(REGULAR, "hud_value", 1.3)?,
        })
    }
}

/// How a line on a notice reads
#[derive(Clone, Copy)]
pub enum Mood {
    Plain,
    Good,
    Warn,
    Bad,
}

/// A panel of text in the middle of the screen
pub struct Notice {
    pub title: String,
    pub lines: Vec<(String, Mood)>,
}

/// Body text size for a window. Whole pixels, so the fonts only rebuild when it really changes
fn em_for(w: f32, h: f32) -> f32 {
    // Short windows go by height, narrow ones by width
    (h.min(w * 0.75) * EM_PER_SCREEN)
        .round()
        .clamp(MIN_EM, MAX_EM)
}

pub struct Hud {
    theme: Theme,
}

impl Hud {
    pub fn new(renderer: &RenderContext) -> Result<Self, Error> {
        let screen = renderer.screen_size();
        let em = em_for(screen.x as f32, screen.y as f32);
        Ok(Self {
            theme: Theme::new(renderer, em)?,
        })
    }

    pub fn draw(
        &mut self,
        renderer: &RenderContext,
        data: &FlightData,
        minimap: &Minimap,
        fps: impl Display,
        notice: Option<&Notice>,
    ) {
        let screen = renderer.screen_size();
        let (w, h) = (screen.x as f32, screen.y as f32);

        let em = em_for(w, h);
        if em != self.theme.em {
            match Theme::new(renderer, em) {
                Ok(theme) => self.theme = theme,
                Err(e) => eprintln!("couldn't resize the HUD fonts: {e}"),
            }
        }
        let t = &self.theme;

        // In the world, under everything else
        if let Some(p) = data
            .flight_path_point
            .and_then(|p| renderer.world_to_screen(p))
        {
            widgets::draw_fpv(renderer, t, p);
        }

        // Top
        let strip_h = strip::draw(renderer, t, w, data);

        // Bottom left
        let pfd = pfd::size(renderer, t);
        pfd::draw(
            renderer,
            t,
            Rectangle::new(t.margin, h - t.margin - pfd.y, pfd.x, pfd.y),
            data,
        );

        // Bottom right
        let map = map::size(t);
        map::draw(
            renderer,
            t,
            Rectangle::new(w - t.margin - map, h - t.margin - map, map, map),
            data,
            minimap,
        );

        renderer.set_font(t.label);
        renderer.draw_text(
            vec2(t.margin, strip_h + t.margin),
            &format!("{fps} FPS"),
            DIM,
        );

        // Middle, over everything
        if let Some(notice) = notice {
            draw_notice(renderer, t, w, h, notice);
        }
    }
}

/// `notice` centered across a `w` x `h` screen, a little above the middle
fn draw_notice(renderer: &RenderContext, t: &Theme, w: f32, h: f32, notice: &Notice) {
    // Size it to fit
    let line_h = (1.4 * t.em).round();
    renderer.set_font(t.value);
    let title = renderer.measure_text(&notice.title);
    renderer.set_font(t.body);
    let widest = notice
        .lines
        .iter()
        .map(|(text, _)| renderer.measure_text(text).x)
        .fold(title.x, f32::max);
    let size = vec2(
        widest + 2.0 * t.pad,
        title.y + t.gap + line_h * notice.lines.len() as f32 + 2.0 * t.pad,
    );
    let rect = Rectangle::new(
        ((w - size.x) / 2.0).round(),
        (0.3 * h).round(),
        size.x,
        size.y,
    );
    widgets::draw_plate(renderer, rect);

    // Title, then each line
    let x = rect.pos.x + t.pad;
    let mut y = rect.pos.y + t.pad;
    renderer.set_font(t.value);
    renderer.draw_text(vec2(x, y), &notice.title, TEXT);
    y += title.y + t.gap;
    renderer.set_font(t.body);
    for (text, mood) in &notice.lines {
        let color = match mood {
            Mood::Plain => TEXT,
            Mood::Good => FPV,
            Mood::Warn => ACCENT,
            Mood::Bad => BAD,
        };
        renderer.draw_text(vec2(x, y), text, color);
        y += line_h;
    }
}
