use std::{cell::RefCell, rc::Rc};

use apricot::{
    app::App,
    geometry::rectangle::Rectangle,
    render::assets::FontId,
    ui::{MsgQueue, UpdateCx, Widget, STYLE},
};
use nalgebra_glm::{vec2, vec4, Vec2, Vec4};

use crate::overview::Overview;

const MIN_VIEW: f32 = 25_000.0;
const VIEW_MARGIN: f32 = 0.2;
const ROUTE_WIDTH: f32 = 2.0; // [px]
const STOP_RADIUS: f32 = 5.0; // [px]
const ROUTE_COLOR: Vec4 = Vec4::new(0.9, 0.4, 1.0, 1.0); // the thin magenta line

pub enum Stop {
    Start,
    Sight,
    Search { radius_m: f64 },
    Land,
}

pub struct Route {
    pub overview: Rc<Overview>,
    pub stops: Vec<(f64, f64, Stop)>,
}

impl Route {
    fn view(&self, frame: Vec2) -> Rectangle {
        let o = &self.overview;
        let (min, max) = self
            .stops
            .iter()
            .map(|(lat, lon, _)| o.pixel(*lat, *lon))
            .fold((o.size, Vec2::zeros()), |(lo, hi), p| {
                (lo.inf(&p), hi.sup(&p))
            });
        let least = MIN_VIEW * o.px_per_m();
        let mut size = ((max - min) * (1.0 + 2.0 * VIEW_MARGIN)).sup(&vec2(least, least));
        let aspect = frame.x / frame.y;
        if size.x < size.y * aspect {
            size.x = size.y * aspect;
        } else {
            size.y = size.x / aspect;
        }
        let size = size.inf(&o.size);
        let pos = ((min + max) / 2.0 - size / 2.0)
            .sup(&Vec2::zeros())
            .inf(&(o.size - size));
        Rectangle { pos, size }
    }
}

pub struct RouteMap {
    rect: Rectangle,
    route: Rc<RefCell<Option<Route>>>,
    font: FontId,
}

impl RouteMap {
    pub fn new(size: Vec2, route: Rc<RefCell<Option<Route>>>, font: FontId) -> Self {
        Self {
            rect: Rectangle {
                pos: Vec2::zeros(),
                size,
            },
            route,
            font,
        }
    }

    fn draw_route(&self, app: &App, route: &Route) {
        let r = &app.renderer;
        let view = route.view(self.rect.size);
        let scale = self.rect.size.x / view.size.x; // screen px per overview px
        let on_screen = |lat: f64, lon: f64| {
            self.rect.pos + (route.overview.pixel(lat, lon) - view.pos) * scale
        };

        r.push_clip(self.rect);
        r.draw_texture(
            route.overview.texture,
            self.rect,
            view,
            vec4(1.0, 1.0, 1.0, 1.0),
        );
        for pair in route.stops.windows(2) {
            let (a, b) = (&pair[0], &pair[1]);
            r.draw_line(
                on_screen(a.0, a.1),
                on_screen(b.0, b.1),
                ROUTE_WIDTH,
                ROUTE_COLOR,
            );
        }
        for (lat, lon, stop) in &route.stops {
            let p = on_screen(*lat, *lon);
            match stop {
                Stop::Start => r.fill_circle(p, STOP_RADIUS, STYLE.positive),
                Stop::Sight => r.fill_circle(p, STOP_RADIUS, ROUTE_COLOR),
                Stop::Search { radius_m } => {
                    let radius = *radius_m as f32 * route.overview.px_per_m() * scale;
                    r.fill_ring(p, radius - 1.0, radius + 1.0, STYLE.warning);
                }
                Stop::Land => r.fill_poly(p, 4, STOP_RADIUS * 1.4, 0.0, STYLE.accent_bright),
            }
        }
        r.pop_clip();
    }
}

impl<Msg: Clone + 'static> Widget<Msg> for RouteMap {
    fn overlay_update(&mut self, _cx: &UpdateCx, _msgq: &mut MsgQueue<Msg>) {}

    fn update(&mut self, _cx: &UpdateCx, _msgq: &mut MsgQueue<Msg>) {}

    fn render(&self, app: &App) {
        let r = &app.renderer;
        r.fill_rect(self.rect, STYLE.surface_deep);
        match self.route.borrow().as_ref() {
            Some(route) => self.draw_route(app, route),
            None => {
                let note = "Fly in this region once to draw its map";
                let size = r.measure_text_in(self.font, note);
                let pos = (self.rect.pos + (self.rect.size - size) / 2.0).map(f32::floor);
                r.draw_text_in(self.font, pos, note, STYLE.text_disabled);
            }
        }
        r.draw_rect(self.rect, 1.0, STYLE.border_subtle);
    }

    fn measure(&self) -> Vec2 {
        self.rect.size
    }

    fn place(&mut self, area: Rectangle) {
        self.rect.pos = area.pos;
    }
}
