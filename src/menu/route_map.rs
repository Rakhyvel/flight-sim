use std::{cell::RefCell, rc::Rc};

use apricot::{
    app::App,
    geometry::rectangle::Rectangle,
    render::assets::FontId,
    ui::{MsgQueue, UpdateCx, Widget, STYLE},
};
use nalgebra_glm::{vec4, Vec2, Vec4};

use crate::overview::Overview;

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

pub struct RouteMap {
    rect: Rectangle,
    route: Rc<RefCell<Option<Route>>>,
    note: Rc<RefCell<String>>,
    font: FontId,
}

impl RouteMap {
    pub fn new(
        size: Vec2,
        route: Rc<RefCell<Option<Route>>>,
        note: Rc<RefCell<String>>,
        font: FontId,
    ) -> Self {
        Self {
            rect: Rectangle {
                pos: Vec2::zeros(),
                size,
            },
            route,
            note,
            font,
        }
    }

    fn draw_route(&self, app: &App, route: &Route) {
        let r = &app.renderer;
        let o = &route.overview;
        let scale = self.rect.size.x / o.size.x; // screen px per overview px
        let on_screen = |lat: f64, lon: f64| self.rect.pos + o.pixel(lat, lon) * scale;
        let whole = Rectangle {
            pos: Vec2::zeros(),
            size: o.size,
        };

        r.push_clip(self.rect);
        r.draw_texture(o.texture, self.rect, whole, vec4(1.0, 1.0, 1.0, 1.0));
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
                let note = &*self.note.borrow();
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
