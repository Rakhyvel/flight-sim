mod route_map;

use std::{
    cell::{Cell, RefCell},
    collections::HashMap,
    rc::Rc,
    thread::JoinHandle,
};

use apricot::{
    app::{App, Scene, SceneCommand},
    container,
    error::Error,
    geometry::rectangle::Rectangle,
    input::Scancode,
    render::{assets::FontId, RenderContext},
    ui::{
        Align, Anchor, AnchorPoint, Button, ButtonLook, Container, Flow, Label, Layers, Modal,
        Rule, ScrollContainer, Widget, STYLE,
    },
};
use nalgebra_glm::{vec2, vec4, Vec2, Vec4};
use sdl2::ttf::FontStyle;

use crate::{
    airport::{load_runways, Runway},
    config::AirplaneConfig,
    heightmap::{Bounds, Heightmap},
    hud::{LIGHT, REGULAR},
    overview::{self, bounds_around, Overview},
    region::{load_regions, Region},
    scenario::{list_scenarios, Scenario, Start},
    FlightSim,
};
use route_map::{Route, RouteMap, Stop};

const SCENARIO_DIR: &str = "scenarios";
const LIST_SIZE: Vec2 = Vec2::new(380.0, 620.0); // [px]
const ENTRY_SIZE: Vec2 = Vec2::new(364.0, 32.0); // [px]
const DETAILS_WIDTH: f32 = 520.0; // [px]
const MAP_SIZE: Vec2 = Vec2::new(520.0, 220.0); // [px]
/// How bright the blurred map behind the menu is drawn
const BACKDROP_TINT: Vec4 = Vec4::new(0.95, 0.95, 0.955, 1.0);

const BRIEF_SIZE: Vec2 = Vec2::new(520.0, 330.0);

#[derive(Clone)]
enum Msg {
    Select(usize),
    Fly,
    Quit,
}

type Text = Rc<RefCell<String>>;

enum Loading {
    /// Reading the terrain on another thread
    Terrain {
        path: String,
        thread: JoinHandle<Result<Heightmap, String>>,
    },
    /// The terrain's read. Builds the flight on the next update, once this has been drawn
    World { path: String, map: Heightmap },
}

impl Loading {
    fn step(&self) -> &'static str {
        match self {
            Loading::Terrain { .. } => "Reading the terrain",
            Loading::World { .. } => "Building the world",
        }
    }
}

pub struct Menu {
    ui: Layers<Msg>,
    /// Each scenario file, and what's in it or why it couldn't be read
    scenarios: Vec<(String, Result<Scenario, String>)>,
    regions: HashMap<String, Region>,
    /// What each plane file calls its plane
    plane_names: HashMap<String, String>,
    runways: HashMap<String, Runway>,
    /// Each region's overview
    overviews: HashMap<String, Rc<Overview>>,
    selected: Option<usize>,
    loading: Option<Loading>,

    /// Maps being drawn on other threads, by scenario files
    drawing: HashMap<String, JoinHandle<Result<(), String>>>,
    /// Scenario files to why they couldn't be mapped
    unmappable: HashMap<String, String>,
    entry_looks: Vec<Rc<Cell<ButtonLook>>>,
    map_note: Text,

    title: Text,
    details: Text,
    description: Text,
    objectives: Text,
    route: Rc<RefCell<Option<Route>>>,
    status: Text,
    status_color: Rc<Cell<Vec4>>,
    can_fly: Rc<Cell<bool>>,
    loading_shown: Rc<Cell<bool>>,
    loading_title: Text,
    loading_step: Text,
}

impl Menu {
    pub fn new(app: &App, launch: Option<String>) -> Result<Self, Error> {
        let r = &app.renderer;
        let heading = r.add_font_from_bytes(REGULAR, "menu_heading", 28, FontStyle::NORMAL)?;
        let body = r.add_font_from_bytes(LIGHT, "menu_body", 18, FontStyle::NORMAL)?;
        let text = |s: &str| Rc::new(RefCell::new(s.to_string()));

        // Load everything from disk
        let scenarios = list_scenarios(SCENARIO_DIR);
        let plane_names = scenarios
            .iter()
            .filter_map(|(_, s)| s.as_ref().ok())
            .map(|s| {
                let name = AirplaneConfig::load(&s.plane_path())
                    .map_or_else(|_| s.plane.clone(), |c| c.name);
                (s.plane.clone(), name)
            })
            .collect();
        let regions = load_regions("assets/regions.toml")
            .into_iter()
            .map(|r| (r.name.clone(), r))
            .collect();
        let runways = load_runways("assets/runways.csv", |_, _| true)
            .unwrap_or_default()
            .into_iter()
            .map(|r| (r.name.clone(), r))
            .collect();
        let entry_looks = scenarios
            .iter()
            .map(|_| Rc::new(Cell::new(ButtonLook::normal(&STYLE))))
            .collect();

        let mut menu = Self {
            ui: Layers::new(),
            scenarios,
            regions,
            plane_names,
            runways,
            overviews: HashMap::new(),
            selected: None,
            loading: None,
            entry_looks,
            drawing: HashMap::new(),
            unmappable: HashMap::new(),
            map_note: text(""),
            title: text("Pick a flight"),
            details: text(""),
            description: text(""),
            objectives: text(""),
            route: Rc::new(RefCell::new(None)),
            status: text(""),
            status_color: Rc::new(Cell::new(STYLE.text_secondary)),
            can_fly: Rc::new(Cell::new(false)),
            loading_shown: Rc::new(Cell::new(false)),
            loading_title: text(""),
            loading_step: text(""),
        };
        menu.ui = Layers::new()
            .layer(menu.build(app, heading, body))
            .layer(menu.build_loading(app, heading, body));
        menu.ui.layout(app);
        if let Some(path) = launch {
            menu.start_loading(path);
        }
        Ok(menu)
    }

    fn build(&self, app: &App, heading: FontId, body: FontId) -> Anchor<Msg> {
        let r = &app.renderer;

        // Build the scenarios
        let entries = self
            .scenarios
            .iter()
            .enumerate()
            .map(|(i, (path, scenario))| {
                let name = scenario
                    .as_ref()
                    .map_or(path.as_str(), |s| s.title.as_str());
                Box::new(
                    Button::text(ENTRY_SIZE, name)
                        .font(body)
                        .on_click(Msg::Select(i)),
                ) as Box<dyn Widget<Msg>>
            })
            .collect();
        let list = Container::new(entries).padding(vec2(8.0, 8.0)).gap(4.0);
        let list = container![ScrollContainer::new(LIST_SIZE, list)]
            .padding(Vec2::zeros())
            .background_color(STYLE.surface_deep)
            .border(STYLE.border_subtle, 1.0);

        // The picked one
        let wrap_width = DETAILS_WIDTH - 16.0; // room for the scroll bar
        let brief = container![
            Label::new(self.title.clone(), heading, r).wrap(wrap_width, r),
            Label::new(self.details.clone(), body, r)
                .color(STYLE.accent_bright)
                .wrap(wrap_width, r),
            Rule::horizontal(wrap_width).color(STYLE.border_subtle),
            Label::new(self.description.clone(), body, r)
                .color(STYLE.text_secondary)
                .wrap(wrap_width, r),
            Label::new(self.objectives.clone(), body, r).wrap(wrap_width, r),
        ]
        .padding(Vec2::zeros())
        .gap(14.0);
        let fly = Button::fit("Fly", heading, r, vec2(28.0, 8.0))
            .look(ButtonLook::accented(&STYLE))
            .active(self.can_fly.clone())
            .on_click(Msg::Fly);
        let details = container![
            ScrollContainer::new(BRIEF_SIZE, brief),
            RouteMap::new(MAP_SIZE, self.route.clone(), self.map_note.clone(), body),
            container![
                fly,
                Label::new(self.status.clone(), body, r)
                    .color(self.status_color.clone())
                    .wrap(DETAILS_WIDTH - 140.0, r),
            ]
            .flow(Flow::Horizontal)
            .cross_align(Align::Center)
            .padding(Vec2::zeros())
            .gap(16.0),
        ]
        .padding(Vec2::zeros())
        .fixed_width(DETAILS_WIDTH)
        .gap(14.0);

        let page = container![
            Label::new("Flight Sim", heading, r),
            container![list, details]
                .flow(Flow::Horizontal)
                .padding(Vec2::zeros())
                .gap(28.0),
            Button::fit("Quit", body, r, vec2(4.0, 4.0))
                .look(ButtonLook::link(&STYLE))
                .on_click(Msg::Quit),
        ]
        .padding(vec2(32.0, 28.0))
        .gap(18.0)
        .background_color(STYLE.surface)
        .border(STYLE.border, 1.0);

        Anchor::new(page, AnchorPoint::Center)
    }

    fn build_loading(&self, app: &App, heading: FontId, body: FontId) -> Modal<Msg> {
        let r = &app.renderer;
        let content = container![
            Label::new(self.loading_title.clone(), heading, r),
            Label::new(self.loading_step.clone(), body, r).color(STYLE.text_secondary),
        ]
        .padding(vec2(40.0, 28.0))
        .gap(12.0)
        .cross_align(Align::Center)
        .background_color(STYLE.surface)
        .border(STYLE.accent, 1.0);
        Modal::new(content)
            .shown(self.loading_shown.clone())
            .backdrop(vec4(0.0, 0.0, 0.0, 0.6))
    }

    /// Set the selected scenario
    fn select(&mut self, renderer: &RenderContext, i: usize) {
        self.selected = Some(i);
        for (j, look) in self.entry_looks.iter().enumerate() {
            look.set(if j == i {
                ButtonLook::accented(&STYLE)
            } else {
                ButtonLook::normal(&STYLE)
            });
        }

        let (path, scenario) = &self.scenarios[i];
        let Ok(s) = scenario else {
            let error = scenario.as_ref().err().cloned().unwrap_or_default();
            *self.title.borrow_mut() = path.clone();
            *self.details.borrow_mut() = "Couldn't read this one".to_string();
            *self.description.borrow_mut() = error;
            self.objectives.borrow_mut().clear();
            *self.route.borrow_mut() = None;
            self.can_fly.set(false);
            return;
        };

        // Setup the full description
        let region = self.regions.get(&s.region);
        let region_title = region.map_or(s.region.as_str(), |r| r.title.as_str());
        let plane = self
            .plane_names
            .get(&s.plane)
            .map_or(s.plane.as_str(), String::as_str);
        *self.title.borrow_mut() = s.title.clone();
        *self.details.borrow_mut() =
            format!("{region_title} | {plane} | {}", clock_time(s.time_of_day));
        *self.description.borrow_mut() = s.description.clone();
        *self.objectives.borrow_mut() = s
            .objectives()
            .iter()
            .enumerate()
            .map(|(n, objective)| format!("{}. {objective}", n + 1))
            .collect::<Vec<_>>()
            .join("\n");

        // Start drawing the map if there isn't one yet
        let stops = stops(s, &self.runways);
        let overview = overview_for(&mut self.overviews, renderer, path);
        let should_draw = overview.is_none()
            && !self.drawing.contains_key(path)
            && !self.unmappable.contains_key(path);
        let bounds = bounds_around(
            stops.iter().map(|(lat, lon, _)| (*lat, *lon)),
            (MAP_SIZE.x / MAP_SIZE.y) as f64,
        );
        if let (true, Some(region), Some(bounds)) = (should_draw, region, bounds) {
            let (tiff, scenario) = (region.tiff.clone(), path.clone());
            let thread = std::thread::spawn(move || draw_map(&tiff, &scenario, &bounds));
            self.drawing.insert(path.clone(), thread);
        }
        *self.map_note.borrow_mut() = match self.unmappable.get(path) {
            Some(why) => why.clone(),
            None if self.drawing.contains_key(path) => "Drawing the map...".to_string(),
            None => "No map".to_string(),
        };
        *self.route.borrow_mut() = overview.map(|overview| Route { overview, stops });

        self.can_fly.set(true);
        self.show_status("", STYLE.text_secondary);
    }

    fn start_loading(&mut self, path: String) {
        let found = Scenario::try_load(&path).and_then(|s| {
            let region = self
                .regions
                .get(&s.region)
                .ok_or_else(|| format!("{path}: no region {}", s.region))?;
            Ok((s.title, region.tiff.clone()))
        });
        let (title, tiff) = match found {
            Ok(found) => found,
            Err(e) => return self.fail(e),
        };

        *self.loading_title.borrow_mut() = title;
        self.loading_shown.set(true);
        self.loading = Some(Loading::Terrain {
            path,
            thread: std::thread::spawn(move || read_terrain(&tiff)),
        });
    }

    /// Carry on with whatever's loading, and say when the flight's ready
    fn keep_loading(&mut self, app: &App) -> SceneCommand {
        match self.loading.take() {
            Some(Loading::Terrain { path, thread }) if thread.is_finished() => {
                match thread.join() {
                    Ok(Ok(map)) => self.loading = Some(Loading::World { path, map }),
                    Ok(Err(e)) => self.fail(format!("{path}: {e}")),
                    Err(_) => self.fail(format!("{path}: reading the terrain panicked")),
                }
            }
            Some(Loading::World { path, map }) => {
                self.loading_shown.set(false);
                match FlightSim::new(app, &path, map) {
                    Ok(sim) => {
                        return SceneCommand::push(sim);
                    }
                    Err(e) => self.fail(format!("{path}: {e}")),
                }
            }
            still_reading => self.loading = still_reading,
        }

        if let Some(loading) = &self.loading {
            let dots = ".".repeat((app.seconds * 2.0) as usize % 4);
            *self.loading_step.borrow_mut() = format!("{}{dots:<3}", loading.step());
        }
        SceneCommand::None
    }

    fn fail(&mut self, error: String) {
        self.loading = None;
        self.loading_shown.set(false);
        self.show_status(error, STYLE.negative);
    }

    fn show_status(&self, text: impl Into<String>, color: Vec4) {
        *self.status.borrow_mut() = text.into();
        self.status_color.set(color);
    }
}

impl Scene for Menu {
    fn update(&mut self, app: &App) -> SceneCommand {
        // While loading, only the loading screen moves
        if self.loading.is_some() {
            let command = self.keep_loading(app);
            self.ui.update(app); // after, so the new step's text is up before it's drawn
            return command;
        }

        // Maps that have finished drawing
        let finished: Vec<String> = self
            .drawing
            .iter()
            .filter(|(_, thread)| thread.is_finished())
            .map(|(path, _)| path.clone())
            .collect();
        for path in finished {
            let drawn = self.drawing.remove(&path).map(|thread| {
                thread
                    .join()
                    .unwrap_or_else(|_| Err("drawing it panicked".to_string()))
            });
            if let Some(Err(why)) = drawn {
                self.unmappable.insert(path.clone(), why);
            }
            if let Some(i) = self.selected.filter(|&i| self.scenarios[i].0 == path) {
                self.select(&app.renderer, i);
            }
        }

        if app.input.key_pressed(Scancode::Escape) {
            return SceneCommand::Pop(1);
        }

        for msg in self.ui.update(app) {
            match msg {
                Msg::Select(i) => self.select(&app.renderer, i),
                Msg::Fly => {
                    if let Some(i) = self.selected {
                        self.start_loading(self.scenarios[i].0.clone());
                    }
                }
                Msg::Quit => return SceneCommand::Pop(1),
            }
        }
        SceneCommand::None
    }

    fn render(&mut self, app: &App) {
        app.renderer.clear(vec4(0.05, 0.06, 0.08, 1.0));
        if let Some(route) = self.route.borrow().as_ref() {
            draw_backdrop(app, &route.overview);
        }
        self.ui.render(app);
    }
}

fn read_terrain(tiff: &str) -> Result<Heightmap, String> {
    Heightmap::load_geotiff(&format!("assets/{tiff}")).map_err(|e| e.to_string())
}

/// Read just the ground under `bounds`, and bake `scenario`'s map from it
fn draw_map(tiff: &str, scenario: &str, bounds: &Bounds) -> Result<(), String> {
    let map = Heightmap::load_geotiff_window(&format!("assets/{tiff}"), bounds)
        .map_err(|e| e.to_string())?;
    overview::bake(&map, scenario).map_err(|e| e.to_string())
}

/// The overview for `tiff`, loading it the first time it's asked for
fn overview_for(
    overviews: &mut HashMap<String, Rc<Overview>>,
    renderer: &RenderContext,
    tiff: &str,
) -> Option<Rc<Overview>> {
    if let Some(overview) = overviews.get(tiff) {
        return Some(overview.clone());
    }
    let overview = Rc::new(Overview::load(renderer, tiff)?);
    overviews.insert(tiff.to_string(), overview.clone());
    Some(overview)
}

/// Where `s` goes, in order, [lat, lon] and what's there
fn stops(s: &Scenario, runways: &HashMap<String, Runway>) -> Vec<(f64, f64, Stop)> {
    let threshold = |name: &str| runways.get(name).map(|r| r.start);
    let start = match &s.start {
        Start::Runway { runway } => threshold(runway),
        Start::Final { distance_nm, .. } => {
            runways.get(&s.land).map(|r| out_on_final(r, *distance_nm))
        }
    };
    let sights = s
        .sights
        .iter()
        .map(|sight| (sight.lat, sight.lon, Stop::Sight));
    let rescue = s.rescue.iter().map(|r| {
        (
            r.lat,
            r.lon,
            Stop::Search {
                radius_m: r.search_radius_m,
            },
        )
    });
    let land = threshold(&s.land).map(|(lat, lon)| (lat, lon, Stop::Land));
    start
        .map(|(lat, lon)| (lat, lon, Stop::Start))
        .into_iter()
        .chain(sights)
        .chain(rescue)
        .chain(land)
        .collect()
}

/// `distance_nm` back from `runway`'s threshold along its final approach [lat, lon]
fn out_on_final(runway: &Runway, distance_nm: f64) -> (f64, f64) {
    let (lat, lon) = runway.start;
    let lon_per_lat = lat.to_radians().cos(); // a degree of longitude is this many degrees of latitude long
    let north = runway.start.0 - runway.end.0;
    let east = (runway.start.1 - runway.end.1) * lon_per_lat;
    let back = distance_nm / 60.0 / north.hypot(east); // TIL a nautical mile is a minute of latitude
    (lat + north * back, lon + east * back / lon_per_lat)
}

/// `overview`'s blurred picture, covering the window
fn draw_backdrop(app: &App, overview: &Overview) {
    let window = vec2(app.window_size.x as f32, app.window_size.y as f32);
    let size = overview.backdrop_size;
    let shown = window / (window.x / size.x).max(window.y / size.y);
    let source = Rectangle {
        pos: (size - shown) / 2.0,
        size: shown,
    };
    let screen = Rectangle {
        pos: Vec2::zeros(),
        size: window,
    };
    app.renderer
        .draw_texture(overview.backdrop, screen, source, BACKDROP_TINT);
}

/// `hours` after midnight as "HH:MM"
fn clock_time(hours: f64) -> String {
    let minutes = (hours * 60.0).round() as u32;
    format!("{:02}:{:02}", minutes / 60 % 24, minutes % 60)
}
