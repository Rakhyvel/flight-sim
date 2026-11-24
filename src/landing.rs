use std::f64::consts::{PI, TAU};

use nalgebra_glm::{vec2, DVec2};

use crate::{
    airplane::Airplane,
    hud::{Mood, Notice},
    units::FT_PER_M,
    world::Strip,
};

/// In the air at least this long before a touchdown counts [s], so hops on the takeoff roll don't count
const AIRBORNE_TIME: f64 = 2.0;
/// Off the ground at least this long counts as a bounce [s]
const BOUNCE_TIME: f64 = 0.3;
/// Slower than this on the ground counts as stopped [m/s]
const STOPPED_SPEED: f64 = 1.0;
/// Wheels coming down faster than this break the gear [m/s]
const CRASH_SINK: f64 = 4.5;
/// Anything but a wheel hitting faster than this is a crash [m/s], slower is a scrape
const CRASH_IMPACT: f64 = 2.0;

const AGAIN: &str = "R to fly again";

// TODO: Common between landing and tour (and likely other objectives), doesn't belong here
/// One line of a debrief
pub struct DebriefRow {
    pub label: String,
    pub value: String,
    pub grade: Grade,
}

// TODO: Common between landing and tour (and likely other objectives), doesn't belong here
/// How well one part of a landing went, best first
#[derive(Clone, Copy, PartialEq, PartialOrd)]
pub enum Grade {
    Butter,
    Good,
    Poor,
    Fail,
}

impl Grade {
    pub fn by(value: f64, butter: f64, good: f64, poor: f64) -> Self {
        if value <= butter {
            Grade::Butter
        } else if value <= good {
            Grade::Good
        } else if value <= poor {
            Grade::Poor
        } else {
            Grade::Fail
        }
    }

    fn mood(self) -> Mood {
        match self {
            Grade::Butter | Grade::Good => Mood::Good,
            Grade::Poor => Mood::Warn,
            Grade::Fail => Mood::Bad,
        }
    }
}

/// The moment the wheels met the ground
#[derive(Clone, Copy)]
struct Touchdown {
    /// Index into `Landing::runways` of the one it came down on, the target if it missed them all
    runway: usize,
    /// How fast the wheels came down [m/s]
    sink_rate: f64,
    /// How far past the threshold, and how far right of the centerline [m]
    spot: DVec2,
    /// Heading minus the runway's [rad], + = nose right
    crab: f64,
    bounces: u32,
}

#[derive(Clone, Copy)]
enum Phase {
    /// On the ground before the flight has really started
    Departing,
    /// In the air for this long [s]
    Flying(f64),
    /// On the ground after a touchdown, or off it again for this long [s]
    Rolling(Touchdown, f64),
    /// Stopped after a touchdown, and whether it was on the runway
    Landed(Touchdown, bool),
    Crashed(&'static str),
}

/// Grades a landing on a runway
pub struct Landing {
    /// Every runway on the map by name, to tell which one the plane came down on
    runways: Vec<(String, Strip)>,
    /// Index in `runways` of the one to land on
    target: usize,
    phase: Phase,
    /// Times the airframe has touched the ground
    scrapes: u32,
    /// Time since the airframe last touched the ground [s]
    since_scrape: f64,
}

impl Landing {
    /// Watch for a landing on `strip`, by a plane starting in the air or on the ground
    pub fn new(target: &str, runways: Vec<(String, Strip)>, airborne: bool) -> Option<Self> {
        let target = runways.iter().position(|(name, _)| name == target)?;
        Some(Self {
            runways,
            target,
            phase: if airborne {
                Phase::Flying(AIRBORNE_TIME)
            } else {
                Phase::Departing
            },
            scrapes: 0,
            since_scrape: f64::INFINITY,
        })
    }

    /// Follow `plane` through the last `dt` [s]
    pub fn update(&mut self, plane: &Airplane, dt: f64) {
        let contact = plane.contact();
        let here = vec2(plane.pos.x, plane.pos.z);
        let ground_speed = vec2(plane.vel.x, plane.vel.z).norm();

        // Hitting too hard ends it whatever's going on
        match self.phase {
            Phase::Landed(..) | Phase::Crashed(_) => return,
            _ if contact.airframe_speed > CRASH_IMPACT => {
                self.phase = Phase::Crashed("Those aren't the wheels")
            }
            Phase::Departing => {} // the plane settles onto its gear at the start
            Phase::Flying(t) if t < AIRBORNE_TIME => {} // a hop, or that first settles
            _ if contact.wheel_speed > CRASH_SINK => self.phase = Phase::Crashed("CRUNCH!"),
            _ => {}
        }

        // Count each time the airframe touches, not each frame it drags
        if contact.airframe {
            if self.since_scrape > BOUNCE_TIME {
                self.scrapes += 1;
            }
            self.since_scrape = 0.0;
        } else {
            self.since_scrape += dt;
        }

        // transition the phase depending on the plane's state
        self.phase = match self.phase {
            Phase::Departing if !contact.wheels => Phase::Flying(0.0),
            Phase::Flying(t) if contact.wheels && t < AIRBORNE_TIME => Phase::Departing, // tis but a hop
            Phase::Flying(_) if contact.wheels => {
                Phase::Rolling(self.touchdown(plane, contact.wheel_speed), 0.0)
            }
            Phase::Flying(t) => Phase::Flying(t + dt),
            Phase::Rolling(_, air) if !contact.wheels && air + dt > AIRBORNE_TIME => {
                Phase::Flying(air + dt) // went around, so the next touchdown gets graded instead
            }
            Phase::Rolling(td, air) if !contact.wheels => Phase::Rolling(td, air + dt),
            Phase::Rolling(mut td, air) => {
                if air > BOUNCE_TIME {
                    td.bounces += 1;
                }
                if ground_speed < STOPPED_SPEED {
                    Phase::Landed(td, self.runways[td.runway].1.contains(here))
                } else {
                    Phase::Rolling(td, 0.0)
                }
            }
            phase => phase,
        }
    }

    // TODO: Common between landing and tour (and likely other objectives), doesn't belong here
    /// How it went, once the plane has stopped or crashed, with `before` listed ahead of the landing
    pub fn debrief(&self, before: &[DebriefRow]) -> Option<Notice> {
        let (td, stopped_on_runway) = match self.phase {
            Phase::Landed(td, on_runway) => (td, on_runway),
            Phase::Crashed(why) => {
                return Some(Notice {
                    title: "Crashed".to_string(),
                    lines: vec![
                        (why.to_string(), Mood::Bad),
                        (AGAIN.to_string(), Mood::Plain),
                    ],
                })
            }
            _ => return None,
        };
        let (name, s) = &self.runways[td.runway];
        let wrong = td.runway != self.target;

        // Grade each part
        let sink_fpm = td.sink_rate * FT_PER_M * 60.0;
        let (past, right) = (td.spot.x, td.spot.y);
        let zone = if past < 0.0 {
            Grade::Fail
        } else {
            Grade::by(past / s.length, 1.0 / 3.0, 0.5, 1.0)
        };
        let crab_deg = td.crab.to_degrees().abs();
        let rows = [
            (
                "Runway",
                if wrong {
                    format!("{name}, not {}", self.runways[self.target].0)
                } else {
                    name.clone()
                },
                if wrong { Grade::Fail } else { Grade::Butter },
            ),
            (
                "Sink rate",
                format!("{sink_fpm:.0} fpm"),
                Grade::by(sink_fpm, 200.0, 400.0, 700.0),
            ),
            (
                "Touchdown",
                if past < 0.0 {
                    format!("{:.0} ft short", -past * FT_PER_M)
                } else {
                    format!("{:.0} ft past the threshold", past * FT_PER_M)
                },
                zone,
            ),
            (
                "Centerline",
                format!(
                    "{:.0} ft {}",
                    right.abs() * FT_PER_M,
                    if right < 0.0 { "left" } else { "right" }
                ),
                Grade::by(right.abs(), 0.1 * s.width, 0.25 * s.width, 0.5 * s.width),
            ),
            (
                "Crab",
                format!("{crab_deg:.0} deg"),
                Grade::by(crab_deg, 3.0, 8.0, 20.0),
            ),
            (
                "Bounces",
                td.bounces.to_string(),
                Grade::by(td.bounces as f64, 0.0, 1.0, 3.0),
            ),
            (
                "Scrapes",
                self.scrapes.to_string(),
                Grade::by(self.scrapes as f64, 0.0, 0.0, 1.0),
            ),
            (
                "Stopped",
                if stopped_on_runway {
                    "on the runway"
                } else {
                    "off the runway"
                }
                .to_string(),
                if stopped_on_runway {
                    Grade::Butter
                } else {
                    Grade::Fail
                },
            ),
        ];

        // Rows from before the landing go first
        let rows: Vec<(&str, String, Grade)> = before
            .iter()
            .map(|row| (row.label.as_str(), row.value.clone(), row.grade))
            .chain(rows)
            .collect();

        // The worst part sets the headline
        let worst = rows
            .iter()
            .map(|r| r.2)
            .fold(Grade::Butter, |a, b| if b > a { b } else { a });
        let title = match worst {
            _ if wrong => "Advise ready to copy a phone number",
            Grade::Butter => "Butter!",
            Grade::Good => "Nice landing",
            Grade::Poor => "Could be worse",
            Grade::Fail => "Any landing you can walk away from...", // TODO: Change to something else if its Fail because of missed sights
        };

        let width = rows.iter().map(|r| r.0.len()).max().unwrap_or(0) + 2;
        let mut lines: Vec<(String, Mood)> = rows
            .into_iter()
            .map(|(label, value, grade)| (format!("{label:<width$}{value}"), grade.mood())) // the HUD font is monospaced, so this lines up
            .collect();
        lines.push((AGAIN.to_string(), Mood::Plain));
        Some(Notice {
            title: title.to_string(),
            lines,
        })
    }

    fn touchdown(&self, plane: &Airplane, sink_rate: f64) -> Touchdown {
        let here = vec2(plane.pos.x, plane.pos.z);
        let heading = plane.heading();

        // Grade against the runway it came down on, or the target if it missed them all
        let runway = self.runway_at(here, heading).unwrap_or(self.target);
        let strip = &self.runways[runway].1;

        Touchdown {
            runway,
            sink_rate,
            spot: strip.local(here),
            crab: angle_off(heading, strip.heading()),
            bounces: 0,
        }
    }

    /// Get the index of the runway at `p` with heading closest to `heading`
    fn runway_at(&self, p: DVec2, heading: f64) -> Option<usize> {
        let off = |s: &Strip| angle_off(heading, s.heading()).abs();
        self.runways
            .iter()
            .enumerate()
            .filter(|(_, (_, s))| s.contains(p))
            .min_by(|(_, (_, a)), (_, (_, b))| off(a).total_cmp(&off(b)))
            .map(|(i, _)| i)
    }
}

/// `a` minus `b`, wrapped to -pi..pi [rad]
fn angle_off(a: f64, b: f64) -> f64 {
    (a - b + PI).rem_euclid(TAU) - PI
}
