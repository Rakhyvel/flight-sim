use crate::{
    airplane::Airplane,
    hud::{Mood, Notice},
    landing::Landing,
};

const FLY_AGAIN: &str = "R to fly again";

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

/// One line of a debrief
pub struct DebriefRow {
    pub label: String,
    pub value: String,
    pub grade: Grade,
    pub fail_title: String,
}

impl DebriefRow {
    pub fn new(
        label: impl Into<String>,
        value: impl Into<String>,
        grade: Grade,
        fail_title: impl Into<String>,
    ) -> Self {
        Self {
            label: label.into(),
            value: value.into(),
            grade,
            fail_title: fail_title.into(),
        }
    }
}

pub trait Objective {
    /// Watch `plane` through the last `dt` [s]
    fn update(&mut self, plane: &Airplane, dt: f64);

    /// How it's gone so far, for the debrief
    fn rows(&self) -> Vec<DebriefRow>;
}

/// How a flight ended
#[derive(Clone, Copy)]
pub enum Ending {
    Stopped,
    Crashed(&'static str),
}

/// His feet are light and nimble. He never sleeps. He says that he will never die. He dances in light and in
/// shadow and he is a great favorite. He never sleeps, the judge. He is dancing, dancing. He says that he
/// will never die.
pub struct Judge {
    landing: Landing,
    objectives: Vec<Box<dyn Objective>>,
}

impl Judge {
    pub fn new(landing: Landing, objectives: Vec<Box<dyn Objective>>) -> Self {
        Self {
            landing,
            objectives,
        }
    }

    pub fn update(&mut self, plane: &Airplane, dt: f64) {
        if self.landing.ending().is_some() {
            return; // nothing counts after the flight is over...
        }
        self.landing.update(plane, dt);
        for objective in &mut self.objectives {
            objective.update(plane, dt);
        }
    }

    pub fn debrief(&self) -> Option<Notice> {
        let ending = self.landing.ending()?;
        let rows: Vec<DebriefRow> = self
            .objectives
            .iter()
            .flat_map(|o| o.rows())
            .chain(self.landing.rows())
            .collect();

        let mut lines = Vec::new();
        let title = match ending {
            Ending::Crashed(why) => {
                lines.push((why.to_string(), Mood::Bad));
                "Crashed".to_string()
            }
            Ending::Stopped => title(&rows),
        };

        let width = rows.iter().map(|r| r.label.len()).max().unwrap_or(0) + 2;
        lines.extend(
            rows.iter()
                .map(|r| (format!("{:<width$}{}", r.label, r.value), r.grade.mood())),
        );
        lines.push((FLY_AGAIN.to_string(), Mood::Plain));

        Some(Notice { title, lines })
    }
}

fn title(rows: &[DebriefRow]) -> String {
    let worst = rows
        .iter()
        .map(|r| r.grade)
        .fold(Grade::Butter, |a, b| if b > a { b } else { a });
    match worst {
        Grade::Butter => "Butter!".to_string(),
        Grade::Good => "Nice landing".to_string(),
        Grade::Poor => "Could be worse".to_string(),
        Grade::Fail => rows
            .iter()
            .find(|r| r.grade == Grade::Fail)
            .map_or_else(String::new, |r| r.fail_title.clone()),
    }
}
