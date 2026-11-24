//! Stuff for deserializing airport and runway and navaid data from OurAirports.com
use serde::Deserialize;
use std::error::Error;

#[derive(Deserialize)]
struct AirportRow {
    ident: String,
    #[serde(rename = "type")]
    kind: String,
    name: String,
    latitude_deg: f64,
    longitude_deg: f64,
}

pub struct Airport {
    pub ident: String,
    pub name: String,
    pub latitude_deg: f64,
    pub longitude_deg: f64,
}

#[derive(Deserialize)]
struct NavaidRow {
    ident: String,
    #[serde(rename = "type")]
    kind: String,
    latitude_deg: f64,
    longitude_deg: f64,
}

/// A VOR or NDB
pub struct Navaid {
    pub ident: String,
    pub kind: String,
    pub latitude_deg: f64,
    pub longitude_deg: f64,
}

/// A runway, by the lat/long [deg] of the two ends
pub struct Runway {
    pub name: String,
    pub start: (f64, f64),
    pub end: (f64, f64),
    pub width: f32,
}

#[derive(Deserialize)]
struct RunwayRow {
    airport_ident: String,
    closed: u8,
    le_ident: String,
    #[serde(deserialize_with = "csv::invalid_option")]
    le_latitude_deg: Option<f64>,
    #[serde(deserialize_with = "csv::invalid_option")]
    le_longitude_deg: Option<f64>,
    he_ident: String,
    #[serde(deserialize_with = "csv::invalid_option")]
    he_latitude_deg: Option<f64>,
    #[serde(deserialize_with = "csv::invalid_option")]
    he_longitude_deg: Option<f64>,
    #[serde(deserialize_with = "csv::invalid_option")]
    width_ft: Option<f64>,
    surface: String,
}

/// Loads a vec of airports from a csv
pub fn load_airports(
    path: &str,
    keep: impl Fn(f64, f64) -> bool,
) -> Result<Vec<Airport>, Box<dyn Error>> {
    let mut airports = Vec::new();
    for row in csv::Reader::from_path(path)?.deserialize() {
        let row: AirportRow = row?;
        let is_airport = matches!(
            row.kind.as_str(),
            "small_airport" | "medium_airport" | "large_airport"
        );
        if is_airport && keep(row.latitude_deg, row.longitude_deg) {
            airports.push(Airport {
                ident: row.ident,
                name: row.name,
                latitude_deg: row.latitude_deg,
                longitude_deg: row.longitude_deg,
            })
        }
    }
    Ok(airports)
}

/// Loads a vec of navaids from a csv
pub fn load_navaids(
    path: &str,
    keep: impl Fn(f64, f64) -> bool,
) -> Result<Vec<Navaid>, Box<dyn Error>> {
    let mut navaids = Vec::new();
    for row in csv::Reader::from_path(path)?.deserialize() {
        let row: NavaidRow = row?;
        let useful = matches!(
            row.kind.as_str(),
            "VOR" | "VOR-DME" | "VORTAC" | "NDB" | "NDB-DME"
        );
        if useful && keep(row.latitude_deg, row.longitude_deg) {
            navaids.push(Navaid {
                ident: row.ident,
                kind: row.kind,
                latitude_deg: row.latitude_deg,
                longitude_deg: row.longitude_deg,
            })
        }
    }
    Ok(navaids)
}

/// Loads a vec of runways from a csv
pub fn load_runways(
    path: &str,
    keep: impl Fn((f64, f64), (f64, f64)) -> bool,
) -> Result<Vec<Runway>, Box<dyn Error>> {
    let mut runways = Vec::new();
    for row in csv::Reader::from_path(path)?.deserialize() {
        let row: RunwayRow = row?;
        if row.closed != 0 || row.surface.to_uppercase().starts_with("WAT") {
            continue;
        }

        let (Some(le_lat), Some(le_lon), Some(he_lat), Some(he_lon)) = (
            row.le_latitude_deg,
            row.le_longitude_deg,
            row.he_latitude_deg,
            row.he_longitude_deg,
        ) else {
            continue;
        };
        let (le, he) = ((le_lat, le_lon), (he_lat, he_lon));
        if !keep(le, he) {
            continue;
        }
        let width = (row.width_ft.unwrap_or(100.0) * 0.3048) as f32;
        println!("found {} {}", row.airport_ident, row.le_ident);
        runways.push(Runway {
            name: format!("{} {}", row.airport_ident, row.le_ident),
            start: le,
            end: he,
            width,
        });
        runways.push(Runway {
            name: format!("{} {}", row.airport_ident, row.he_ident),
            start: he,
            end: le,
            width,
        });
    }
    Ok(runways)
}
