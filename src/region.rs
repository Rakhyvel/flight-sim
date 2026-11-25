use serde::Deserialize;

#[derive(Deserialize)]
pub struct RegionFile {
    pub regions: Vec<Region>,
}

#[derive(Deserialize)]
pub struct Region {
    pub name: String,
    pub title: String,
    pub tiff: String,
}

pub fn load_regions(path: &str) -> Vec<Region> {
    let text = std::fs::read_to_string(path).unwrap();
    let regions: RegionFile = toml::from_str(&text).unwrap_or_else(|e| panic!("{path}: {e}"));
    regions.regions
}
