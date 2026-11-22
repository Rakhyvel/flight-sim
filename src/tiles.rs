use nalgebra_glm::DVec3;

/// Quads along each side of every tile, at every level
pub const TILE_QUADS: u32 = 64;

/// A square of terrain
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub struct Tile {
    /// How detailed the tile is, 0 being fully detailed
    pub level: u32,
    /// Counted in tiles of this level from the map's corner
    pub x: i32,
    pub z: i32,
}

impl Tile {
    /// The number of heightmap cells across this tile is
    pub fn cells(&self) -> u32 {
        TILE_QUADS << self.level
    }

    /// The number of cells between vertices this tile is
    pub fn step(&self) -> u32 {
        1 << self.level
    }

    /// The corner with the smallest x and z, in cells
    pub fn origin(&self) -> (i32, i32) {
        let n = self.cells() as i32;
        (self.x * n, self.z * n)
    }

    /// The 4 tiles one level down that make up this one. Not for level 0!
    pub fn children(&self) -> [Tile; 4] {
        let (level, x, z) = (self.level - 1, self.x * 2, self.z * 2);
        [
            Tile { level, x, z },
            Tile { level, x: x + 1, z },
            Tile { level, x, z: z + 1 },
            Tile {
                level,
                x: x + 1,
                z: z + 1,
            },
        ]
    }
}

pub struct TileTree {
    /// Level of the biggest tiles, which cover the map
    pub root_level: u32,
    /// Map size [cells]
    pub map_cells: (u32, u32),
    /// Meters per cell, along x and z
    pub spacing: (f64, f64),
    /// Split a tree when its closer than this many times its own size. Higher is more detail
    pub split: f64,
    /// Nothing farther than this is drawn [m]
    pub max_distance: f64,
}

impl TileTree {
    /// Get the biggest tiles, covering the whole map
    fn roots(&self) -> Vec<Tile> {
        let n = TILE_QUADS << self.root_level;
        let level = self.root_level;
        let (w, d) = (
            self.map_cells.0.div_ceil(n) as i32,
            self.map_cells.1.div_ceil(n) as i32,
        );
        (0..w)
            .flat_map(|x| (0..d).map(move |z| Tile { level, x, z }))
            .collect()
    }

    /// How far `pos` is from the closest part of `tile` along the ground [m]
    fn ground_distance(&self, tile: Tile, pos: DVec3) -> f64 {
        let (ox, oz) = tile.origin();
        let n = tile.cells() as f64;
        let (sx, sz) = self.spacing;
        let (x0, z0) = (ox as f64 * sx, oz as f64 * sz);
        let dx = (x0 - pos.x).max(pos.x - (x0 + n * sx)).max(0.0);
        let dz = (z0 - pos.z).max(pos.z - (z0 + n * sz)).max(0.0);
        (dx * dx + dz * dz).sqrt()
    }

    /// How far `pos` is from the closest part of the _terrain_ of `tile` [m]
    fn distance(&self, tile: Tile, pos: DVec3) -> f64 {
        self.ground_distance(tile, pos).hypot(pos.y.max(0.0))
    }

    /// Determine if `tile` is close enough to `pos` to be split up. Level 0 tiles can't be split up.
    fn wants_split(&self, tile: Tile, pos: DVec3) -> bool {
        let size = tile.cells() as f64 * self.spacing.0.max(self.spacing.1);
        tile.level > 0 && self.distance(tile, pos) < self.split * size
    }

    /// The children that are on the map and in range
    fn children(&self, tile: Tile, pos: DVec3) -> Vec<Tile> {
        if tile.level == 0 {
            // these guys are too fine and have no children (many such cases...)
            return vec![];
        }
        tile.children()
            .into_iter()
            .filter(|&c| self.on_map(c) && self.in_range(c, pos))
            .collect()
    }

    /// Determine if `tile` is within the bounds of the map
    fn on_map(&self, tile: Tile) -> bool {
        let (ox, oz) = tile.origin();
        ox >= 0 && oz >= 0 && (ox as u32) < self.map_cells.0 && (oz as u32) < self.map_cells.1
    }

    /// Determine if `tile` is within the max distance, relative to `pos`
    fn in_range(&self, tile: Tile, pos: DVec3) -> bool {
        self.ground_distance(tile, pos) < self.max_distance
    }

    /// The tiles we'd draw from `pos` if every one was built, closest first
    pub fn ideal(&self, pos: DVec3) -> Vec<Tile> {
        let mut out = Vec::new();
        for root in self.roots().into_iter().filter(|&r| self.in_range(r, pos)) {
            self.collect_ideal(root, pos, &mut out);
        }
        out.sort_by(|a, b| self.distance(*a, pos).total_cmp(&self.distance(*b, pos)));
        out
    }

    /// Add the tiles we'd ideally draw to cover `tile`'s area from `pos`
    fn collect_ideal(&self, tile: Tile, pos: DVec3, out: &mut Vec<Tile>) {
        let kids = self.children(tile, pos);
        if self.wants_split(tile, pos) && !kids.is_empty() {
            // Too close for this much ground per triangle, cover it with its finer children
            for c in kids {
                self.collect_ideal(c, pos, out);
            }
        } else {
            // Detailed enough from here
            out.push(tile);
        }
    }

    /// The tiles to draw from `pos` using only `loaded` ones, as close to "ideal" as possible, never
    /// overlapping, and without holes once the area's been built
    pub fn drawable(&self, pos: DVec3, loaded: impl Fn(Tile) -> bool) -> Vec<Tile> {
        let mut out = Vec::new();
        for root in self.roots().into_iter().filter(|&r| self.in_range(r, pos)) {
            self.collect_drawable(root, pos, &loaded, &mut out);
        }
        out
    }

    /// Whether `tile`s area can be drawn completely from tiles that are `loaded`
    fn ready(&self, tile: Tile, pos: DVec3, loaded: &impl Fn(Tile) -> bool) -> bool {
        if loaded(tile) {
            return true;
        }
        let kids = self.children(tile, pos);
        if kids.is_empty() {
            return false; // nothing finer to fall back on
        }

        if self.wants_split(tile, pos) {
            // We're moving closer, its children have to be able to cover it, at any depth
            kids.iter().all(|&c| self.ready(c, pos, loaded))
        } else {
            // We're moving away, the finer tiles are still around until this one's built
            kids.iter().all(|&c| loaded(c))
        }
    }

    /// Add the tiles to draw for `tile`'s area
    fn collect_drawable(
        &self,
        tile: Tile,
        pos: DVec3,
        loaded: &impl Fn(Tile) -> bool,
        out: &mut Vec<Tile>,
    ) {
        let kids = self.children(tile, pos);
        if self.wants_split(tile, pos)
            && !kids.is_empty()
            && kids.iter().all(|&c| self.ready(c, pos, loaded))
        {
            // Close, and its children can cover it, so go finer
            for c in kids {
                self.collect_drawable(c, pos, loaded, out);
            }
        } else if loaded(tile) {
            // Detailed enough, or its children aren't ready yet
            out.push(tile)
        } else {
            // Not built yet, draw whatever finer tiles are still loaded
            out.extend(kids.into_iter().filter(|&c| loaded(c)))
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn tree() -> TileTree {
        TileTree {
            root_level: 4,
            map_cells: (4096, 4096),
            spacing: (30.0, 30.0),
            split: 2.0,
            max_distance: 150_000.0,
        }
    }

    #[test]
    fn closest_tiles_are_full_detail_and_far_ones_coarser() {
        let pos = DVec3::new(60_000.0, 100.0, 60_000.0);
        let ideal = tree().ideal(pos);
        assert_eq!(ideal[0].level, 0);
        assert!(ideal.last().unwrap().level > 0);
    }

    #[test]
    fn keeps_drawing_the_parent_until_all_its_children_are_built() {
        let t = tree();
        let pos = DVec3::new(60_000.0, 100.0, 60_000.0);
        let ideal = t.ideal(pos);
        let missing = ideal[0];
        let parent = Tile {
            level: 1,
            x: missing.x.div_euclid(2),
            z: missing.z.div_euclid(2),
        };

        let loaded = |tile: Tile| tile != missing && (ideal.contains(&tile) || tile == parent);
        let draw = t.drawable(pos, loaded);

        assert!(draw.contains(&parent));
        assert!(
            parent.children().iter().all(|c| !draw.contains(c)),
            "no overlap"
        );
    }

    #[test]
    fn draws_the_ideal_tiles_once_everything_is_built() {
        let t = tree();
        let pos = DVec3::new(60_000.0, 3_000.0, 60_000.0);
        let mut ideal = t.ideal(pos);
        let mut draw = t.drawable(pos, |tile| ideal.contains(&tile));
        let key = |t: &Tile| (t.level, t.x, t.z);
        ideal.sort_by_key(key);
        draw.sort_by_key(key);
        assert_eq!(ideal, draw);
    }
}
