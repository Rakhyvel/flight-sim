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

/// Info about a tile's Terrain
#[derive(Clone, Copy)]
pub struct TileHeight {
    /// Highest point [m]
    pub peak: f32,
    /// The most the tile's mesh is off from the full detail terrain [m]
    pub error: f32,
}

/// `TileHeight` for every tile at every level
pub struct TileHeights {
    /// Per level (width, depth) in tiles, and each tile's heights, rows along z
    levels: Vec<((i32, i32), Vec<TileHeight>)>,
}

impl TileHeights {
    /// For a heightmap `w` x `d` cells, up to `top_level`
    pub fn new(w: u32, d: u32, top_level: u32, height: impl Fn(i32, i32) -> f32) -> Self {
        let (w, d) = (w as i32, d as i32);
        let mut levels: Vec<((i32, i32), Vec<TileHeight>)> = Vec::new();
        for level in 0..=top_level {
            let cells = (TILE_QUADS << level) as i32;
            let size = ((w + cells - 1) / cells, (d + cells - 1) / cells);
            let index = |x: i32, z: i32| ((z / cells) * size.0 + x / cells) as usize;
            let empty = TileHeight {
                peak: f32::MIN,
                error: 0.0,
            };
            let mut tiles = vec![empty; (size.0 * size.1) as usize];

            if level == 0 {
                // Full detail, so only the peaks
                for z in 0..d {
                    for x in 0..w {
                        let t = &mut tiles[index(x, z)];
                        t.peak = t.peak.max(height(x, z));
                    }
                }
            } else {
                // The worst of the 4 tiles below
                let ((bw, bd), below) = &levels[level as usize - 1];
                for bz in 0..*bd {
                    for bx in 0..*bw {
                        let b = below[(bz * bw + bx) as usize];
                        let t = &mut tiles[((bz / 2) * size.0 + bx / 2) as usize];
                        t.peak = t.peak.max(b.peak);
                        t.error = t.error.max(b.error);
                    }
                }

                // Plus how far the level below's vertices are from this level's surface
                let s = 1 << level; // cells between this level's vertices
                let mut off = vec![0.0f32; tiles.len()];
                for z in (0..d).step_by(s as usize / 2) {
                    for x in (0..w).step_by(s as usize / 2) {
                        let (x0, z0) = (x - x % s, z - z % s);
                        let (fx, fz) = ((x - x0) as f32 / s as f32, (z - z0) as f32 / s as f32);
                        let near = height(x0, z0) + (height(x0 + s, z0) - height(x0, z0)) * fx;
                        let far =
                            height(x0, z0 + s) + (height(x0 + s, z0 + s) - height(x0, z0 + s)) * fx;
                        let surface = near + (far - near) * fz;
                        let i = index(x, z);
                        off[i] = off[i].max((height(x, z) - surface).abs());
                    }
                }
                for (t, off) in tiles.iter_mut().zip(off) {
                    t.error += off; // errors stack up from level to level
                }
            }
            levels.push((size, tiles));
        }
        Self { levels }
    }

    /// Get the height info for `tile`, `None` if it's off the map
    pub fn get(&self, tile: Tile) -> Option<TileHeight> {
        let ((w, d), tiles) = self.levels.get(tile.level as usize)?;
        let inside = tile.x >= 0 && tile.z >= 0 && tile.x < *w && tile.z < *d;
        inside.then(|| tiles[(tile.z * w + tile.x) as usize])
    }
}

pub struct TileTree {
    /// Level of the biggest tiles, which cover the map
    pub root_level: u32,
    /// Map size [cells]
    pub map_cells: (u32, u32),
    /// Meters per cell, along x and z
    pub spacing: (f64, f64),
    /// Never split a tile farther than this many times its own size
    pub split: f64,
    /// Always split a tile closer than this many times its own size, so its colors stay sharp
    pub texture_split: f64,
    /// Nothing farther than this is drawn [m]
    pub max_distance: f64,

    /// Info about every tile's terrain
    pub heights: TileHeights,
    /// Split a tile when its bumps would be off by more than this on screen [px]
    pub max_error_px: f64,
    /// Screen pixels per radian of view
    pub pixels_per_radian: f64,
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

    /// Whether `tile` is too coarse to draw from `pos`, either its colors or its shape
    fn wants_split(&self, tile: Tile, pos: DVec3) -> bool {
        if tile.level == 0 {
            return false;
        }
        let size = tile.cells() as f64 * self.spacing.0.max(self.spacing.1);
        let dist = self.distance(tile, pos).max(1.0);
        if dist >= self.split * size {
            return false; // far enough that we'd never want more
        }
        let error = self.heights.get(tile).map_or(0.0, |h| h.error as f64);
        dist < self.texture_split * size // close enough that the texture would look blurry
            || error / dist * self.pixels_per_radian > self.max_error_px // bumps that would visibly be wrong
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
            texture_split: 2.0,
            max_distance: 150_000.0,
            heights: TileHeights::new(4096, 4096, 4, |_, _| 100.0),
            max_error_px: 2.0,
            pixels_per_radian: 1800.0,
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

    #[test]
    fn bumpy_terrain_gets_more_detail_than_flat() {
        let pos = DVec3::new(60_000.0, 3_000.0, 60_000.0);
        let bumpy = TileTree {
            heights: TileHeights::new(4096, 4096, 4, |x, z| ((x * 7 + z * 13) % 50) as f32 * 10.0),
            ..tree()
        };
        assert!(bumpy.ideal(pos).len() > tree().ideal(pos).len());
    }

    #[test]
    fn flat_terrain_gets_less_detail_than_bumpy() {
        let pos = DVec3::new(60_000.0, 3_000.0, 60_000.0);
        let flat = TileTree {
            texture_split: 0.5,
            ..tree()
        };
        let bumpy = TileTree {
            heights: TileHeights::new(4096, 4096, 4, |x, z| ((x * 7 + z * 13) % 50) as f32 * 10.0),
            texture_split: 0.5,
            ..tree()
        };
        assert!(flat.ideal(pos).len() < bumpy.ideal(pos).len());
    }
}
