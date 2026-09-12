# Agent G — Environment data + world checker

## Acceptance (current state, repeatable)

    $ python3 tools/worldcheck.py --frei 0.3 --startseite 1.0        # Exit 0
      ok   maze: 12.5 x 9.5 m, 20 rectangles, 4 starts, goal yes
             widest start->goal path: narrowest point 0.75 m free (needs 0.51 m)
      ok   production: 20.0 x 12.0 m, 10 rectangles, 4 starts, goal yes
             widest start->goal path: narrowest point 1.25 m free (needs 0.51 m)
      ok   track: 18.0 x 11.0 m, 5 rectangles, 4 starts, goal yes
             widest start->goal path: narrowest point 0.75 m free (needs 0.51 m)
      0 worlds with problems
    $ SDL_VIDEODRIVER=dummy python3 -m pytest tests -q          -> 52 passed, 1 skipped
    $ ./lab sim --world {maze,track,production} --robots a1,b2 --headless --seconds 2
                                                               -> no output
    load_world: maze 20 / track 5 / production 10 wall rectangles, 4 starts each (heading
                0, 90, 180, 270 degrees), goals maze (10.75, 7.75), track (9.25, 9.75),
                production (15.25, 6.75)

## Changed

* `worlds/maze.txt`, `worlds/track.txt`, `worlds/production.txt` — rebuilt (generator,
  free space as rectangles, corridor width is a number in the code):
  maze: 4x3 rooms of 5 cells, 1-cell walls, 3-cell gates with a random offset -> stubs
  and dead ends, delivery hall bottom left, goal room top right.
  track: closed ring, roadway 4 cells (2.0 m), straights 5 cells, the lower
  finish straight 7 rows tall as the charging/start area, infield, `|` track edge, `-` start line.
  production: arena with a delivery zone (13 x 7 cells), three table rows at the top, two
  shelf blocks, charging station bottom right, goal in the middle aisle.
  Before: maze corridors partly 0.5 m, track start 0.25 m from the wall, production islands
  4 cm inside the T2 square path (that caused the five wall contacts in T2).
* `tools/worldcheck.py` — 182 lines now. New: option `--startseite <m>` (default 1.0).

## Two checker fixes without which the acceptance run would not have been honest

1. "narrowest point" was measured along the **shortest** cell path; that path hugs the walls,
   and every cell at a wall has 0.25 m of clearance by definition — the maze failed at
   0.35 m even though its corridors are 1.5 m wide. Now the **widest path**
   (maximin over cell clearance, Dijkstra with `heapq`). Reachability BFS, message texts,
   and the output format stay unchanged.
2. `abstand()` did not clamp the point into the rectangle (`max(min(x, x0), x0)` is always
   `x0`) and so measured nonsense everywhere. Correct now, over the merged
   `world.walls` (faster and exact at the corners).

## Criterion: clearance on all sides, not only in +x/+y

`--startseite` first checked only the square in +x/+y. That was a false green: the
start heading changes per participant (0/90/180/270 degrees, rule in `worlds.py`), and T2
drives its square in the start heading — for participants 2..4 it would have pointed at a wall.
The check now uses the hull over all four directions, i.e. **1.26 m of clearance
in every compass direction around every start pose** (start side 1.0 + footprint 0.21 + 0.05
of slack). That is why the zones have to be 7 rows (3.5 m) tall.

## Recommendation for the integrator (not commissioned, so not built)

The maze has become an arena with shelf aisles because of this: **T2 dictates the geometry
of all three worlds through `--startseite`, even though `config/tasks.json` runs T2 only in
`production`.** Two clean ways out:

* `--startseite` only for the worlds that run a square task (e.g. option
  `--startseite-nur production`), then `maze` may go back to 2-cell corridors and
  with that, real maze character again; or
* put T2 on a fourth, flat world "kalibrierung" and build `maze`/`track` tight.

Second finding for the handout: with 0.5 m cells, a 0.42 m robot and 0.51 m of required
clearance a tight maze is geometrically impossible. Corridors need 3 cells, and a 1-cell
gate has only 0.25 m in the middle, so gates have to be 3 cells wide as well.
Real meandering would need `"worlds": {"cell": 0.25}` (double resolution, same checker).
Side effect: `--frei 0.3` rules out 2-cell corridors (0.5 m) by 1 cm; with
`--frei 0.25` they count as passing.

## Do-not-touch rule respected, so these stay open

1. **Save the generator:** the three worlds come from `/tmp/gen.py` (~130 LOC, deterministic,
   seed 20260912). `/tmp` is ephemeral, and ASCII grids cannot carry comments
   (a line with `#` is a wall) without touching `parse_grid`. If the worlds are ever rebuilt
   or generated at 0.25 m resolution: `cp /tmp/gen.py tools/genworlds.py`
   — that would be your file, not mine. Otherwise `worlds/*.txt` are hand-made from now on.
2. `worlds.cell` (0.5 m) appears only in `parse_grid(text, cell=...)` and `World.cell`; the
   sim does not read the cell size from the config. The 0.25 m idea would need
   `load_world(name, cell=...)` threaded through — one line in `worlds.py` (someone else's
   file, untouched).
3. `student/solution.py` describes the lateral stabilization as a "1 m corridor"; the
   track roadway is 2.0 m now. Only a comment, but it tells students a number
   that no longer matches their world.
4. The T4 goal is `spawns[-1]` (charging spot, bottom left, 1.25 m to the nearest wall). The
   charging station bottom right has nothing to do with it — if the handout/world explanation
   suggests "charging station = T4 goal", that is the place to rename
   (e.g. pallet/shelf) instead of softening the task.

LOC: worldcheck.py 182 lines, worlds 19/22/24 lines, notes = this file. No git, nothing
installed, tested headless, not a line touched in `mecanum_lab/` or `student/`.
