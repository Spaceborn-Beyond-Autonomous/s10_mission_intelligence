# S10 planning, optimization and replanning (Member 3)

This page documents the planning side of `s10_mission_intelligence`: the two
config files, how each mission command becomes waypoints, how the best route
is chosen, and how a running mission is replanned.

```
MissionGraph ──► waypoint_generator ──► route_planner ──► route_optimizer ──► Route
 (Member 2)      names → coordinates     detours around     scores candidates   (waypoints
                                         no-fly zones       with cost_model      + cost)

change event ──► replanner:  adapt graph ─► plan + optimize the REMAINDER ─► new graph + new route
```

**Frame.** Everything is local ENU, in metres, origin = home/base:
`x` = metres east, `y` = metres north, `z` = metres above home. The core never
sees lat/lon; an adapter converts at the system edge.

> Every number in the shipped `config/*.yaml` files is placeholder test data,
> not a real site and not a measurement.

---

## 1. `config/scenarios.yaml`

Describes the world: where things are, flight defaults, and the zones.
Loaded by `s10_scenario_load()` into a fixed-size `ScenarioTable` (no heap,
nothing to free).

| Section | Shape | Used for |
|---|---|---|
| `frame` | `local_enu_m` (the only accepted value) | frame marker |
| `home` | `{x, y, z}` | start of every mission, target of RTH |
| `defaults` | see below (**required**) | flight parameters |
| `points` | `name: {x, y}` | single locations: GOTO, PICKUP, DROPOFF, LAND |
| `areas` | `name: {x_min, y_min, x_max, y_max}` | SCAN regions; patrol corners |
| `routes` | `name:` then a list of `{x, y}` | PATROL loops |
| `no_fly` | `name: {x_min, y_min, x_max, y_max}` | cannot be flown through or over |
| `gps_denied` | `name: {x_min, y_min, x_max, y_max}` | flyable, but adds risk |

`defaults` keys: `cruise_alt_m`, `cruise_speed_mps`, `pickup_hold_s`,
`dropoff_hold_s`, `scan_spacing_m`, `detour_margin_m`.

```yaml
frame: local_enu_m
home: {x: 0, y: 0, z: 0}
defaults:
  cruise_alt_m: 30
  cruise_speed_mps: 8
  pickup_hold_s: 10
  dropoff_hold_s: 10
  scan_spacing_m: 20
  detour_margin_m: 10
points:
  warehouse: {x: 150, y: 40}
areas:
  north field: {x_min: 300, y_min: 500, x_max: 500, y_max: 700}
routes:
  perimeter:
    - {x: 0, y: 0}
    - {x: 200, y: 0}
    - {x: 200, y: 200}
    - {x: 0, y: 200}
no_fly:
  airport zone: {x_min: 200, y_min: 100, x_max: 300, y_max: 220}
gps_denied:
  urban canyon: {x_min: 320, y_min: 120, x_max: 380, y_max: 260}
```

**Name matching.** Names (YAML keys and the text in graph nodes) are lowercased,
trimmed, internal whitespace collapsed, and leading `at ` / `the ` / `in ` /
`to ` stripped, so `"at the Warehouse"` finds `warehouse`. A name that does not
fit in 63 characters is treated as empty, never truncated. Lookup order is
`points`, then `areas`, then `routes`; the first hit wins. Names must be unique
*within* a section (a duplicate is a load error), but the same name may appear
in two different sections.

**Validation.** The load fails (and the table is left zeroed, never half
filled) when:
- the file is missing, is not valid YAML, or has no `defaults` section;
- a value is not a number, or a key/section is unknown (a value under an
  unknown section or key is rejected; bare scalars inside a list are rejected);
- `cruise_speed_mps`, `scan_spacing_m` or `detour_margin_m` is `<= 0`, or
  `cruise_alt_m` or a hold time is negative;
- any rectangle (area, no-fly, gps-denied) has `x_min >= x_max` or
  `y_min >= y_max`;
- a route has no points;
- a table exceeds its maximum: **32** points, **16** areas, **8** routes,
  **32** points per route, **16** no-fly zones, **16** gps-denied zones.

Problems are reported on `stderr` with the line or entry name.

---

## 2. `config/objectives.yaml`

How routes are scored. Loaded and validated by `s10_objectives_load()`
(`ObjectiveConfig`). Every section is `section → key → number`.

| Section | Keys | Rule |
|---|---|---|
| `weights` | `fuel`, `time`, `risk` | each `>= 0`, not all zero |
| `references` | `fuel`, `time_s` | both `> 0`; a route costing exactly this scores 1.0 on that axis |
| `cost` | `fuel_per_m`, `fuel_ref_speed_mps`, `fuel_speed_exponent`, `fuel_per_hover_s`, `gps_denied_risk_per_s` | `fuel_ref_speed_mps > 0`, the rest `>= 0` |
| `speed_profiles` | `name: factor` | 1 to 4 profiles, each factor `> 0`, unique names, names under 16 characters, file order kept |

The load fails on a missing file, broken YAML, a non-numeric value, an unknown
section or key, a duplicate profile name, more than 4 profiles, a missing
`speed_profiles` section, or any rule above being broken. On failure the config
is left zeroed.

**Scoring.**

```
total = w_fuel * (fuel / fuel_ref) + w_time * (time_s / time_ref_s) + w_risk * risk
```

- time = sum of (leg distance / waypoint speed) + every waypoint's `hold_s`
  (distances are 3D);
- fuel = distance * `fuel_per_m` * (speed / `fuel_ref_speed_mps`)^`fuel_speed_exponent`
  per leg, plus total hold time * `fuel_per_hover_s`;
- risk = sum of per-leg risk from a plug-in function, clamped to `[0, 1]`.
  The built-in stub charges `gps_denied_risk_per_s` for time spent inside
  `gps_denied` rectangles. It is a placeholder until Member 4's risk estimator
  replaces it; nothing else in the planner changes when it does.

**Candidates.** The optimizer builds `profile_count * 2` candidate routes:
every speed profile is tried twice, **variant 0** flying straight through
GPS-denied zones (shorter, riskier) and **variant 1** routing around them
(treated as no-fly on a copy of the table; longer, safer). Candidate index =
`profile_index * 2 + variant`. Nothing is random, and a tie goes to the lowest
index, so the same inputs always give the same route.

---

## 3. Per-command waypoint rules

`s10_generate_waypoints()` walks the graph in chain order. "Previous" is where
the prior waypoint ended (home before the first). Unless stated, `z` is
`cruise_alt_m`, speed is `cruise_speed_mps`, and hold is 0. The optimizer
multiplies speeds by the profile factor afterwards.

| Command | Waypoints generated |
|---|---|
| `GOTO` | One at `cmd.position`. A point resolves to itself, an area to its entry corner `(x_min, y_min)`, a route to its first point. Position is required. |
| `PICKUP` | Like `GOTO`, hold = `pickup_hold_s`. Position required. |
| `DROPOFF` | Like `GOTO`, hold = `dropoff_hold_s`. Position required. |
| `HOVER` | One at `cmd.position` if given, otherwise previous `x, y`; hold = `cmd.duration_s` (if `> 0`). |
| `TAKEOFF_CMD` | One at `cmd.position` if given, otherwise previous `x, y` (home at the start), at cruise altitude. |
| `LAND_CMD` | One at `cmd.position` if given, otherwise previous `x, y`; `z = 0`. |
| `RTH` | One at home `x, y`, cruise altitude. |
| `SCAN` | Lawnmower across the area named in `cmd.area` (must be an area). Lines run along `x`, `scan_spacing_m` apart starting at `y_min`, with a last line on `y_max`; direction alternates each line. A first point the drone is already standing on is skipped. No hold. |
| `PATROL_CMD` | The route named in `cmd.route_or_area`, **once** (one lap, the loop is not closed back to point 0). An area name patrols its four corners `(x_min,y_min) → (x_max,y_min) → (x_max,y_max) → (x_min,y_max)`. A point name is an error. |

Example: the shipped `north field` is 200 m tall, so a SCAN has 11 lines; after
a `GOTO north field` (which already stands on the first corner) that is
21 SCAN waypoints.

Each waypoint records `source_node_id` (the graph node it came from; several
waypoints can share one), `action`, `position`, `speed_mps`, `hold_s`, and a
`label` (the normalized location name, for logs only).

**Generation errors** (`S10GenStatus`, with the failing node id and a reason):
`UNKNOWN_LOCATION`, `MISSING_PARAM` (a command lacks a field it needs, or has an
unknown type), `TOO_MANY_WAYPOINTS`, `BAD_GRAPH` (NULL/empty graph, more than one
start node, a cycle, or not a single chain).

### Detours (route planner)

After generation, each leg (start → first waypoint, then each consecutive pair)
is checked against every `no_fly` rectangle. A leg that crosses one is replaced
by the shortest clear path through the start, the goal and the corners of every
no-fly rectangle inflated by `detour_margin_m`. No-fly zones are full height:
they are flown around, never over. Inserted points have
`source_node_id = S10_NO_SOURCE_NODE` (-1), action `GOTO`, hold 0, the speed of
the waypoint they lead into, and an interpolated `z`.

| Plan status | Meaning |
|---|---|
| `S10_PLAN_TARGET_IN_NO_FLY` | a waypoint lies inside a no-fly zone; no detour can fix that |
| `S10_PLAN_NO_PATH` | a leg has no clear path (including a start inside a zone) |
| `S10_PLAN_TOO_MANY_WAYPOINTS` | with detours the route exceeds `S10_MAX_WAYPOINTS` (256) |
| `S10_PLAN_RESUME_NODE_NOT_FOUND` | the resume node produced no waypoint |
| `S10_PLAN_GENERATION_FAILED`, `S10_PLAN_BAD_ARGUMENT` | generation failed / NULL or invalid input |

---

## 4. Replanner contract

`s10_replan()` adapts the running graph to a change event
(`s10_graph_apply_change`, owned by Member 2) and then plans and optimizes the
**remaining** mission from a given node. `s10_replan_replace()` does the same
and swaps the result into the caller's active plan in one step.

### The rules

1. **Pass the NEXT node.** `current_node_id` is the next node to execute; the
   drone is assumed to be at the end of node `current_node_id - 1` (at home when
   it is 0). A node already in progress restarts.
2. **Ids equal array indices.** The first changed node has id `current_node_id`,
   so `current_node_id` is also the resume node in the new graph. The returned
   route's `source_node_id` values refer to the **new** graph.
3. **Replace graph and route together.** The new graph and the new route belong
   to each other; keeping one old half and one new half gives ids that point at
   the wrong nodes. Use `s10_replan_replace()`, which frees the old pair and
   moves the new pair in (no copy) only when the replan succeeded.
4. **The route is the remainder only.** Its first waypoint belongs to the resume
   node (detour points may come first), and its cost covers the remainder only,
   measured from `info.start_used`.
5. **It always replans**, even when the graph comes back unchanged: a changed
   constraint such as a new no-fly zone arrives through the `ScenarioTable`,
   not through the event.
6. **Pass the UPDATED `ScenarioTable`**, and `risk_ctx` must point at that same
   table. Nothing here can detect a stale one.
7. **Failure is safe.** The old graph is never modified. On any non-OK status
   `s10_replan()` leaves its outputs empty, and `s10_replan_replace()` leaves the
   active graph and route byte-for-byte unchanged (same pointers, counts and
   contents) with nothing leaked. The active route may be an empty, zeroed
   `Route` (no plan yet).
8. **`start_override`** (optional real drone position). With it, a HOVER or LAND
   that has no position of its own is placed at the real position (a HOVER at
   its altitude, a LAND on the ground below it) instead of the assumed one.
   Nodes with an explicit position are never moved. The cost is measured from
   the real position, and `info.start_used` reports it.

```c
S10RiskStubCtx ctx = { &table, &cfg };          /* risk_ctx must see the SAME table */
S10ReplanInfo  info;

S10ReplanStatus st = s10_replan_replace(&mission_graph, &mission_route,
                                        next_node_id, &event,
                                        &table, &cfg,
                                        s10_leg_risk_stub, &ctx,
                                        real_pos_or_NULL, &info);
if (st != S10_REPLAN_OK) {
    /* mission_graph and mission_route are still the valid active plan */
}
```

### Status values

| `S10ReplanStatus` | Meaning |
|---|---|
| `S10_REPLAN_OK` | new plan produced (and installed, for `_replace`) |
| `S10_REPLAN_BAD_ARGS` | NULL pointer or invalid config (for `_replace`: NULL active graph or route) |
| `S10_REPLAN_APPLY_FAILED` | the graph could not be adapted (`info.apply_status` says why) |
| `S10_REPLAN_PLAN_FAILED` | the planner rejected the new mission (`info.plan_result` has the failing node and reason) |
| `S10_REPLAN_NO_FEASIBLE_ROUTE` | planned, but no candidate scored |

### How the graph is adapted (Member 2, `mission_builder.h`)

Nodes before `current_node_id` are history and are kept.

| Severity | Effect |
|---|---|
| LOW | no change |
| MEDIUM | a 30 s HOVER is inserted before the current node |
| HIGH | cut at the first pending node matching the event, then RTH |
| CRITICAL | drop all pending nodes, then LAND (battery) or RTH |

---

## 5. Known limits

These are current behaviour, and most are pinned by tests.

- **Resume repeats the node's work.** Resuming at PICKUP or DROPOFF flies the
  hold again; resuming at SCAN flies the whole pattern again.
- **History counts toward the 256-waypoint cap.** Waypoints are generated for
  the whole graph before the flown prefix is dropped, so a mission one waypoint
  over the cap fails to replan even when the remainder is tiny.
- **Only the first positionless HOVER/LAND uses the real position.** With
  `start_override`, any later positionless node still uses the assumed position.
- **A real position inside a no-fly zone fails the replan** (`TARGET_IN_NO_FLY`
  or `NO_PATH`), even for a LAND. A real position inside a GPS-denied zone is
  fine.
- **Chain graphs only.** The graph must be one strict chain
  (`edge_count == node_count - 1`).
- **No-fly and GPS-denied zones are axis-aligned rectangles.** No-fly zones are
  full height. A leg gets at most 8 detour points (`S10_MAX_DETOUR_POINTS`).
- **The "safe" variant can be skipped.** It copies the GPS-denied zones into the
  no-fly table; if that exceeds 16 zones only the direct candidates run. A
  target inside a GPS-denied zone also makes the safe candidates infeasible.
- **Risk is a stub.** GPS-denied exposure time is the only risk term until
  Member 4's estimator is wired in.
- **Fixed capacities:** 100 graph nodes, 256 waypoints, 4 speed profiles,
  and the table sizes in section 1.
- **A stale table is not detected** (rule 6).

---

## 6. Building and testing

From the package root (needs libyaml):

```sh
gcc -std=c99 -Wall -Wextra -Werror -Iinclude -Isrc/planning -Isrc/optimization -Isrc/intent \
    $(find src -name '*.c' ! -name 'test_*.c') tests/test_m3_d5.c -lyaml -lm -o test_day5
./test_day5 config/scenarios.yaml config/objectives.yaml
```

Swap `tests/test_m3_d5.c` for `d2`, `d3` or `d4` (d4 takes only the objectives path and builds its scenario in code), and add
`-fsanitize=address,undefined -O0 -g` for a sanitizer run.

| Suite | Covers | Last recorded |
|---|---|---|
| `test_m3_d2.c` | waypoint generator, graph planner, route planner | 100 / 0 |
| `test_m3_d3.c` | objective manager, cost model, route optimizer | 83 / 0 |
| `test_m3_d4.c` | smoke tests: resume planning, optimizer wrapper, replanner | 59 / 0 |
| `test_m3_d5.c` | whole-mission scenarios, resume, replanning, `s10_replan_replace` | 127 / 0 |
