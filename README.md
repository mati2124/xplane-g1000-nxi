# X-Plane G1000 NXi

A high-performance glass-cockpit (G1000-style PFD/MFD) for X-Plane, built as
**one C++ engine with two shells** so the same avionics code runs:

1. **Natively inside X-Plane** — an XPLM plugin drawn in the sim's render pass.
2. **As a standalone desktop program** — its own window + 60 fps loop, fed by
   X-Plane over the network.

![Primary Flight Display with inset map](docs/screenshots/pfd.png)

![Multi-Function Display — Navigation Map page](docs/screenshots/mfd.png)

Screenshots from the standalone shell running on the built-in mock feed (PFD with
inset map enabled; MFD on the Navigation Map page).

## Why C++ and this structure

- Native in-cockpit rendering must run inside X-Plane's graphics context and
  frame budget, which rules out a JS/TypeScript engine for that target.
- A 60 fps vector display wants GPU-accelerated drawing (NanoVG over
  GL/Vulkan/Metal).
- ~90% of the work (G1000 behavior + gauge drawing) is platform-independent, so
  it lives once in `avionics-core`; the shells only differ in **where data comes
  from** and **where pixels go**.

```
avionics-core/      C++ static lib: logic + rendering, zero platform deps
  include/avionics/ public headers (DataSource, Renderer, AvionicsEngine, ...)
  src/              engine + sample PrimaryFlightDisplay + MockDataSource
render-nanovg/      shared NanoVG Renderer backend used by both shells
shell-xplane/       XPLM plugin: DatarefDataSource + sim draw callback
shell-standalone/   GLFW window + 60 fps loop; mock or live X-Plane (UDP RREF)
```

The two seams that vary per platform are the abstract interfaces
`avionics::DataSource` (data in) and `avionics::Renderer` (pixels out).

| Concern        | X-Plane shell                          | Standalone shell                                   |
| -------------- | -------------------------------------- | -------------------------------------------------- |
| Frame loop     | Sim calls our draw callback            | We own a 60 fps loop                               |
| Data source    | `XPLMGetDataf` datarefs (in-process)   | X-Plane over UDP (RREF) or built-in mock feed       |
| Renderer       | NanoVG over the sim's GL/Vulkan/Metal  | NanoVG over our GLFW/SDL window                     |

## Build

Default build is the core + standalone shell. GLFW and NanoVG are fetched and
built automatically at configure time (no system packages or external SDKs
needed):

```bash
cmake -S . -B build
cmake --build build
./build/shell-standalone/avionics-standalone   # opens a window rendering the PFD (Esc to quit)
```

By default the standalone shell runs on the built-in mock feed. To connect to a
running X-Plane instance over its UDP data interface, or to switch feeds at
runtime:

```bash
# Start on the live X-Plane connection (default host 127.0.0.1, port 49000):
./build/shell-standalone/avionics-standalone --source xplane
./build/shell-standalone/avionics-standalone --source xplane --xplane-host 192.168.1.50 --xplane-port 49000

# Press M at any time to toggle between MOCK DATA and X-PLANE.
```

The display shows a power-on **boot screen** for a few seconds, then the live
PFD. If no data is arriving (e.g. X-Plane isn't running), it shows a large red
**X** — the same failure annunciation real glass cockpits use when a display
loses its data source.

The connection is written against a generic `SimulatorConnection` interface, so
a future Microsoft Flight Simulator (SimConnect) backend can drop in behind the
same seam without touching the engine or rendering code.

### X-Plane plugin

Requires the [X-Plane SDK](https://developer.x-plane.com/sdk/):

```bash
cmake -S . -B build -DBUILD_XPLANE_SHELL=ON -DXPLANE_SDK_DIR=/path/to/X-Plane-SDK
cmake --build build
```

This produces `build/shell-xplane/xplane-avionics.xpl`. Install it into X-Plane
under `Resources/plugins/xplane-avionics/<platform>/` (e.g. `mac.xpl` on macOS).

#### Live flight-plan bridge

X-Plane exposes the active **FMS flight plan only through the plugin SDK** — it
is not on the UDP RREF telemetry stream or the Web API — so the standalone shell
cannot read an in-cockpit route on its own. The plugin therefore runs a small
**flight-plan bridge**: it reads the live FMS on the sim thread and serves the
route over UDP (default port **49100**) to the standalone shell, which requests
it on a background thread and draws it on the map and FPL page automatically.

Install the plugin (above) on the machine running X-Plane and the standalone
shell picks up the live route with no manual save. Override the port with
`--fms-bridge-port` if 49100 is in use. When the plugin isn't installed (or
X-Plane is on another host without it), the shell falls back to a loaded/exported
`.fms` file (`--fms-plan`) or a SimBrief OFP, exactly as before.

The bridge is **bidirectional**: edits made in the shell are programmed back into
X-Plane's FMS over the same channel. Building or editing a plan on the FPL page,
loading a SimBrief OFP, and activating Direct-To all push to the real FMS (route
waypoints resolve to database navaids where possible, otherwise lat/lon, and
Direct-To uses X-Plane's present-position direct leg). The shell sends each edit
with an acknowledgement + retry so a dropped UDP packet doesn't lose the write.
Pass `--no-fms-write` to keep edits display-only while still reading the live
route.

## Per-aircraft checklists & engine display (EIS)

Neither the MFD **Checklist** page group nor the **EIS** engine strip is
hardcoded: both are plain-text files that an aircraft author ships with the
airframe, so the displayed checklists and the engine gauges change with the
aircraft without rebuilding the avionics. Both files are line-oriented, ignore
blank lines and `#` comments, and **hot-reload** — edit the file while the
display is running and it re-parses on the next frame (the store watches the
file's modification time), so you can iterate without a restart.

The bundled samples double as the format reference:

| Concern   | Sample file                          | Parser / keywords                          |
| --------- | ------------------------------------ | ------------------------------------------ |
| Checklist | `shell-standalone/assets/checklists.txt` | `avionics-core/include/avionics/Checklist.h` |
| EIS       | `avionics-core/assets/eis/c172s.eis`     | `avionics-core/include/avionics/Eis.h`       |

### Checklists

A checklist file is a set of named **groups** (e.g. `NORMAL PROCEDURES`), each
holding named **checklists** (e.g. `BEFORE TAKEOFF`), each a list of **items**.
The MFD pages a flat sequence of every checklist across all groups, with the
owning group name shown in the page header.

```
# Lines beginning with # are comments.
GROUP NORMAL PROCEDURES

  CHECKLIST BEFORE STARTING ENGINE
    Preflight Inspection : COMPLETE        # "<description> : <response>"
    Fuel Selector : BOTH
    Avionics Switch : OFF
    A note with no response and no colon   # renders as a plain line
```

- `GROUP <name>` starts a new group.
- `CHECKLIST <name>` starts a new checklist in the current group (a default
  unnamed group is created if none has been declared yet).
- Any other non-empty line is an **item**: the description and the expected
  response are split on the first ` : ` (space-colon-space). A line with no
  ` : ` becomes an item with an empty response (a note).

**Selecting the file (standalone shell):** pass `--checklist PATH`; with no
flag the bundled sample (`shell-standalone/assets/checklists.txt`) is used so
the page is populated during development.

```bash
./build/shell-standalone/avionics-standalone --checklist /path/to/my_aircraft_checklists.txt
```

### Engine display (EIS strip)

The EIS file describes the engine strip drawn on the left edge of every MFD
page: a title, an ordered list of **sections**, each holding **gauges**, plus a
`BIND` block that maps each logical channel to a simulator dataref.

```
TITLE ENGINE                 # strip heading

GAUGE RPM_DIAL               # gauge types: RPM_DIAL, BAR, READOUT, ELECTRICAL
  CHANNEL eng.rpm            # logical channel id (see below)
  MIN 0 MAX 3000
  REDLINE 2700
  BAND GREEN 2100 2500       # BAND <GREEN|YELLOW|RED> <lo> <hi>
  BAND RED 2700 3000

SECTION FUEL QTY GAL         # SECTION [title] starts a new group of gauges
GAUGE BAR
  LABEL L                    # text shown next to the gauge
  CHANNEL fuel.qty_left
  MIN 0 MAX 24
  BAND RED 0 1.5
  BAND GREEN 1.5 24

GAUGE READOUT                # numeric readout
  LABEL ENG HRS
  CHANNEL eng.hours
  FORMAT %.1f                # printf-style format for the value

GAUGE ELECTRICAL            # paired left/right readout (e.g. main/ess bus)
  LABEL BUS VOLTS
  LEFT M elec.bus_main       # LEFT  <tag> <channel>
  RIGHT E elec.bus_ess       # RIGHT <tag> <channel>
  FORMAT %.1f

# BIND <channel> <dataref> <scale> <offset>: display = raw * scale + offset.
BIND eng.rpm sim/cockpit2/engine/indicators/engine_speed_rpm[0] 1 0
BIND eng.oil_temp sim/cockpit2/engine/indicators/oil_temperature_deg_C[0] 1.8 32
```

Gauge keys: `LABEL`, `CHANNEL`, `MIN [MAX <n>]`, `MAX`, `REDLINE`, `BAND`,
`FORMAT`, and `LEFT`/`RIGHT` (for `ELECTRICAL`). The **channel** ids are the
logical names the gauges and `BIND` lines agree on (the canonical set lives in
`eis_channels` in `Eis.h`: `eng.rpm`, `eng.fuel_flow`, `eng.oil_pres`,
`eng.oil_temp`, `eng.egt`, `eng.vacuum`, `fuel.qty_left`, `fuel.qty_right`,
`eng.hours`, `elec.bus_main`, `elec.bus_ess`, `elec.batt_main`,
`elec.batt_standby`). Each `BIND` line converts a sim dataref's native units to
the gauge's display units with a `scale` and `offset` (e.g. `1.8`/`32` for
°C → °F).

**Where the EIS file is loaded from** (first match wins):

1. An explicit selector — `--eis PATH` on the standalone shell.
2. `g1000_eis.txt` next to the loaded `.acf` (in-sim plugin only).
3. `<acf_stem>_eis.txt` next to the loaded `.acf` (in-sim plugin only).
4. The build-time bundled default (`avionics-core/assets/eis/c172s.eis`).

The **in-sim X-Plane plugin** resolves the per-aircraft file automatically from
`sim/aircraft/view/acf_relative_path`, so dropping a `g1000_eis.txt` (or
`<acfname>_eis.txt`) beside the aircraft's `.acf` is enough — the strip switches
when you change aircraft. The **standalone shell over UDP** has no aircraft path
to key off, so point it at the file explicitly:

```bash
./build/shell-standalone/avionics-standalone --eis /path/to/MyAircraft/g1000_eis.txt
```

## Status / next steps

The shared core renders a basic PFD (attitude indicator,
airspeed/altitude/heading readouts) through the abstract renderer, and the
standalone shell now shows it live in a GLFW window via the NanoVG backend,
driven by either `MockDataSource` or a live `XPlaneConnection`.

Still to wire up:

- [x] NanoVG renderer backend implementing `avionics::Renderer` (shared by both
      shells, bound to each shell's GPU context).
- [x] GLFW window + real 60 fps loop in `shell-standalone`.
- [x] Live X-Plane data source for the standalone shell (`XPlaneConnection` over
      the UDP RREF protocol) behind a generic `SimulatorConnection` interface,
      with boot + connection-lost (red X) screens and a mock/live toggle.
- [x] Per-instrument reversionary red X. AHRS failure X's the attitude window
      ("AHRS") and HSI compass ("HDG"); air data computer failure X's the
      airspeed, altitude, and vertical-speed tapes. Driven live from X-Plane's
      `sim/operation/failures/rel_ss_*` datarefs.
- [x] Softkey bar (`SoftkeyController`) driven by a row of twelve physical
      softkey selection keys on the bezel below the screen, like the real GDU
      bezel: the on-screen bar is labels only (white on black, selected keys
      black-on-gray per the Pilot's Guide), and all interaction goes through
      the bezel keys. Pressing "Alerts" toggles the Alerts/messages window,
      which lists active alerts (color-coded by severity) derived from the
      sensor-validity flags.
- [x] Softkey menu state machine: the bar is a menu stack, so the root menu
      opens submenus (Map/HSI, PFD Opt, XPDR), each with a `Back` key that pops
      back up. Display-option keys are toggles that stay highlighted while on,
      and the XPDR submenu is a radio group (STBY/ON/ALT/GND).
- [x] MFD page groups (MAP / WPT / AUX / NRST / FPL) with per-group page
      memory, modeled on the G1000 Pilot's Guide for Cessna Nav III. The
      on-screen dual FMS knob works like the real one (large knob selects the
      page group, small knob steps pages within it), the group softkeys also
      select groups (pressing the active group's key again steps to its next
      page), and the FPL bezel key
      toggles the Active Flight Plan page. Pages: Navigation Map; Airport /
      Intersection / NDB / VOR Information; Trip Planning, GPS Status,
      System Status; Nearest Airports / Intersections / NDB / VOR /
      Airspaces; Active Flight Plan. A page group/page indicator box sits
      above the softkeys, and navaid frequencies are parsed from
      `earth_nav.dat` for the information pages.
- [x] EIS engine strip on the left edge of every MFD page (Cessna 172S fit):
      RPM dial with green/red arcs, FFLOW / OIL PRES / OIL TEMP / EGT / VAC
      bar indicators, per-tank FUEL QTY, ENG HRS, and the BUS VOLTS / BATT
      AMPS electrical rows — driven live from X-Plane's engine, fuel, and
      electrical datarefs (animated values on the mock feed).
- [x] PFD pop-up windows in the lower right, one at a time like the real unit:
      Timer/References (`Tmr/Ref`) with the generic timer (Start?/Stop?/Reset?
      via ENT, shown as a `TMR` field in the bottom info bar while running),
      V-speed reference bug On/Off toggles honored by the airspeed tape, and
      barometric minimums; Nearest Airports (`Nearest`) listing distance-sorted
      airports with bearing/distance, COM frequency, and longest runway,
      scrolled with the FMS knob. The large FMS knob moves the References
      cursor, the small knob steps the MINS altitude, ENT activates fields,
      and CLR closes the window, per the Pilot's Guide.
- [x] Altimeter alerting per the NXi Pilot's Guide: barometric minimums (BARO
      MIN box at the bottom left of the altimeter plus a tape bug, staging
      cyan -> white within 100 ft -> amber at minimums) and Selected Altitude
      alerting (the readout flashes black-on-cyan within 1000 ft, cyan within
      200 ft, and amber on a post-capture deviation).
- [x] Transponder functions: `Ident` annunciates a green IDNT in the
      transponder box for 18 seconds (inoperative in Standby, reverts the XPDR
      softkeys to the top level), and the XPDR > Code digit keys show the
      in-progress squawk entry in the data box with BKSP support. Committing
      the completed code to the radio still needs a sim command channel, like
      the VFR and STD Baro keys.
- [x] SimBrief integration (standalone shell): the MFD AUX – SIMBRIEF page
      fetches the account's latest OFP over HTTPS (SimBrief's public fetcher,
      JSON v2) and loads its geocoded navlog as the active flight plan on both
      feeds. The numeric Pilot ID is typed on XPDR-style digit softkeys (`ID`
      key; ENT commits) and persists across runs; `FETCH` re-downloads on
      demand, and a saved ID auto-fetches at startup (`--simbrief-id` to
      override). The page shows fetch status plus the OFP's origin/destination,
      generation time, and filed route. Needs libcurl (bundled with macOS) and
      nlohmann/json (fetched at configure time).
- [x] Flight plan editing on the FPL – Active Flight Plan page with the dual
      FMS knob, following the Pilot's Guide procedures: push the knob for the
      selection cursor, the large knob highlights a leg, and turning the small
      knob opens the Waypoint Information window where identifiers are spelled
      character by character (small knob selects the character starting at K,
      large knob moves the cursor) with database spell-ahead auto-fill; ENT
      inserts the waypoint ahead of the highlighted row (or appends on the
      blank slot, building a plan from scratch). CLR on a leg opens the
      `REMOVE <wpt>?` OK/CANCEL confirmation, and MENU offers Delete Flight
      Plan. Idents resolve against the parsed nav database (airports, VORs,
      NDBs, fixes; nearest wins on duplicates), and edits become the active
      plan on both feeds — the mock keeps flying toward the same waypoint, and
      on the X-Plane feed the edit is programmed into the sim's FMS through the
      flight-plan bridge (display-only override under `--no-fms-write`).
- [x] GPS Direct-To: the Direct-To bezel key opens the Direct To window over
      any MFD page, pre-filled with the active waypoint (or the highlighted
      flight-plan leg). The FMS knob spells the destination with the same
      database spell-ahead entry; the first ENT confirms the waypoint and arms
      the `ACTIVATE?` prompt, the second ENT engages the direct course. The map
      then draws the magenta direct-to leg from the aircraft straight to the
      waypoint (over the white flight plan); the mock flies it and sequences
      back onto the route on arrival; on the X-Plane feed the flight-plan bridge
      engages a present-position Direct-To in the sim's own FMS (or shows the
      course only under `--no-fms-write`).
- [x] Live FMS flight-plan bridge: the in-sim plugin reads the active FMS route
      (reachable only through the plugin SDK, not the UDP/Web API transports)
      and serves it over UDP to the standalone shell, which draws the real
      in-cockpit route on the map and FPL page automatically. Falls back to a
      `.fms` file or SimBrief OFP when the plugin isn't installed.
- [x] FMS write-back over the same bridge: FPL-page edits, SimBrief OFP loads,
      and Direct-To activations are programmed into X-Plane's FMS (navaid or
      lat/lon entries, present-position Direct-To), applied on the sim thread
      with an acknowledgement + retry. `--no-fms-write` keeps edits
      display-only.
- [ ] Bind the NanoVG renderer inside the plugin via the X-Plane Avionics
      Device API (`XPLMCreateAvionicsEx`), replacing the legacy draw callback.

### G1000 reference

Replicate **behavior and layout** from public Garmin documentation (Pilot's
Guide / Cockpit Reference Guide) and the free G1000 PC Trainer. Do **not** copy
Garmin's bitmaps, fonts, or any decompiled code — reimplement the look with
vector drawing.

## License

This project is licensed under the **GNU General Public License v3.0** — see
the [LICENSE](LICENSE) file for the full text.

> Copyright (C) 2026 Andrew Miller
>
> This program is free software: you can redistribute it and/or modify it under
> the terms of the GNU General Public License as published by the Free Software
> Foundation, either version 3 of the License, or (at your option) any later
> version. This program is distributed in the hope that it will be useful, but
> WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
> FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more
> details.

"Garmin" and "G1000" are trademarks of Garmin Ltd. This is an independent,
unofficial reimplementation and is not affiliated with or endorsed by Garmin.
