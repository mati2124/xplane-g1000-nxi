# X-Plane G1000 NXi

> 🚫 **FOR FLIGHT SIMULATOR USE ONLY.** Under no circumstances should this
> software be used for real-life aviation, navigation, or flight operations of
> any kind. It is **not** a certified avionics device, contains no airworthiness
> guarantees, and must never be relied upon in an actual aircraft. This is a
> hobby project intended solely for use with flight simulators.

> ⚠️ **Work in progress — not complete.** This project is currently more of a
> proof of concept than a finished product. By no means is it complete, expect
> bugs, missing features, and rough edges. It is, however, actively planned to be
> finished.

A high-performance glass-cockpit (G1000-style PFD/MFD) for X-Plane, built as
**one C++ engine with two shells** so the same avionics code runs:

1. **Natively inside X-Plane** — an XPLM plugin drawn in the sim's render pass.
2. **As a standalone desktop program** — its own window + 60 fps loop, fed by
   X-Plane over the network.

![Primary Flight Display with HSI map](docs/screenshots/pfd.png)

![Multi-Function Display — Navigation Map page](docs/screenshots/mfd.png)

Screenshots from the standalone shell (PFD with the HSI map enabled; MFD on the
Navigation Map page), captured with the offscreen `--screenshot` tool.

> **Yes, AI helped write this — but it wasn't a one-liner.** This codebase was
> developed with substantial assistance from AI coding tools, and I'm transparent
> about that. It is **not** as simple as telling an AI to "make a G1000 NXi" and
> walking away. I spent significant time and resources on it: Garmin Pilot's
> Guide research, architecture decisions, iterative debugging, visual comparison
> against the real NXi and PC Trainer, and steering the AI through thousands of
> small, precise changes until the behavior and layout were right. Judge the
> result on its merits.

## Architecture

One C++ engine (`avionics-core`) runs in two shells — an X-Plane plugin and a
standalone desktop app — wired through `DataSource` (sim data in) and `Renderer`
(pixels out). Native in-cockpit rendering and a 60 fps vector UI drove the
choice of C++ with a shared NanoVG backend.

**Contributors:** module layout, data flow, and "where do I change X?" pointers
live in **[ARCHITECTURE.md](ARCHITECTURE.md)** (not duplicated here).

## Build

Default build is the core + standalone shell. GLFW and NanoVG are fetched and
built automatically at configure time (no system packages or external SDKs
needed):

```bash
cmake -S . -B build
cmake --build build
./build/shell-standalone/avionics-standalone   # opens a window rendering the PFD (Esc to quit)
```

The standalone shell always connects to a running X-Plane over its UDP data
interface (default host `127.0.0.1`, port `49000`). Point it at another host or
port with:

```bash
./build/shell-standalone/avionics-standalone --xplane-host 192.168.1.50 --xplane-port 49000
```

The display shows a power-on **boot screen** for a few seconds (Garmin logo
splash, then the MFD power-up page with database review), then the live PFD.
Until X-Plane starts sending data (e.g. it isn't running yet), each instrument
shows a large red **X** — the same failure annunciation real glass cockpits use
when a display loses its data source. See [MFD power-up screen (boot)](#mfd-power-up-screen-boot)
for per-aircraft hero images.

#### Running on a separate PC (nav data)

Live instruments need only the network link, so the standalone can run on a
different PC from X-Plane: point `--xplane-host` at the sim machine (and install
the plugin there for the [flight-plan](#live-flight-plan-bridge) and
[command](#command-bridge-cockpit-keys--standalone) bridges).

The **moving map**, however, is drawn from navigation databases read off the
**local** filesystem — there is no nav-data streaming. By default the shell
finds them through X-Plane's own install list (`x-plane_install_12.txt`), so a
PC with no X-Plane install shows a blank map (instruments still work).

To run the full map without X-Plane on that PC, copy the nav-data tree from the
sim machine and point the shell at it with `--nav-data-dir`:

```bash
./build/shell-standalone/avionics-standalone \
  --xplane-host 192.168.1.50 \
  --nav-data-dir "/data/xplane-navdata"
```

The directory is treated like an X-Plane install root, so lay the copied files
out the same way (only the parts you want are needed):

```
<nav-data-dir>/
  Custom Data/earth_nav.dat earth_fix.dat earth_awy.dat earth_aptmeta.dat
  Custom Data/CIFP/<ICAO>.dat          # SID/STAR/approach procedures
  Custom Data/Airspaces/airspace.txt   # airspace boundaries
  Global Scenery/Global Airports/Earth nav data/apt.dat   # airport diagrams
  Global Scenery/.../Earth nav data/<+LAT-LON>.dsf         # terrain (optional)
```

`Resources/default data/` is accepted in place of `Custom Data/` for the nav,
fix, CIFP, and airspace files, matching a real install. Anything missing simply
degrades (e.g. no DSF tiles falls back to procedural terrain). The override is
tried first and then any real local install, so a partial copy can still fall
back to an installed X-Plane on the same PC.

The same folder can be set without the command line: the Windows installer's
**Display Setup** page has a *Navigation data folder (optional)* field, and the
app also reads a saved `navDataDir=` entry from its settings file (under
`%APPDATA%\XPlaneAvionics`, `~/Library/Application Support/XPlaneAvionics`, or
`~/.config/XPlaneAvionics`). The `--nav-data-dir` flag overrides the saved value.

#### Choosing monitors (full screen)

For a two-screen cockpit, run each display full screen on its own monitor with
`--pfd-monitor N` / `--mfd-monitor N` (0-based indices), or set them on the
Windows installer's **Display Setup** page. To see which physical screen each
index is:

```bash
./build/shell-standalone/avionics-standalone --list-monitors      # prints indices
./build/shell-standalone/avionics-standalone --identify-monitors   # flashes the
                                                                   # index on each
                                                                   # screen
```

`--identify-monitors` briefly shows each monitor's number large on that screen
(add `--time SECONDS` to change how long). The installer's Display Setup page
exposes the same thing as an **Identify monitors** button next to the monitor
pickers.

#### Standalone keyboard shortcuts

The standalone window has no menu bar; these keys control it (each toggle is
saved and restored on the next launch, and applies to both the PFD and MFD
windows):

| Key       | Action                                                              |
| --------- | ------------------------------------------------------------------ |
| `B`       | Show/hide the hardware **bezel** strips (keys + softkey row)        |
| `T`       | Show/hide the OS window **title bar** (close / minimize / maximize) |
| `P`       | Keep the windows **always on top** of other windows                |
| `Enter`   | Acknowledge the power-up page (same as the bezel **ENT** key)       |
| `Esc`     | Quit                                                                |

Window positions are always remembered between runs. The installers can also set
the standalone to start automatically when you sign in (Windows task, macOS
"Start at Login.command", Linux `--startup`).

#### Updates

On launch the standalone checks the GitHub Releases API in the background and,
when a newer version exists, prompts to update (skip the check with the
`AVIONICS_SKIP_UPDATE_CHECK` environment variable). On every platform it then
downloads that release's installer asset, **verifies it against the release's
`SHA256SUMS`**, applies it in place, and relaunches — no browser, no URL. If a
download or checksum check fails it falls back to opening the release page.

- **Windows**: runs `g1000nxi-setup-<ver>.exe` silently (per-user, so no admin
  prompt), updating only the standalone component and relaunching.
- **macOS**: mounts the downloaded `.dmg`, replaces the installed
  `G1000 NXi.app` bundle, clears its quarantine flag (the already-trusted
  running app authorizes the swap), and reopens it. The prompt is a native
  dialog.
- **Linux**: extracts the `.tar.gz` and refreshes the installed files under
  `~/.local/share/g1000-nxi`, then relaunches. The prompt uses `zenity` or
  `kdialog` when present; on a headless/minimal desktop it logs the notice
  instead.

Because these macOS/Linux builds are **unsigned**, macOS self-update relies on
clearing the quarantine flag rather than Developer ID signing + notarization —
adding signing later would make it fully Gatekeeper-clean. The in-sim plugin
only ever shows a notice — it is never updated automatically, since replacing it
while X-Plane is running is unsafe.

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

#### Binding the physical bezel keys

Every PFD/MFD bezel key, softkey, knob detent, and the RANGE/pan joystick is
bindable from X-Plane's **Settings → Keyboard** and **Joystick** screens, so you
can drive the glass from a keyboard, a HOTAS, or a hardware G1000 panel:

- **Stock G1000 commands** (`sim/GPS/g1000n1_*`, `sim/GPS/g1000n3_*`) are
  intercepted, so an aircraft or controller already bound to X-Plane's built-in
  G1000 keeps working with no changes.
- **Dedicated commands** are also created under the **`xplane_avionics/`**
  namespace (search "G1000 NXi" in the bindings list) so every key is bindable
  even in aircraft that don't expose the full stock command set:
  - `xplane_avionics/pfd/*` and `xplane_avionics/mfd/*` — `softkey1`…`softkey12`,
    `direct`, `menu`, `fpl`, `proc`, `clr`, `ent`, `cursor`, the FMS knob detents
    (`fms_outer_up/down`, `fms_inner_up/down`), `range_up/down`, and the map
    `pan_*` directions.
  - `xplane_avionics/radio/*` — COM/NAV select, flip-flop, and the inner/outer
    tuning detents.

Both routes drive the same on-screen avionics, so bind whichever your hardware
already sends. (Holding `clr` still triggers CLR → Default Map.)

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
`.fms` file (`--fms-plan`) and any locally active catalog/edit route.

The bridge is **bidirectional**: edits made in the shell are programmed back into
X-Plane's FMS over the same channel. Building or editing a completed plan on the
FPL page, activating a stored catalog plan, and activating Direct-To all push to
the real FMS (route waypoints resolve to database navaids where possible,
otherwise lat/lon, and Direct-To uses X-Plane's present-position direct leg). The
shell sends each edit with an acknowledgement + retry so a dropped UDP packet
doesn't lose the write. Pass `--no-fms-write` to keep edits display-only while
still reading the live route.

#### SimBrief OFP import

The AUX - SIMBRIEF page signs in through Navigraph's device-code flow and fetches
the latest SimBrief OFP for the signed-in account. The two shells intentionally
handle a successful fetch differently:

| Shell | Result |
| ----- | ------ |
| **Standalone** | The OFP is saved as a **Flight Plan Catalog** entry. Open **FPL → Flight Plan Catalog**, preview the row, and press **Activate** to make it the active route and write it to X-Plane through the bridge. The standalone must have a live X-Plane link before Login/FETCH are enabled. |
| **X-Plane plugin** | The OFP immediately replaces the active displayed route inside the sim. |

Opening the standalone Flight Plan Catalog page re-fetches the latest OFP when
signed in, so a newly generated dispatch can appear without restarting the app.

#### Command bridge (cockpit keys → standalone)

Physical G1000 bezel keys, softkeys, and COM/NAV knobs only fire X-Plane
commands inside the sim process. When you run the **standalone** windows on a
second monitor, the plugin also forwards those commands over UDP so the external
displays respond to cockpit hardware:

1. Install and enable the in-sim plugin (it listens for registration on UDP port
   **49102**).
2. Start the standalone shell as usual. It binds UDP port **49101** by default,
   registers with the plugin every few seconds, and applies forwarded events on
   the next frame.
3. Press a cockpit key (or a joystick binding to `sim/GPS/g1000n*` /
   `xplane_avionics/*`) — the standalone PFD/MFD receives the same softkey or
   bezel press.

Override the listen port with `--command-bridge-port` if 49101 is in use. The
plugin intercepts GDU commands whenever it is enabled, even when the in-sim
glass takeover is turned off in the Plugins menu — so you can run **standalone
only** for the displays and still drive them from the cockpit.

#### SPAD.neXt + Stream Deck

Ready-made SPAD.neXt profiles for Elgato Stream Deck live under [`spad/`](spad/).
They send the same `xplane_avionics/*` commands as the keyboard/joystick bindings
above. See [`spad/README.md`](spad/README.md) for install steps.

## Per-aircraft assets (checklists, EIS, boot screen)

The MFD **power-up page**, the **Checklist** page group, and the **EIS** engine
strip are not hardcoded: checklists and EIS use plain-text files; the boot
screen's center **hero image** and airframe label are driven by PNG assets and
the detected aircraft profile. An aircraft author can ship files with the
airframe, **or any user can add their own for any aircraft with no rebuild and
no code change** by dropping ICAO-keyed files into the plugin's assets folder
(see the load-order lists below). Checklist and EIS files are line-oriented,
ignore blank lines and `#` comments, and **hot-reload** — edit the file while
the display is running and it re-parses on the next frame (the store watches the
file's modification time), so you can iterate without a restart. Boot hero
images are picked up on the next power-up cycle (or when you change aircraft).

The bundled samples double as the format reference:

| Concern   | Sample file                          | Parser / keywords                          |
| --------- | ------------------------------------ | ------------------------------------------ |
| Boot panel | `avionics-core/assets/boot/c172.png` | `resolveBootHeroAsset()` / `typeKeyedBootHeroAsset()` |
| Checklist | `shell-standalone/assets/checklists.txt` | `avionics-core/include/avionics/Checklist.h` |
| EIS       | `avionics-core/assets/eis/c172s.eis`     | `avionics-core/include/avionics/Eis.h`       |

### MFD power-up screen (boot)

After the centered Garmin logo splash on both GDUs, the MFD shows the NXi
**Power-up Page**: the shared G1000 NXi logo, a large airframe **hero image**
on the left, a database currency list on the right (with per-row icons), the
map/terrain disclaimer, and an **ENT** / right-softkey prompt to continue. The
**Navigation Data** row reflects the loaded nav database (cycle / expiry); other
rows show representative trainer-style values. Layout and chrome are shared
across aircraft — only the hero art and the airframe label row change per type.

**Shared asset** (same for every aircraft):

- `assets/boot/g1000_nxi_logo.png` — top-left G1000 NXi wordmark on the power-up
  page (and the logo splash uses vector drawing).

**Where the hero image is loaded from** (first match wins):

1. `g1000_boot.png` next to the loaded `.acf` (in-sim plugin only).
2. `<acf_stem>_boot.png` next to the loaded `.acf` (in-sim plugin only).
3. A **user-droppable, ICAO-keyed file** in the plugin's assets folder:
   `Resources/plugins/xplane-avionics/assets/boot/<icao>.png`, where `<icao>` is
   the aircraft's `acf_ICAO` type code lowercased (e.g. `c172`, `tbm9`). This
   adds a custom boot graphic for **any** aircraft with no rebuild and no code
   change.
4. The bundled profile default: `boot/<profile-id>.png` (e.g. `boot/c172.png` for
   the Cessna piston profile when ICAO is `C172`).

The airframe name on the first database row (e.g. `Cessna 172S`) comes from the
built-in profile for known types; otherwise it falls back to the ICAO string.

Capture the boot page for development or regression checks:

```bash
./build/shell-standalone/avionics-standalone \
  --screenshot boot.ppm --state boot --no-bezel
```

(`--state boot` renders the MFD power-up page at full opacity; `--state bootlogo`
and `--state bootfade` capture mid-animation frames. Output is PPM; convert to
PNG if needed.)

Reference captures from the Garmin PC Trainer and this project live under
`docs/screenshots/refs/` (e.g. `trainer-mfd-boot-172.png`).

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

**Where the checklist file is loaded from** (first match wins):

1. An explicit selector — `--checklist PATH` on the standalone shell.
2. `g1000_checklist.txt` next to the loaded `.acf` (in-sim plugin only).
3. `<acf_stem>_checklist.txt` next to the loaded `.acf` (in-sim plugin only).
4. A **user-droppable, ICAO-keyed file** in the plugin's assets folder:
   `Resources/plugins/xplane-avionics/assets/checklists/<icao>.checklist`, where
   `<icao>` is the aircraft's `acf_ICAO` type code lowercased (e.g. `tbm9`,
   `b738`). This adds support for **any** aircraft with no rebuild and no code
   change, and overrides the bundled default.
5. The bundled checklist for the detected aircraft profile (defaults to the
   Cessna piston set).

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
4. A **user-droppable, ICAO-keyed file** in the plugin's assets folder:
   `Resources/plugins/xplane-avionics/assets/eis/<icao>.eis`, where `<icao>` is
   the aircraft's `acf_ICAO` type code lowercased (e.g. `tbm9`, `b738`). This
   adds support for **any** aircraft with no rebuild and no code change, and
   overrides the bundled default.
5. The build-time bundled default (`avionics-core/assets/eis/c172s.eis`).

The **in-sim X-Plane plugin** resolves the per-aircraft file automatically from
`sim/aircraft/view/acf_relative_path` and `acf_ICAO`, so either dropping a
`g1000_eis.txt` (or `<acfname>_eis.txt`) beside the aircraft's `.acf`, or adding
an ICAO-keyed `assets/eis/<icao>.eis` in the plugin folder, is enough — the
strip switches when you change aircraft (and hot-reloads on edit). The
**standalone shell over UDP** has no aircraft path to key off, so point it at
the file explicitly:

```bash
./build/shell-standalone/avionics-standalone --eis /path/to/MyAircraft/g1000_eis.txt
```

## NEXRAD weather (real datalink radar)

The MFD **NEXRAD** map overlay (Map Opt → NEXRAD) shows **real-world** ground
weather radar, not a simulation of it. X-Plane exposes no datalink/NEXRAD
product through any of its APIs (the SDK only offers the *onboard* airborne
radar texture, which is empty on aircraft without a radar), so the overlay is
fed from live public weather-radar tiles
([RainViewer](https://www.rainviewer.com/api.html) composite reflectivity,
~10-minute frames) fetched over HTTPS on a background thread and resampled onto
the moving map around the aircraft.

Because it is **actual current weather keyed to the aircraft's latitude /
longitude**, it reflects what is really happening on the ground there right now
and **will not match the weather set in the simulator** (X-Plane's clouds,
storms, or a custom weather preset). Fly near real precipitation and it appears;
set a thunderstorm in a clear-sky region and the overlay still shows clear.

This works identically in both shells (the X-Plane plugin and the standalone),
needs an internet connection, and falls back to nothing when offline. The
dedicated MFD **Weather Radar** page is separate and still driven by the
airframe's onboard radar.

## Map obstacles (FAA DDOF)

The moving map draws **obstacle symbols** (towers, antennas, wind turbines) from
the FAA **Daily Digital Obstacle File** in CSV format — the same US-only
database the real G1000 NXi uses. Symbols appear at close range (Map Setup →
**Obstacle Data**, default 10 NM) and are colored white/yellow/red by proximity
to your altitude, matching the Pilot's Guide hazard bands.

The CSV is large (~90 MB) and updated daily, so it is **not** committed to git.
Fetch it once before building or packaging:

```bash
python3 tools/fetch_obstacles.py
```

That writes `shell-standalone/assets/obstacles.csv`. Both the standalone app and
the X-Plane plugin resolve it automatically from their bundled `assets/` folder
at runtime (`assets::resolve("obstacles.csv")`). Release builds run the fetch
step in CI so installers ship the file. When the asset is missing the obstacle
layer simply stays empty; everything else works normally.

To refresh after the FAA publishes a new cycle:

```bash
rm -rf tools/.dof_cache    # optional: force a re-download
python3 tools/fetch_obstacles.py
```

Then rebuild and redeploy (standalone: reinstall or copy the updated
`assets/obstacles.csv`; plugin: run `tools/install-xplane-plugin.sh` or copy
`obstacles.csv` into `Resources/plugins/xplane-avionics/assets/`).

Override the path explicitly on the standalone shell:

```bash
./build/shell-standalone/avionics-standalone --obstacles /path/to/DOF.CSV
```

## G1000 reference

Replicate **behavior and layout** from public Garmin documentation (Pilot's
Guide / Cockpit Reference Guide) and the free G1000 PC Trainer. Do **not** copy
Garmin's bitmaps, fonts, or any decompiled code — reimplement the look with
vector drawing.

## License

This project is **source available** under the
**[PolyForm Noncommercial License 1.0.0](LICENSE)** — not OSI-approved open
source.

- **Source code** is public. You may use, modify, and share it for
  **non-commercial** purposes.
- **Freeware aircraft** may bundle or integrate this avionics suite under those
  terms (include [LICENSE](LICENSE) and the `Required Notice` line).
- **Commercial use** — including payware aircraft or any for-profit
  distribution — requires a separate license. See
  [COMMERCIAL-LICENSE.md](COMMERCIAL-LICENSE.md).

Contributions are welcome; see [CONTRIBUTING.md](CONTRIBUTING.md).

> Copyright (C) 2026 Andrew Miller
>
> Required Notice: Copyright Andrew Miller
> (https://github.com/andywmm9-pixel/xplane-g1000-nxi)

"Garmin" and "G1000" are trademarks of Garmin Ltd. This is an independent,
unofficial reimplementation and is not affiliated with or endorsed by Garmin.

## Caravan EIS v3 test branch
Uses the Working Title-matching DejaVu Sans SemiBold face and revised real-aircraft gauge geometry.
