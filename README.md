# XPlaneAvionics

A high-performance glass-cockpit (G1000-style PFD/MFD) for X-Plane, built as
**one C++ engine with two shells** so the same avionics code runs:

1. **Natively inside X-Plane** — an XPLM plugin drawn in the sim's render pass.
2. **As a standalone desktop program** — its own window + 60 fps loop, fed by
   X-Plane over the network.

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
- [x] Clickable softkey bar (`SoftkeyController`) with Working-Title-style
      animations: a key-press flash and an eased slide/fade for pop-up windows.
      Clicking "Alerts" toggles the Alerts/messages window, which lists active
      alerts (color-coded by severity) derived from the sensor-validity flags.
- [x] Softkey menu state machine: the bar is a menu stack, so the root menu
      opens submenus (Map/HSI, PFD Opt, XPDR), each with a `Back` key that pops
      back up. Display-option keys are toggles that stay highlighted while on,
      and the XPDR submenu is a radio group (STBY/ON/ALT/GND).
- [ ] Bind the NanoVG renderer inside the plugin via the X-Plane Avionics
      Device API (`XPLMCreateAvionicsEx`), replacing the legacy draw callback.
- [ ] MFD page groups (MAP / WPT / AUX / NRST), reusing the softkey state
      machine for page-group navigation.

### G1000 reference

Replicate **behavior and layout** from public Garmin documentation (Pilot's
Guide / Cockpit Reference Guide) and the free G1000 PC Trainer. Do **not** copy
Garmin's bitmaps, fonts, or any decompiled code — reimplement the look with
vector drawing.
