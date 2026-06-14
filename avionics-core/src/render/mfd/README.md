# MFD rendering (`render/mfd/`)

Drawing code for the **Multi-Function Display**: page content, EIS engine
strip, and MFD-specific chrome. Page **routing and softkey logic** live in
[`MfdController`](../../../include/avionics/MfdController.h) (split across
`MfdController*.cpp` in this folder).

Public entry point: [`MultiFunctionDisplay::render`](../../../include/avionics/render/MultiFunctionDisplay.h)
in [`MultiFunctionDisplay.cpp`](MultiFunctionDisplay.cpp).

## Orchestration

`MultiFunctionDisplay.cpp`:

1. Draws the shared top/bottom bars (matching PFD typography)
2. Renders the left **EIS strip** when an engine layout is present
3. Dispatches to the active page's `draw…Page` function (see [`MfdPages.h`](MfdPages.h))
4. Draws the MFD softkey bar and cursor highlight

## Page dispatch (`MfdPages.h`)

| File | Page group / screen |
| ---- | ------------------- |
| [`MfdMapPage.cpp`](MfdMapPage.cpp) | MAP — Navigation Map and Traffic Map (both use `MapView`) |
| [`MfdNearestPages.cpp`](MfdNearestPages.cpp) | NRST — nearest airports, intersections, … |
| [`MfdWaypointPages.cpp`](MfdWaypointPages.cpp) | WPT — airport / navaid information |
| [`MfdFlightPlanPage.cpp`](MfdFlightPlanPage.cpp) | FPL — active flight plan |
| [`MfdAuxPages.cpp`](MfdAuxPages.cpp) | AUX — trip planning, utility, GPS status, system setup |
| [`MfdChecklistPage.cpp`](MfdChecklistPage.cpp) | Checklist pages |
| [`MfdMapSettingsPage.cpp`](MfdMapSettingsPage.cpp) | Map setup overlays |
| [`MfdWeatherRadarPage.cpp`](MfdWeatherRadarPage.cpp) | MAP — airborne weather radar (GWX wedge) |

Traffic map (`drawTrafficMapPage`) is implemented in [`MfdMapPage.cpp`](MfdMapPage.cpp)
alongside the navigation map.

## Controller (state, not drawing)

| File | Responsibility |
| ---- | -------------- |
| [`MfdController.cpp`](MfdController.cpp) | Core page state, softkey handling |
| [`MfdControllerPageMenu.cpp`](MfdControllerPageMenu.cpp) | Page menu / group navigation |
| [`MfdControllerMapSettings.cpp`](MfdControllerMapSettings.cpp) | Map setup menus and toggles |
| [`MfdControllerProcedures.cpp`](MfdControllerProcedures.cpp) | Departure / arrival / approach UI |
| [`MfdControllerChecklist.cpp`](MfdControllerChecklist.cpp) | Checklist navigation |

## Shared MFD internals

| File | Purpose |
| ---- | ------- |
| [`MfdStyle.h`](MfdStyle.h) | Colors, font weights, shared style helpers |
| [`MfdPageSupport.h`](MfdPageSupport.h) / [`.cpp`](MfdPageSupport.cpp) | Reusable list rows, data fields, scroll regions |
| [`MfdMapSettings.h`](MfdMapSettings.h) | Map setup state shared between controller and pages |
| [`EisStrip.h`](EisStrip.h) / [`.cpp`](EisStrip.cpp) | Engine instrument strip (g1000_eis.txt layout) |

## EIS

Engine display layouts are parsed in [`EisStore`](../../../include/avionics/EisStore.h)
/ [`EisParser`](../../../src/EisParser.cpp) at the core level; this folder only
**renders** the strip from `EisLayout` + live `FlightData`.

## Adding a page

1. Add `MfdPage` enum value in [`MfdController.h`](../../../include/avionics/MfdController.h).
2. Declare `drawYourPage(…)` in [`MfdPages.h`](MfdPages.h).
3. Implement in `MfdYourPage.cpp`.
4. Wire softkeys in the relevant `MfdController*.cpp`.
5. Add `case` + title in [`MultiFunctionDisplay.cpp`](MultiFunctionDisplay.cpp).

## Conventions

- Namespace `avionics::mfd` for page helpers; top-level `MultiFunctionDisplay`
  stays in `avionics`.
- Reuse [`PfdInternal.h`](../pfd/PfdInternal.h) bar metrics so PFD and MFD
  chrome align.
- Keep controller state out of draw functions where possible — pages receive
  `const MfdController&` and read-only snapshots.

See also: [Architecture overview](../../../../ARCHITECTURE.md),
[map folder](../map/README.md).
