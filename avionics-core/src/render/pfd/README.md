# PFD rendering (`render/pfd/`)

Drawing code for the **Primary Flight Display**: flight instruments, HSI, inset
map, and the chrome around them (top bar, info panel, popups, softkey bar).

Public entry point: [`PrimaryFlightDisplay::render`](../../../include/avionics/render/PrimaryFlightDisplay.h)
in [`PrimaryFlightDisplay.cpp`](../PrimaryFlightDisplay.cpp).

## Orchestration

`PrimaryFlightDisplay.cpp` paints layers in z-order (back to front):

1. Attitude indicator
2. Inset map (when enabled)
3. HSI map backdrop (rose-over-map layout)
4. Airspeed / altimeter / VSI tapes
5. Vertical deviation (glideslope / VNAV)
6. HSI rose, needles, bearing pointers, wind
7. Chrome (bars, popups, softkeys)

## Shared internals

| File | Purpose |
| ---- | ------- |
| [`PfdInternal.h`](PfdInternal.h) | `Layout` struct, `computeLayout`, tape/ADI/HSI `draw…` declarations, WT 1024×768 constants |
| [`PfdShared.cpp`](PfdShared.cpp) | Helpers shared across instruments (formatting, small primitives) |
| [`HsiInternal.h`](HsiInternal.h) | HSI-specific layout and rose/map-mode helpers |
| [`ChromeInternal.h`](ChromeInternal.h) | Chrome layout + `drawChrome` and per-window `draw…` declarations |
| [`Chrome.cpp`](Chrome.cpp) | Chrome orchestrator — calls each chrome sub-drawer in order |
| [`ChromeShared.cpp`](ChromeShared.cpp) | Shared chrome drawing helpers |

## Flight instruments

| File | Instrument |
| ---- | ---------- |
| [`AttitudeIndicator.cpp`](AttitudeIndicator.cpp) | ADI (pitch/roll, flight director cues) |
| [`AirspeedTape.cpp`](AirspeedTape.cpp) | Airspeed tape + trend / bugs |
| [`Altimeter.cpp`](Altimeter.cpp) | Altimeter tape + baro / minimums |
| [`VerticalSpeedIndicator.cpp`](VerticalSpeedIndicator.cpp) | VSI |
| [`VerticalDeviation.cpp`](VerticalDeviation.cpp) | Glideslope / VNAV deviation scale |
| [`InsetMap.cpp`](InsetMap.cpp) | Small circular map in classic HSI layout |

## HSI

| File | Concern |
| ---- | ------- |
| [`Hsi.cpp`](Hsi.cpp) | Rose vs map layout, heading bug, range, orchestration |
| [`HsiCourseNeedle.cpp`](HsiCourseNeedle.cpp) | Course needle and deviation |
| [`HsiBearingPointer.cpp`](HsiBearingPointer.cpp) | Bearing pointers (BRG1/BRG2) |
| [`HsiTurnRate.cpp`](HsiTurnRate.cpp) | Turn-rate trend vector |
| [`HsiCdiAnnunciation.cpp`](HsiCdiAnnunciation.cpp) | CDI source / mode annunciations |
| [`WindIndicator.cpp`](WindIndicator.cpp) | Wind direction/speed on the HSI |

HSI map mode reuses [`MapView`](../map/MapView.cpp) via `drawHsiMap` in
`Hsi.cpp`.

## Chrome (UI around the instruments)

| File | UI element |
| ---- | ---------- |
| [`ChromeTopBar.cpp`](ChromeTopBar.cpp) | NAV/COM frequency bar |
| [`ChromeInfoPanel.cpp`](ChromeInfoPanel.cpp) | Bottom data fields (wind, timers, OAT, …) |
| [`ChromeCasAnnunciations.cpp`](ChromeCasAnnunciations.cpp) | CAS alert window |
| [`ChromeAlertsWindow.cpp`](ChromeAlertsWindow.cpp) | Alerts popup |
| [`ChromeReferencesWindow.cpp`](ChromeReferencesWindow.cpp) | References popup |
| [`ChromeNearestWindow.cpp`](ChromeNearestWindow.cpp) | Nearest airports popup |
| [`ChromePfdSetupWindow.cpp`](ChromePfdSetupWindow.cpp) | PFD Setup menu |
| [`ChromeDirectToWindow.cpp`](ChromeDirectToWindow.cpp) | Direct-To window |
| [`ChromeSoftkeyBar.cpp`](ChromeSoftkeyBar.cpp) | On-screen softkey labels |

Popup windows share the lower-right region; only one is primary at a time, but
fade animations can overlap (see comments in `Chrome.cpp`).

## Conventions

- All `draw…` functions live in namespace `avionics::pfd`.
- Take a `Layout` from `computeLayout(width, height)` plus `FlightData`,
  `MapData` (for map/inset), and `SoftkeyController` (for UI state).
- Prefer adding a new `.cpp` over growing an orchestrator file when the concern
  is visually distinct (matches the HSI and Chrome splits).

See also: [Architecture overview](../../../../ARCHITECTURE.md),
[map folder](../map/README.md).
