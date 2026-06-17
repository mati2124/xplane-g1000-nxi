# Moving map rendering (`render/map/`)

Shared **navigation map** used by the PFD inset, PFD HSI-map layout, and MFD
Navigation Map / Traffic Map pages.

Public entry point: [`MapView::render`](../../../include/avionics/render/MapView.h)
in [`MapView.cpp`](MapView.cpp).

## Orchestration

`MapView::render` builds a projection context, then paints layers back to front:

1. Terrain / land shading (when available)
2. Airspace boundaries
3. Airways
4. Airport diagrams
5. Nav features (airports, navaids, intersections)
6. Route / flight plan line
7. Traffic
8. Weather raster (NEXRAD overlay)
9. Ownship, track vector, wind vector, fuel ring
10. Map chrome (range rings, orientation label, declutter)

Layer order and clipping are centralized in `MapView.cpp`; individual layers
live in their own translation units.

## Shared internals

| File | Purpose |
| ---- | ------- |
| [`MapViewInternal.h`](MapViewInternal.h) | `Proj` projection context, symbol sizes, shared `draw…` declarations, clip helpers |
| [`MapProjection.h`](MapProjection.h) / [`.cpp`](MapProjection.cpp) | Lat/lon ↔ screen projection, rotation (north / heading / track up) |
| [`MapPrimitives.cpp`](MapPrimitives.cpp) | Low-level map strokes, range rings, dashed lines |
| [`MapChrome.cpp`](MapChrome.cpp) | Range readout, orientation annunciation, map box chrome |
| [`MapSymbols.cpp`](MapSymbols.cpp) | Shared nav-feature symbology |

## Layers

| File | Layer |
| ---- | ----- |
| [`MapLandLayer.cpp`](MapLandLayer.cpp) | Land / coast shading from bundled `land_data.bin` |
| [`TerrainRaster.cpp`](TerrainRaster.cpp) | DSF terrain elevation tint (when shell provides tiles) |
| [`MapAirspaceLayer.cpp`](MapAirspaceLayer.cpp) | Controlled / special-use airspace |
| [`MapAirwayLayer.cpp`](MapAirwayLayer.cpp) | Victor/Jet airways |
| [`MapAirportDiagram.cpp`](MapAirportDiagram.cpp) | apt.dat runway/taxi geometry |
| [`MapNavFeatures.cpp`](MapNavFeatures.cpp) | Airports, VORs, NDBs, intersections |
| [`MapRoute.cpp`](MapRoute.cpp) | Active flight plan legs |
| [`MapTrafficLayer.cpp`](MapTrafficLayer.cpp) | TIS-B / sim traffic symbols |
| [`WeatherRaster.cpp`](WeatherRaster.cpp) | NEXRAD precipitation overlay |
| [`MapObstacleLayer.cpp`](MapObstacleLayer.cpp) | FAA DDOF obstacle symbols (bundled `assets/obstacles.csv`) |

## Ownship & vectors

| File | Element |
| ---- | ------- |
| [`MapOwnship.cpp`](MapOwnship.cpp) | Aircraft symbol |
| [`MapTrackVector.cpp`](MapTrackVector.cpp) | Track vector / heading line |
| [`MapWindVector.cpp`](MapWindVector.cpp) | Wind barb on map |
| [`MapFuelRing.cpp`](MapFuelRing.cpp) | Range ring from fuel endurance |

## Data sources

This folder only **draws** [`MapData`](../../../include/avionics/MapData.h).
Parsing and spatial queries live in:

- [`avionics-core/src/nav/`](../../nav/) — file format parsers
- Shell stores (`NavData`, `AptDatStore`, `AirspaceStore`, `ObstacleStore`, …)
  — populate `MapData` each frame. Obstacles come from the FAA DDOF CSV fetched
  by [`tools/fetch_obstacles.py`](../../../../tools/fetch_obstacles.py).

The map can pan off ownship when the MFD map pointer is active; shells push
pointer position into `DataSource` so feature queries stay centered on the
view.

## Conventions

- Namespace: `avionics::mapview` for internals; `MapView` is in `avionics`.
- Symbol sizes are in **768 px canvas units** (see `kFeatureSymbolWt` in
  `MapViewInternal.h`) so icons stay the same apparent size on PFD inset and
  full MFD map.
- Add a new visible layer as a `draw…Layer` in its own `.cpp` and call it from
  `MapView::render` at the appropriate z-index.

See also: [Architecture overview](../../../../ARCHITECTURE.md),
[PFD folder](../pfd/README.md) (inset / HSI map),
[MFD folder](../mfd/README.md) (full-screen map pages).
