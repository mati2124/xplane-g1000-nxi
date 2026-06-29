# Flight-plan and navigation editing

This guide is for contributors changing active flight plans, procedure loading,
SimBrief import, Direct-To, VNAV, or the standalone/plugin FMS bridge. It
documents the ownership seams; user-facing setup stays in the top-level
[`README.md`](../../README.md).

## Subsystem map

| Concern | Start here | Notes |
| ------- | ---------- | ----- |
| Route edit helpers and FPL row math | [`FplRouteEdit.h`](../../avionics-core/include/avionics/FplRouteEdit.h) | Shared by the PFD Active Flight Plan window and MFD FPL page. |
| Persisted active plan, Direct-To, procedure blocks | [`FlightPlanPersistence.h`](../../avionics-core/include/avionics/FlightPlanPersistence.h) | Settings serialization, approach/SID/STAR grouping, metadata restore. |
| Stored flight plans | [`FlightPlanCatalog.h`](../../avionics-core/include/avionics/FlightPlanCatalog.h) | 99-slot catalog; SimBrief imports land here in the standalone shell. |
| Procedure loading and menus | [`ProcedureSupport.h`](../../avionics-core/include/avionics/ProcedureSupport.h), [`ProcedureMenu.h`](../../avionics-core/include/avionics/ProcedureMenu.h) | CIFP expansion, PROC page/menu state, terminal procedure metadata. |
| SimBrief UI state and import metadata | [`SimBrief.h`](../../avionics-core/include/avionics/SimBrief.h), [`SimBriefOfpSupport.h`](../../avionics-core/include/avionics/SimBriefOfpSupport.h) | Core structs consumed by shell-owned Navigraph/SimBrief clients. |
| VNAV and glidepath | [`VnavGuidance.h`](../../avionics-core/include/avionics/VnavGuidance.h), [`GlidepathGuidance.h`](../../avionics-core/include/avionics/GlidepathGuidance.h) | Computes vertical path from `MapLeg` altitude constraints; approach glidepath can suppress VNAV. |
| Lateral sequencing | [`FmsNavigator.h`](../../avionics-core/include/avionics/FmsNavigator.h) | Active leg, Direct-To, holds, missed approach. |
| Standalone FMS bridge client | [`FlightPlanBridgeClient.h`](../../shell-standalone/src/FlightPlanBridgeClient.h) | Background UDP polling and write ack/retry. |
| X-Plane FMS bridge server | [`FlightPlanBridge.h`](../../shell-xplane/src/FlightPlanBridge.h), [`FmsRouteProgrammer.cpp`](../../shell-xplane/src/FmsRouteProgrammer.cpp) | SDK reads/writes must run on the sim thread. |
| Wire protocol | [`FlightPlanBridgeProtocol.h`](../../avionics-core/include/avionics/FlightPlanBridgeProtocol.h) | Shared message tags, version, serialization caps. Treat as normative. |

## Active route ownership

The core controllers own the editable route state shown on the GDUs:

- PFD: `SoftkeyController` and `SoftkeyFlightPlan.cpp`
- MFD: `MfdController`, `MfdControllerFlightPlan.cpp`, and
  `MfdFlightPlanPage.cpp`

`AvionicsEngine::syncFlightPlanPeer()` keeps the PFD and MFD editors aligned.
When adding edit states, update both controllers and preserve the peer-sync rule:
the GDU that moved away from the last shared route wins, while a GDU with a
waypoint-entry or confirmation window may temporarily reject adoption.

Shells publish completed edits to their data source by draining
`consumeFlightPlanEdit()`:

```text
PFD/MFD editor
  -> consumeFlightPlanEdit()
  -> shell ApplyConsumedFlightPlan(...)
  -> XPlaneConnection::setRouteOverride(...) or setLocalFlightPlan(...)
  -> MapData.flightPlan, and optionally FlightPlanBridgeClient::writePlan(...)
```

`flightPlanReadyForSimulator()` is the gate between draft display state and a
route that is complete enough to program into the simulator FMS. Incomplete
plans use `setLocalFlightPlan()`: they draw on the map and FPL pages but are not
written to X-Plane. Completed edits use `setRouteOverride()`, which becomes
authoritative even for an empty route so a deleted plan does not fall back to an
older `.fms` file.

In the standalone shell, `XPlaneConnection` chooses a displayed route in this
order:

1. `routeOverride_` from a local edit, activated catalog plan, or explicit
   external route.
2. The live X-Plane FMS route reported by the in-sim bridge.
3. A loaded/exported `.fms` file (`--fms-plan`) when the bridge is unavailable.

## Standalone/plugin FMS bridge

The standalone app cannot read or program X-Plane's active FMS through UDP RREF
or the Web API. The in-sim plugin therefore exposes the FMS over a small UDP
bridge on port `49100` by default.

- [`FlightPlanBridgeProtocol.h`](../../avionics-core/include/avionics/FlightPlanBridgeProtocol.h)
  defines the complete protocol: `FPLQ` read requests, `FPLR` route replies,
  `FPLS` route writes, `FPLD` Direct-To writes, `FPLX` display-only Direct-To
  clear, `FPLG` active-leg writes, and `FPLA` acknowledgements.
- `kProtocolVersion` must be bumped whenever a payload layout changes. Receivers
  ignore mismatched versions, so an old plugin/new standalone pair fails closed
  instead of mis-parsing.
- Protocol v4 carries each leg's VNAV altitude constraint. Keep this intact when
  changing serialization; without those fields, X-Plane's FMS has no vertical
  restrictions to arm VNAV against.
- `FlightPlanBridge` separates SDK access from network I/O: a flight-loop
  callback on the sim thread reads/programs the FMS, while a background UDP
  server answers requests and queues writes.
- `FlightPlanBridgeClient` runs a background thread in the standalone shell. If
  the bridge is unreachable, reads report unavailable and writes are retried then
  dropped; the render loop keeps using fallback route sources.

`--no-fms-write` disables write-back from the standalone shell. Bridge reads can
still populate the displayed route, but local edits, Direct-To, and active-leg
changes stay display-only.

Direct-To has an extra constraint: while AP NAV is already coupled, reprogramming
X-Plane's FMS can briefly invalidate GPS and drop NAV. The standalone therefore
uses the `programFms` flag on `FPLD`, and `FPLX` can clear the plugin's stored
Direct-To display state without touching the sim FMS.

## SimBrief and Navigraph import

Navigraph OAuth and SimBrief OFP fetches are shell-owned network work. The core
only defines the UI state (`SimBriefState`) and route metadata helpers.

| Shell | Fetch result behavior | Token storage |
| ----- | --------------------- | ------------- |
| Standalone | Stores the OFP as a Flight Plan Catalog entry. The pilot must open FPL -> Flight Plan Catalog and Activate it before it becomes the active route or is written through the bridge. | App settings (`NavigraphStore` publishes refresh-token updates). |
| X-Plane plugin | Immediately replaces the active displayed route and data-source route override. | Per-user `navigraph_token.txt` helpers in the plugin config path. |

Standalone Navigraph communication is allowed only while the app has a live
X-Plane link (`SimBriefState::commAllowed`); the in-sim plugin is always in a
connected simulator session. Credentials are loaded from
`NAVIGRAPH_CLIENT_ID`/`NAVIGRAPH_CLIENT_SECRET` or the ignored credentials file
described in [`AGENTS.md`](../../AGENTS.md). Never commit credentials.

SimBrief navlog legs can be less structured than CIFP procedure legs. Import code
therefore:

1. Parses the OFP in shell code (`SimBriefClient`) into `SimBriefFetchResult`.
2. Uses `inferSimBriefProcedureBlocks()` to infer SID/STAR ranges from
   `via_airway` tags and the OFP procedure names.
3. Stores procedure metadata in `PersistedFlightPlan`.
4. Uses `restoreTerminalProcedureMetadata()` against the local nav database when
   possible, re-attaching CIFP roles, altitude constraints, holds, and corrected
   block spans.

When changing SimBrief import, test both the catalog-first standalone behavior
and the plugin's immediate-active behavior; they intentionally differ.

## Procedures, VNAV, and glidepath

Loaded procedures are represented as contiguous blocks in the active plan:
departure, arrival, and approach each carry start/count metadata plus a header
label. FPL list renderers use those blocks for parent rows, collapse/remove
behavior, and active-row cursor math. If you insert or remove legs, update the
affected block starts/counts through the helpers in `FplRouteEdit.h` and
`FlightPlanPersistence.h` rather than editing raw indices ad hoc.

VNAV is derived from `MapData.flightPlan` legs whose `MapLeg` fields carry:

- `altitudeConstraintFt`
- `altitudeConstraint`
- `altitudeDesignated`

`applyVnav()` computes the active vertical profile and publishes PFD/MFD fields
on `FlightData`. `suppressVnav()` yields to a valid approach glidepath; see
`GlidepathGuidance.h` for the capture window that decides when the approach
glidepath supersedes geometric VNAV.

## Test and troubleshooting runbook

For flight-plan work, the focused CPU tests are in `avionics-core/tests/`:

| Test file | Covers |
| --------- | ------ |
| `FlightPlanBridgeProtocolTest.cpp` | Bridge messages, Direct-To payloads, v4 altitude constraints. |
| `FlightPlanCatalogTest.cpp` | Catalog import/store/activate/invert and SimBrief re-import dedupe. |
| `FlightPlanPeerSyncTest.cpp` | PFD/MFD editor synchronization and cursor/altitude-column edge cases. |
| `FlightPlanPersistenceTest.cpp` | Saved plans, approach inference, terminal metadata restore. |
| `SimBriefParseTest.cpp` | Core SimBrief helper behavior. |
| `DepartureLoadTest.cpp`, `ArrivalLoadTest.cpp`, `AirwayLoadTest.cpp`, `ProcedureRemoveTest.cpp` | CIFP/procedure expansion, insertion, and removal. |
| `VnavGuidanceTest.cpp` | Constraint binding, TOD/VDI behavior, VPTH coupling. |

Build and run tests with GCC in this environment:

```bash
cmake -S . -B build -DCMAKE_C_COMPILER=gcc -DCMAKE_CXX_COMPILER=g++ \
  -DCMAKE_BUILD_TYPE=Debug -DBUILD_STANDALONE_SHELL=ON -DBUILD_TESTS=ON
cmake --build build -j"$(nproc)"
cd build && ctest --output-on-failure
```

Manual shell integration is still useful for bridge and Navigraph changes because
the UDP bridge, XPLM FMS programming, and OAuth/HTTP clients are not covered by
the core GoogleTest suite.

## Common pitfalls

- Keep plugin and standalone protocol changes in one commit; mismatched
  `kProtocolVersion` pairs ignore each other's datagrams.
- Do not auto-activate SimBrief imports in the standalone shell. Catalog-first
  behavior is intentional.
- Preserve VNAV altitude fields when copying, serializing, or bridge-sending
  `MapLeg`.
- Use `setLocalFlightPlan()` for incomplete pilot edits; only completed routes
  should program X-Plane's FMS.
- Treat an empty `setRouteOverride({})` as an authoritative deleted flight plan.
- Avoid FMS reprogramming during AP NAV-coupled Direct-To capture; use the
  existing Direct-To bridge flags/helpers.
- When fixing FPL display grouping, account for duplicate idents at procedure
  boundaries and for SimBrief imports that need CIFP metadata restoration.
