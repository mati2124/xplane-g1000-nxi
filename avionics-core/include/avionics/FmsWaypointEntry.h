#pragma once

#include <string>

#include "avionics/MapData.h"
#include "avionics/NavFeatureSource.h"

namespace avionics {

// Shared FMS identifier-entry state (Pilot's Guide, "Using the FMS Knob to
// enter data"): the small knob selects the character under the cursor, the
// large knob moves the cursor, and the database spell-ahead completes the typed
// prefix. Used by the MFD's FPL insert / Direct-To / Waypoint pages and the
// PFD's Direct-To window, so the entry behavior is identical on both displays.
class FmsWaypointEntry {
 public:
  // Longest identifier enterable (covers ICAO airports, navaids, fixes).
  static constexpr int kMaxChars = 6;

  bool active = false;
  std::string chars;     // typed characters, contiguous from cell 0
  int pos = 0;           // cell under the entry cursor
  std::string autofill;  // full database ident completing the prefix
  MapFeature match;
  bool hasMatch = false;
  bool notFound = false;

  // When true, a typed prefix that names a published airway (rather than a
  // waypoint) resolves to a match -- used by the FPL insert window, which can
  // load an airway. The Direct-To and Waypoint pages leave it false.
  bool allowAirways = false;

  // The displayed identifier: the spell-ahead completion when present, else the
  // typed prefix.
  std::string ident() const { return autofill.empty() ? chars : autofill; }
  int typedCount() const { return static_cast<int>(chars.size()); }

  // Reset to a fresh, inactive entry.
  void reset() {
    const bool keepAirways = allowAirways;
    *this = FmsWaypointEntry{};
    allowAirways = keepAirways;
  }

  // Open a fresh entry, optionally seeded with an initial identifier. nav may be
  // null (entry falls back to the nearby map features); map may be null.
  void open(const NavFeatureSource* nav, const MapData* map,
            const std::string& initial = "");
  // Small knob: step the character under the cursor (blank starts at K).
  void turnChar(const NavFeatureSource* nav, const MapData* map, int step);
  // Large knob: move the character cursor, adopting the auto-filled character
  // into the typed prefix when stepping right.
  void moveCursor(const NavFeatureSource* nav, const MapData* map, int step);

 private:
  // Recompute the spell-ahead auto-fill + matched waypoint for the typed
  // prefix, from the nav database (or the nearby map features without one).
  void updateAutofill(const NavFeatureSource* nav, const MapData* map);
};

}  // namespace avionics
