#include "avionics/FmsWaypointEntry.h"

#include <algorithm>
#include <cmath>
#include <cctype>
#include <vector>

namespace avionics {
namespace {

// The FMS data-entry character sequence: the alphabet then the digits, with the
// small knob starting "in the middle at K" on a blank placeholder (Pilot's
// Guide, "Using the FMS Knob to enter data").
constexpr char kEntryChars[] = "ABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789";
constexpr int kEntryCharCount = 36;

char stepEntryChar(char c, int step) {
  int idx = 0;
  for (int i = 0; i < kEntryCharCount; ++i) {
    if (kEntryChars[i] == c) {
      idx = i;
      break;
    }
  }
  return kEntryChars[((idx + step) % kEntryCharCount + kEntryCharCount) %
                     kEntryCharCount];
}

bool isValidEntryChar(char ch) {
  ch = static_cast<char>(std::toupper(static_cast<unsigned char>(ch)));
  for (int i = 0; i < kEntryCharCount; ++i) {
    if (kEntryChars[i] == ch) return true;
  }
  return false;
}

}  // namespace

void FmsWaypointEntry::open(const NavFeatureSource* nav, const MapData* map,
                            const std::string& initial) {
  active = true;
  chars = initial;
  pos = 0;
  autofill.clear();
  match = MapFeature{};
  hasMatch = false;
  notFound = false;
  selectAll = !initial.empty();
  if (!chars.empty()) {
    updateAutofill(nav, map);
    if (!selectAll) {
      pos = std::min(std::max(0, static_cast<int>(chars.size()) - 1),
                     kMaxChars - 1);
    }
  }
}

void FmsWaypointEntry::updateAutofill(const NavFeatureSource* nav,
                                      const MapData* map) {
  autofill.clear();
  hasMatch = false;

  if (chars.empty()) return;

  // Spell-ahead: the alphabetically-first database ident extending the typed
  // prefix. Without a nav database, fall back to the nearby map features so
  // entry still works on the demo feed.
  if (nav != nullptr && nav->ready()) {
    autofill = nav->firstIdentWithPrefix(chars);
  }
  if (autofill.empty() && map != nullptr) {
    for (const MapFeature& f : map->features) {
      if (f.id.compare(0, chars.size(), chars) != 0) continue;
      if (autofill.empty() || f.id < autofill) autofill = f.id;
    }
  }
  if (autofill.empty()) return;

  // Resolve the filled ident to a waypoint, nearest to ownship when the same
  // ident names several (a fix and a VOR, duplicates across regions).
  std::vector<MapFeature> candidates;
  if (nav != nullptr && nav->ready()) {
    candidates = nav->lookupIdent(autofill, 16);
  }
  if (candidates.empty() && map != nullptr) {
    for (const MapFeature& f : map->features) {
      if (f.id == autofill) candidates.push_back(f);
    }
  }
  if (candidates.empty()) {
    if (!hasMatch && nav != nullptr && nav->ready() && allowAirways &&
        nav->isAirwayName(chars)) {
      hasMatch = true;
      match = MapFeature{};
      match.id = chars;
    }
    return;
  }

  const MapFeature* best = &candidates.front();
  if (map != nullptr && map->positionValid) {
    constexpr double kDegToRad = 3.14159265358979323846 / 180.0;
    const double cosLat = std::max(0.05, std::cos(map->ownshipLat * kDegToRad));
    double bestSq = 0.0;
    bool first = true;
    for (const MapFeature& f : candidates) {
      const double dLat = f.lat - map->ownshipLat;
      const double dLon = (f.lon - map->ownshipLon) * cosLat;
      const double dSq = dLat * dLat + dLon * dLon;
      if (first || dSq < bestSq) {
        best = &f;
        bestSq = dSq;
        first = false;
      }
    }
  }
  match = *best;
  hasMatch = true;
}

void FmsWaypointEntry::clearSelectedField() {
  chars.clear();
  autofill.clear();
  match = MapFeature{};
  hasMatch = false;
  notFound = false;
  pos = 0;
  selectAll = false;
}

void FmsWaypointEntry::turnChar(const NavFeatureSource* nav, const MapData* map,
                                int step) {
  if (selectAll) clearSelectedField();
  const std::string shown = ident();
  const char base = pos < static_cast<int>(shown.size()) ? shown[pos] : '\0';
  // Only the first cell on a blank field starts "in the middle at K"; later
  // cells start at A (Pilot's Guide data-entry procedure).
  const char blankStart = (pos == 0 && chars.empty()) ? 'K' : 'A';
  const char next = base == '\0' ? stepEntryChar(blankStart, step)
                                   : stepEntryChar(base, step);
  if (static_cast<int>(chars.size()) <= pos) {
    chars.push_back(next);
  } else {
    chars[pos] = next;
  }
  notFound = false;
  updateAutofill(nav, map);
}

void FmsWaypointEntry::moveCursor(const NavFeatureSource* nav,
                                  const MapData* map, int step) {
  if (selectAll) clearSelectedField();
  if (step < 0) {
    pos = std::max(0, pos - 1);
    return;
  }
  if (pos + 1 >= kMaxChars) return;
  ++pos;
  // Moving right adopts the character under the cursor into the typed prefix
  // (stepping through the auto-filled ident, like the real unit).
  if (static_cast<int>(chars.size()) <= pos) {
    const std::string shown = ident();
    chars.push_back(pos < static_cast<int>(shown.size()) ? shown[pos] : 'A');
  }
  updateAutofill(nav, map);
}

void FmsWaypointEntry::typeChar(const NavFeatureSource* nav, const MapData* map,
                                char ch) {
  if (!active) return;
  if (selectAll) clearSelectedField();
  ch = static_cast<char>(std::toupper(static_cast<unsigned char>(ch)));
  if (!isValidEntryChar(ch) || pos >= kMaxChars) return;
  if (static_cast<int>(chars.size()) <= pos) {
    chars.push_back(ch);
  } else {
    chars[static_cast<std::size_t>(pos)] = ch;
    chars.resize(static_cast<std::size_t>(pos + 1));
  }
  notFound = false;
  if (pos + 1 < kMaxChars) ++pos;
  updateAutofill(nav, map);
}

void FmsWaypointEntry::backspaceChar(const NavFeatureSource* nav,
                                     const MapData* map) {
  if (!active) return;
  if (selectAll) {
    clearSelectedField();
    updateAutofill(nav, map);
    return;
  }
  if (chars.empty()) {
    pos = 0;
    notFound = false;
    updateAutofill(nav, map);
    return;
  }
  if (pos > static_cast<int>(chars.size())) {
    pos = static_cast<int>(chars.size());
  }
  if (pos > 0) {
    --pos;
    chars.resize(static_cast<std::size_t>(pos));
  } else {
    chars.clear();
  }
  notFound = false;
  updateAutofill(nav, map);
}

}  // namespace avionics
