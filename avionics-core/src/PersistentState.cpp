#include "avionics/PersistentState.h"

#include <algorithm>
#include <cmath>
#include <cstring>

namespace avionics {
namespace {

// Per-aircraft V-speed lines: "av.vspeed.<ICAO>=on0,kt0,on1,kt1,on2,kt2,on3,kt3"
constexpr const char* kKeyVspeedPrefix = "av.vspeed.";

// Serialization keys. The "av." prefix keeps these distinct from any other
// keys a shell stores in the same preferences file.
constexpr const char* kKeyPfdMapLayout = "av.pfd.mapLayout";
constexpr const char* kKeyPfdMapDetail = "av.pfd.mapDetail";
constexpr const char* kKeyPfdWind = "av.pfd.wind";
constexpr const char* kKeyPfdToggles = "av.pfd.toggles";
constexpr const char* kKeyPfdInsetRange = "av.pfd.insetRange";
constexpr const char* kKeyPfdInsetRangeVer = "av.pfd.insetRangeVer";
constexpr const char* kKeyMfdTerrain = "av.mfd.terrain";
constexpr const char* kKeyMfdAirways = "av.mfd.airways";
constexpr const char* kKeyMfdTraffic = "av.mfd.traffic";
constexpr const char* kKeyMfdWeather = "av.mfd.weather";
constexpr const char* kKeyMfdDetail = "av.mfd.detail";
constexpr const char* kKeyMfdOrient = "av.mfd.orient";
constexpr const char* kKeyMfdRange = "av.mfd.range";
constexpr const char* kKeyMfdRangeVer = "av.mfd.rangeVer";

// Number of distinct values for each persisted enum, used to reject
// out-of-range values from a hand-edited or stale file before casting back.
constexpr int kMapLayoutCount = 3;     // Off, Inset, Hsi
constexpr int kWindOptionCount = 4;    // Off, Option1, Option2, Option3
constexpr int kMapDetailCount = 4;     // All, Detail3, Detail2, Detail1
constexpr int kTerrainCount = 3;       // Off, Topo, Rel
constexpr int kAirwayCount = 4;        // Off, All, Low, High
constexpr int kOrientationCount = 3;   // NorthUp, HeadingUp, TrackUp

bool parseInt(const std::string& value, int& out) {
  try {
    std::size_t consumed = 0;
    const int parsed = std::stoi(value, &consumed);
    if (consumed == 0) return false;
    out = parsed;
    return true;
  } catch (...) {
    return false;
  }
}

// Parse an enum stored as its underlying int, accepting it only when it falls
// inside [0, count); leaves `target` untouched otherwise (keeping the default).
template <typename Enum>
void parseEnum(const std::string& value, int count, Enum& target) {
  int raw = 0;
  if (parseInt(value, raw) && raw >= 0 && raw < count) {
    target = static_cast<Enum>(raw);
  }
}

void parseBool(const std::string& value, bool& target) {
  if (value == "1" || value == "true") {
    target = true;
  } else if (value == "0" || value == "false") {
    target = false;
  }
}

void parseRangeIndex(const std::string& value, int& target) {
  int raw = 0;
  if (parseInt(value, raw)) {
    target = std::max(0, std::min(kMapRangeLadderCount - 1, raw));
  }
}

int clampRangeIndex(int index) {
  return std::max(0, std::min(kMapRangeLadderCount - 1, index));
}

// Pack/unpack the display-toggle array as a bitmask (kDisplayToggleCount bits).
int togglesToMask(const std::array<bool, kDisplayToggleCount>& toggles) {
  int mask = 0;
  for (int i = 0; i < kDisplayToggleCount; ++i) {
    if (toggles[i]) mask |= (1 << i);
  }
  return mask;
}

void maskToToggles(int mask, std::array<bool, kDisplayToggleCount>& toggles) {
  for (int i = 0; i < kDisplayToggleCount; ++i) {
    toggles[i] = (mask & (1 << i)) != 0;
  }
}

// Parse "on0,kt0,on1,kt1,..." (kVspeedRefCount pairs) into a VspeedProfile.
// Returns false (leaving `out` untouched) on any malformed/short field so a
// stale or hand-edited line cannot corrupt the bugs.
bool parseVspeedCsv(const std::string& value, VspeedProfile& out) {
  VspeedProfile parsed;
  std::size_t pos = 0;
  for (int i = 0; i < kVspeedRefCount; ++i) {
    const std::size_t onEnd = value.find(',', pos);
    if (onEnd == std::string::npos) return false;
    const std::string onStr = value.substr(pos, onEnd - pos);
    pos = onEnd + 1;
    // The value is the last field of the pair; the next pair (if any) follows
    // after a comma.
    std::size_t ktEnd = value.find(',', pos);
    const std::string ktStr = value.substr(
        pos, ktEnd == std::string::npos ? std::string::npos : ktEnd - pos);
    if (ktEnd != std::string::npos) pos = ktEnd + 1;

    int ktRaw = 0;
    if (!parseInt(ktStr, ktRaw)) return false;
    parsed.on[i] = (onStr == "1" || onStr == "true");
    parsed.kt[i] = std::max(kVspeedMinKt,
                            std::min(kVspeedMaxKt, static_cast<float>(ktRaw)));
    if (ktEnd == std::string::npos && i + 1 < kVspeedRefCount) return false;
  }
  out = parsed;
  return true;
}

}  // namespace

bool operator==(const PfdPersistentState& a, const PfdPersistentState& b) {
  return a.mapLayout == b.mapLayout && a.mapDetail == b.mapDetail &&
         a.windOption == b.windOption && a.toggles == b.toggles &&
         a.insetRangeIndex == b.insetRangeIndex;
}

bool operator==(const MfdPersistentState& a, const MfdPersistentState& b) {
  return a.terrain == b.terrain && a.airways == b.airways &&
         a.showTraffic == b.showTraffic && a.showWeather == b.showWeather &&
         a.detail == b.detail &&
         a.orientation == b.orientation && a.rangeIndex == b.rangeIndex;
}

bool operator==(const VspeedProfile& a, const VspeedProfile& b) {
  if (a.on != b.on) return false;
  for (int i = 0; i < kVspeedRefCount; ++i) {
    if (std::lround(a.kt[i]) != std::lround(b.kt[i])) return false;
  }
  return true;
}

bool operator==(const AvionicsPersistentState& a,
                const AvionicsPersistentState& b) {
  return a.pfd == b.pfd && a.mfd == b.mfd &&
         a.vspeedByAircraft == b.vspeedByAircraft;
}

void capturePfdState(const SoftkeyController& controller,
                     PfdPersistentState& out) {
  out.mapLayout = controller.mapLayout_;
  out.mapDetail = controller.mapDetail_;
  out.windOption = controller.windOption_;
  out.toggles = controller.toggles_;
  out.insetRangeIndex = controller.insetRangeIndex_;
}

void captureMfdState(const MfdController& controller, MfdPersistentState& out) {
  out.terrain = controller.terrain_;
  out.airways = controller.airways_;
  out.showTraffic = controller.showTraffic_;
  out.showWeather = controller.showWeather_;
  out.detail = controller.detail_;
  out.orientation = controller.mapOrientation_;
  out.rangeIndex = controller.rangeIndex_;
}

void applyPfdState(SoftkeyController& controller, const PfdPersistentState& s) {
  controller.mapLayout_ = s.mapLayout;
  controller.mapDetail_ = s.mapDetail;
  controller.windOption_ = s.windOption;
  controller.toggles_ = s.toggles;
  controller.insetRangeIndex_ =
      migrateMapRangeIndex(clampRangeIndex(s.insetRangeIndex),
                           s.insetRangeSavedVersion);
  // Start the zoom animator at the restored range so it doesn't glide from the
  // power-on default on the first frame.
  controller.insetDisplayRangeNm_ = mapRangeNmAt(controller.insetRangeIndex_);
  controller.rebuildLabels();
}

void applyMfdState(MfdController& controller, const MfdPersistentState& s) {
  controller.terrain_ = s.terrain;
  controller.airways_ = s.airways;
  controller.showTraffic_ = s.showTraffic;
  controller.showWeather_ = s.showWeather;
  controller.detail_ = s.detail;
  controller.mapOrientation_ = s.orientation;
  controller.rangeIndex_ =
      migrateMapRangeIndex(clampRangeIndex(s.rangeIndex), s.rangeSavedVersion);
  // Start the zoom animator at the restored range so it doesn't glide from the
  // power-on default on the first frame.
  controller.displayRangeNm_ = mapRangeNmAt(controller.rangeIndex_);
  controller.rebuildLabels();
}

void captureVspeeds(const SoftkeyController& controller, VspeedProfile& out) {
  out.on = controller.vspeedOn_;
  out.kt = controller.vspeedKt_;
}

void applyVspeeds(SoftkeyController& controller, const VspeedProfile& s) {
  controller.vspeedOn_ = s.on;
  for (int i = 0; i < kVspeedRefCount; ++i) {
    controller.vspeedKt_[i] =
        std::max(kVspeedMinKt, std::min(kVspeedMaxKt, s.kt[i]));
  }
}

bool VspeedAircraftMemory::sync(SoftkeyController& controller,
                                const std::string& aircraftIcao,
                                std::map<std::string, VspeedProfile>& store) {
  // Until the aircraft type is known, leave the controller on its defaults.
  if (aircraftIcao.empty()) return false;

  if (!primed_ || aircraftIcao != lastIcao_) {
    // Aircraft (re)identified: restore that airframe's saved bugs, or the
    // built-in defaults when it has none stored yet.
    lastIcao_ = aircraftIcao;
    primed_ = true;
    const auto it = store.find(aircraftIcao);
    applyVspeeds(controller, it != store.end() ? it->second : VspeedProfile{});
    return false;
  }

  // Same aircraft: remember any edits the pilot has made since last frame.
  VspeedProfile current;
  captureVspeeds(controller, current);
  VspeedProfile& stored = store[aircraftIcao];
  if (stored != current) {
    stored = current;
    return true;
  }
  return false;
}

void appendStateLines(const AvionicsPersistentState& state, std::string& out) {
  const auto append = [&out](const char* key, int value) {
    out += key;
    out += '=';
    out += std::to_string(value);
    out += '\n';
  };
  append(kKeyPfdMapLayout, static_cast<int>(state.pfd.mapLayout));
  append(kKeyPfdMapDetail, static_cast<int>(state.pfd.mapDetail));
  append(kKeyPfdWind, static_cast<int>(state.pfd.windOption));
  append(kKeyPfdToggles, togglesToMask(state.pfd.toggles));
  append(kKeyPfdInsetRange, state.pfd.insetRangeIndex);
  append(kKeyPfdInsetRangeVer, kMapRangeLadderVersion);
  append(kKeyMfdTerrain, static_cast<int>(state.mfd.terrain));
  append(kKeyMfdAirways, static_cast<int>(state.mfd.airways));
  append(kKeyMfdTraffic, state.mfd.showTraffic ? 1 : 0);
  append(kKeyMfdWeather, state.mfd.showWeather ? 1 : 0);
  append(kKeyMfdDetail, static_cast<int>(state.mfd.detail));
  append(kKeyMfdOrient, static_cast<int>(state.mfd.orientation));
  append(kKeyMfdRange, state.mfd.rangeIndex);
  append(kKeyMfdRangeVer, kMapRangeLadderVersion);

  // One line per aircraft that has remembered V-speed bugs.
  for (const auto& [icao, prof] : state.vspeedByAircraft) {
    out += kKeyVspeedPrefix;
    out += icao;
    out += '=';
    for (int i = 0; i < kVspeedRefCount; ++i) {
      if (i != 0) out += ',';
      out += prof.on[i] ? '1' : '0';
      out += ',';
      out += std::to_string(static_cast<int>(std::lround(prof.kt[i])));
    }
    out += '\n';
  }
}

bool applyStateLine(const std::string& key, const std::string& value,
                    AvionicsPersistentState& state) {
  if (key == kKeyPfdMapLayout) {
    parseEnum(value, kMapLayoutCount, state.pfd.mapLayout);
  } else if (key == kKeyPfdMapDetail) {
    parseEnum(value, kMapDetailCount, state.pfd.mapDetail);
  } else if (key == kKeyPfdWind) {
    parseEnum(value, kWindOptionCount, state.pfd.windOption);
  } else if (key == kKeyPfdToggles) {
    int mask = 0;
    if (parseInt(value, mask)) maskToToggles(mask, state.pfd.toggles);
  } else if (key == kKeyPfdInsetRange) {
    parseRangeIndex(value, state.pfd.insetRangeIndex);
  } else if (key == kKeyPfdInsetRangeVer) {
    parseInt(value, state.pfd.insetRangeSavedVersion);
  } else if (key == kKeyMfdTerrain) {
    parseEnum(value, kTerrainCount, state.mfd.terrain);
  } else if (key == kKeyMfdAirways) {
    parseEnum(value, kAirwayCount, state.mfd.airways);
  } else if (key == kKeyMfdTraffic) {
    parseBool(value, state.mfd.showTraffic);
  } else if (key == kKeyMfdWeather) {
    parseBool(value, state.mfd.showWeather);
  } else if (key == kKeyMfdDetail) {
    parseEnum(value, kMapDetailCount, state.mfd.detail);
  } else if (key == kKeyMfdOrient) {
    parseEnum(value, kOrientationCount, state.mfd.orientation);
  } else if (key == kKeyMfdRange) {
    parseRangeIndex(value, state.mfd.rangeIndex);
  } else if (key == kKeyMfdRangeVer) {
    parseInt(value, state.mfd.rangeSavedVersion);
  } else if (key.rfind(kKeyVspeedPrefix, 0) == 0) {
    const std::string icao = key.substr(std::strlen(kKeyVspeedPrefix));
    if (!icao.empty()) {
      VspeedProfile prof;
      if (parseVspeedCsv(value, prof)) state.vspeedByAircraft[icao] = prof;
    }
  } else {
    return false;
  }
  return true;
}

}  // namespace avionics
