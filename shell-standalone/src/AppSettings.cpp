#include "AppSettings.h"

#include <cstdio>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <ostream>
#include <string>

#include "avionics/FlightPlanCatalog.h"

namespace avionics {
namespace {

constexpr const char* kAppDirName = "XPlaneAvionics";
constexpr const char* kSettingsFileName = "settings.txt";
constexpr const char* kKeyShowBezel = "showBezel";
constexpr const char* kKeyNavigraphRefreshToken = "navigraphRefreshToken";
constexpr const char* kKeyNavDataDir = "navDataDir";
constexpr const char* kKeyShowWindowChrome = "showWindowChrome";
constexpr const char* kKeyAlwaysOnTop = "alwaysOnTop";
constexpr const char* kKeyPfdFullscreen = "pfdFullscreen";
constexpr const char* kKeyMfdFullscreen = "mfdFullscreen";
constexpr const char* kKeyPfdMonitor = "pfdMonitor";
constexpr const char* kKeyMfdMonitor = "mfdMonitor";
constexpr const char* kKeyPfdWindowX = "pfdWindowX";
constexpr const char* kKeyPfdWindowY = "pfdWindowY";
constexpr const char* kKeyMfdWindowX = "mfdWindowX";
constexpr const char* kKeyMfdWindowY = "mfdWindowY";
constexpr const char* kKeyDebugDataSource = "debugDataSource";
constexpr const char* kKeyDemoMasterPower = "demoMasterPower";
constexpr const char* kKeyDemoAvionicsPower = "demoAvionicsPower";
constexpr const char* kKeyDemoCasMessages = "demoCasMessages";
constexpr const char* kKeyApproachActive = "approachActive";
constexpr const char* kKeyApproachAirport = "approachAirport";
constexpr const char* kKeyApproachType = "approachType";
constexpr const char* kKeyApproachName = "approachName";
constexpr const char* kKeyApproachTransition = "approachTransition";
constexpr const char* kKeyApproachRunway = "approachRunway";
constexpr const char* kKeyApproachKind = "approachKind";
constexpr const char* kKeyApproachLos = "approachLos";
constexpr const char* kKeyFplActive = "fplActive";
constexpr const char* kKeyFplDestFilled = "fplDestFilled";
constexpr const char* kKeyFplApproachStart = "fplApproachStart";
constexpr const char* kKeyFplApproachCount = "fplApproachCount";
constexpr const char* kKeyFplApproachAirport = "fplApproachAirport";
constexpr const char* kKeyFplDepStart = "fplDepStart";
constexpr const char* kKeyFplDepCount = "fplDepCount";
constexpr const char* kKeyFplDepActive = "fplDepActive";
constexpr const char* kKeyFplDepAirport = "fplDepAirport";
constexpr const char* kKeyFplDepType = "fplDepType";
constexpr const char* kKeyFplDepName = "fplDepName";
constexpr const char* kKeyFplDepTransition = "fplDepTransition";
constexpr const char* kKeyFplDepRunway = "fplDepRunway";
constexpr const char* kKeyFplArrStart = "fplArrStart";
constexpr const char* kKeyFplArrCount = "fplArrCount";
constexpr const char* kKeyFplArrActive = "fplArrActive";
constexpr const char* kKeyFplArrAirport = "fplArrAirport";
constexpr const char* kKeyFplArrType = "fplArrType";
constexpr const char* kKeyFplArrName = "fplArrName";
constexpr const char* kKeyFplArrTransition = "fplArrTransition";
constexpr const char* kKeyFplArrRunway = "fplArrRunway";
constexpr const char* kKeyDtoActive = "dtoActive";
constexpr const char* kKeyDtoTarget = "dtoTarget";
constexpr const char* kKeyDtoOriginLat = "dtoOriginLat";
constexpr const char* kKeyDtoOriginLon = "dtoOriginLon";
constexpr const char* kKeyDtoOriginValid = "dtoOriginValid";

// Stable text tokens for the persisted DebugDataSource selection.
constexpr const char* kSourceXPlane = "xplane";
constexpr const char* kSourceDemoGround = "demoGround";
constexpr const char* kSourceDemoFlying = "demoFlying";
constexpr const char* kSourceDemoTurbulence = "demoTurbulence";

const char* DebugDataSourceToken(DebugDataSource source) {
  switch (source) {
    case DebugDataSource::XPlane:
      return kSourceXPlane;
    case DebugDataSource::DemoGround:
      return kSourceDemoGround;
    case DebugDataSource::DemoFlying:
      return kSourceDemoFlying;
    case DebugDataSource::DemoTurbulence:
      return kSourceDemoTurbulence;
  }
  return kSourceXPlane;
}

DebugDataSource ParseDebugDataSource(const std::string& value,
                                     DebugDataSource fallback) {
  if (value == kSourceXPlane) return DebugDataSource::XPlane;
  if (value == kSourceDemoGround) return DebugDataSource::DemoGround;
  if (value == kSourceDemoFlying) return DebugDataSource::DemoFlying;
  if (value == kSourceDemoTurbulence) return DebugDataSource::DemoTurbulence;
  return fallback;
}

bool ParseBool(const std::string& value, bool fallback);

void ApplyPersistedProcedureMetaField(PersistedLoadedApproach& meta,
                                      const std::string& field,
                                      const std::string& value) {
  if (field == "Active") {
    meta.active = ParseBool(value, meta.active);
  } else if (field == "Airport") {
    meta.airportIcao = value;
  } else if (field == "Type") {
    try {
      const int t = std::stoi(value);
      if (t >= 0 && t <= 2) {
        meta.type = static_cast<ProcedureType>(t);
      }
    } catch (...) {
    }
  } else if (field == "Name") {
    meta.name = value;
  } else if (field == "Transition") {
    meta.transition = value;
  } else if (field == "Runway") {
    meta.runway = value;
  } else if (field == "Kind") {
    meta.approachKind = value;
  } else if (field == "Los") {
    meta.levelOfService = value;
  }
}

void WritePersistedProcedureMeta(std::ostream& out, const char* prefix,
                                 const PersistedLoadedApproach& meta) {
  out << prefix << "Active=" << (meta.active ? '1' : '0') << '\n';
  out << prefix << "Airport=" << meta.airportIcao << '\n';
  out << prefix << "Type=" << static_cast<int>(meta.type) << '\n';
  out << prefix << "Name=" << meta.name << '\n';
  out << prefix << "Transition=" << meta.transition << '\n';
  out << prefix << "Runway=" << meta.runway << '\n';
  out << prefix << "Kind=" << meta.approachKind << '\n';
  out << prefix << "Los=" << meta.levelOfService << '\n';
}

bool CatalogEntryHasTerminalProcedureMeta(const PersistedFlightPlan& entry) {
  return entry.departureMeta.active || entry.departureLegCount > 0 ||
         entry.arrivalMeta.active || entry.arrivalLegCount > 0 ||
         entry.approachLegCount > 0 || entry.approachMeta.active;
}

void WriteCatalogTerminalProcedureMeta(std::ostream& out, std::size_t index,
                                       const PersistedFlightPlan& entry) {
  if (entry.departureMeta.active || entry.departureLegCount > 0) {
    out << "fplCat" << index << "DepStart=" << entry.departureLegStart << '\n';
    out << "fplCat" << index << "DepCount=" << entry.departureLegCount << '\n';
    WritePersistedProcedureMeta(out, ("fplCat" + std::to_string(index) + "Dep").c_str(),
                                entry.departureMeta);
  }
  if (entry.arrivalMeta.active || entry.arrivalLegCount > 0) {
    out << "fplCat" << index << "ArrStart=" << entry.arrivalLegStart << '\n';
    out << "fplCat" << index << "ArrCount=" << entry.arrivalLegCount << '\n';
    WritePersistedProcedureMeta(out, ("fplCat" + std::to_string(index) + "Arr").c_str(),
                                entry.arrivalMeta);
  }
  if (entry.approachLegCount > 0 || entry.approachMeta.active) {
    out << "fplCat" << index << "ApprStart=" << entry.approachLegStart << '\n';
    out << "fplCat" << index << "ApprCount=" << entry.approachLegCount << '\n';
    out << "fplCat" << index << "ApprAirport=" << entry.approachAirportIcao
        << '\n';
    WritePersistedProcedureMeta(out, ("fplCat" + std::to_string(index) + "Appr").c_str(),
                                entry.approachMeta);
  }
}

// Per-user config directory, following each platform's convention. Empty when
// the environment does not point anywhere sensible (settings then no-op).
std::filesystem::path ConfigDir() {
#if defined(_WIN32)
  if (const char* appData = std::getenv("APPDATA")) {
    return std::filesystem::path(appData) / kAppDirName;
  }
  return {};
#elif defined(__APPLE__)
  if (const char* home = std::getenv("HOME")) {
    return std::filesystem::path(home) / "Library" / "Application Support" /
           kAppDirName;
  }
  return {};
#else
  if (const char* xdg = std::getenv("XDG_CONFIG_HOME")) {
    return std::filesystem::path(xdg) / kAppDirName;
  }
  if (const char* home = std::getenv("HOME")) {
    return std::filesystem::path(home) / ".config" / kAppDirName;
  }
  return {};
#endif
}

std::filesystem::path SettingsPath() {
  const std::filesystem::path dir = ConfigDir();
  return dir.empty() ? std::filesystem::path() : dir / kSettingsFileName;
}

bool ParseBool(const std::string& value, bool fallback) {
  if (value == "1" || value == "true") return true;
  if (value == "0" || value == "false") return false;
  return fallback;
}

// Parses a window coordinate (may legitimately be negative on multi-monitor
// setups). Marks hasWindowPos so a saved position is only restored when every
// coordinate key was actually present and numeric.
void ParseWindowCoord(const std::string& value, int& target, bool& valid) {
  try {
    target = std::stoi(value);
  } catch (...) {
    valid = false;
  }
}

}  // namespace

AppSettings LoadAppSettings() {
  AppSettings settings;
  const std::filesystem::path path = SettingsPath();
  if (path.empty()) return settings;

  std::ifstream in(path);
  if (!in.is_open()) return settings;

  std::string line;
  int windowCoords = 0;
  bool coordsValid = true;
  while (std::getline(in, line)) {
    const std::string::size_type eq = line.find('=');
    if (eq == std::string::npos) continue;
    const std::string key = line.substr(0, eq);
    const std::string value = line.substr(eq + 1);
    if (key == kKeyShowBezel) {
      settings.showBezel = ParseBool(value, settings.showBezel);
    } else if (key == kKeyNavigraphRefreshToken) {
      settings.navigraphRefreshToken = value;
    } else if (key == kKeyNavDataDir) {
      settings.navDataDir = value;
    } else if (key == kKeyShowWindowChrome) {
      settings.showWindowChrome =
          ParseBool(value, settings.showWindowChrome);
    } else if (key == kKeyAlwaysOnTop) {
      settings.alwaysOnTop = ParseBool(value, settings.alwaysOnTop);
    } else if (key == kKeyPfdFullscreen) {
      settings.pfdFullscreen = ParseBool(value, settings.pfdFullscreen);
    } else if (key == kKeyMfdFullscreen) {
      settings.mfdFullscreen = ParseBool(value, settings.mfdFullscreen);
    } else if (key == kKeyPfdMonitor) {
      try {
        settings.pfdMonitor = std::stoi(value);
      } catch (...) {
      }
    } else if (key == kKeyMfdMonitor) {
      try {
        settings.mfdMonitor = std::stoi(value);
      } catch (...) {
      }
    } else if (key == kKeyPfdWindowX) {
      ParseWindowCoord(value, settings.pfdWindowX, coordsValid);
      ++windowCoords;
    } else if (key == kKeyPfdWindowY) {
      ParseWindowCoord(value, settings.pfdWindowY, coordsValid);
      ++windowCoords;
    } else if (key == kKeyMfdWindowX) {
      ParseWindowCoord(value, settings.mfdWindowX, coordsValid);
      ++windowCoords;
    } else if (key == kKeyMfdWindowY) {
      ParseWindowCoord(value, settings.mfdWindowY, coordsValid);
      ++windowCoords;
    } else if (key == kKeyDebugDataSource) {
      settings.debugDataSource =
          ParseDebugDataSource(value, settings.debugDataSource);
    } else if (key == kKeyDemoMasterPower) {
      settings.demoMasterPowerOn =
          ParseBool(value, settings.demoMasterPowerOn);
    } else if (key == kKeyDemoAvionicsPower) {
      settings.demoAvionicsPowerOn =
          ParseBool(value, settings.demoAvionicsPowerOn);
    } else if (key == kKeyDemoCasMessages) {
      settings.demoCasMessagesOn =
          ParseBool(value, settings.demoCasMessagesOn);
    } else if (key == kKeyApproachActive) {
      settings.persistedApproach.active =
          ParseBool(value, settings.persistedApproach.active);
    } else if (key == kKeyApproachAirport) {
      settings.persistedApproach.airportIcao = value;
    } else if (key == kKeyApproachType) {
      try {
        const int t = std::stoi(value);
        if (t >= 0 && t <= 2) {
          settings.persistedApproach.type =
              static_cast<ProcedureType>(t);
        }
      } catch (...) {
      }
    } else if (key == kKeyApproachName) {
      settings.persistedApproach.name = value;
    } else if (key == kKeyApproachTransition) {
      settings.persistedApproach.transition = value;
    } else if (key == kKeyApproachRunway) {
      settings.persistedApproach.runway = value;
    } else if (key == kKeyApproachKind) {
      settings.persistedApproach.approachKind = value;
    } else if (key == kKeyApproachLos) {
      settings.persistedApproach.levelOfService = value;
    } else if (key == kKeyFplActive) {
      settings.persistedFlightPlan.active = ParseBool(value, false);
    } else if (key == kKeyFplDestFilled) {
      settings.persistedFlightPlan.destinationFilled = ParseBool(value, false);
    } else if (key == kKeyFplApproachStart) {
      try {
        settings.persistedFlightPlan.approachLegStart = std::stoi(value);
      } catch (...) {
      }
    } else if (key == kKeyFplApproachCount) {
      try {
        settings.persistedFlightPlan.approachLegCount = std::stoi(value);
      } catch (...) {
      }
    } else if (key == kKeyFplApproachAirport) {
      settings.persistedFlightPlan.approachAirportIcao = value;
    } else if (key == kKeyFplDepStart) {
      try {
        settings.persistedFlightPlan.departureLegStart = std::stoi(value);
      } catch (...) {
      }
    } else if (key == kKeyFplDepCount) {
      try {
        settings.persistedFlightPlan.departureLegCount = std::stoi(value);
      } catch (...) {
      }
    } else if (key == kKeyFplDepActive) {
      ApplyPersistedProcedureMetaField(settings.persistedFlightPlan.departureMeta,
                                       "Active", value);
    } else if (key == kKeyFplDepAirport) {
      ApplyPersistedProcedureMetaField(settings.persistedFlightPlan.departureMeta,
                                       "Airport", value);
    } else if (key == kKeyFplDepType) {
      ApplyPersistedProcedureMetaField(settings.persistedFlightPlan.departureMeta,
                                       "Type", value);
    } else if (key == kKeyFplDepName) {
      ApplyPersistedProcedureMetaField(settings.persistedFlightPlan.departureMeta,
                                       "Name", value);
    } else if (key == kKeyFplDepTransition) {
      ApplyPersistedProcedureMetaField(settings.persistedFlightPlan.departureMeta,
                                       "Transition", value);
    } else if (key == kKeyFplDepRunway) {
      ApplyPersistedProcedureMetaField(settings.persistedFlightPlan.departureMeta,
                                       "Runway", value);
    } else if (key == kKeyFplArrStart) {
      try {
        settings.persistedFlightPlan.arrivalLegStart = std::stoi(value);
      } catch (...) {
      }
    } else if (key == kKeyFplArrCount) {
      try {
        settings.persistedFlightPlan.arrivalLegCount = std::stoi(value);
      } catch (...) {
      }
    } else if (key == kKeyFplArrActive) {
      ApplyPersistedProcedureMetaField(settings.persistedFlightPlan.arrivalMeta,
                                       "Active", value);
    } else if (key == kKeyFplArrAirport) {
      ApplyPersistedProcedureMetaField(settings.persistedFlightPlan.arrivalMeta,
                                       "Airport", value);
    } else if (key == kKeyFplArrType) {
      ApplyPersistedProcedureMetaField(settings.persistedFlightPlan.arrivalMeta,
                                       "Type", value);
    } else if (key == kKeyFplArrName) {
      ApplyPersistedProcedureMetaField(settings.persistedFlightPlan.arrivalMeta,
                                       "Name", value);
    } else if (key == kKeyFplArrTransition) {
      ApplyPersistedProcedureMetaField(settings.persistedFlightPlan.arrivalMeta,
                                       "Transition", value);
    } else if (key == kKeyFplArrRunway) {
      ApplyPersistedProcedureMetaField(settings.persistedFlightPlan.arrivalMeta,
                                       "Runway", value);
    } else if (key == kKeyDtoActive) {
      settings.persistedDirectTo.active = ParseBool(value, false);
      if (!settings.persistedDirectTo.active) {
        settings.persistedDirectTo = {};
      }
    } else if (key == kKeyDtoTarget) {
      MapLeg leg;
      if (parsePersistedFlightPlanLeg(value, leg)) {
        settings.persistedDirectTo.target = std::move(leg);
      }
    } else if (key == kKeyDtoOriginLat) {
      try {
        settings.persistedDirectTo.originLat = std::stod(value);
      } catch (...) {
      }
    } else if (key == kKeyDtoOriginLon) {
      try {
        settings.persistedDirectTo.originLon = std::stod(value);
      } catch (...) {
      }
    } else if (key == kKeyDtoOriginValid) {
      settings.persistedDirectTo.originValid = ParseBool(value, false);
    } else if (key.rfind("fplLeg", 0) == 0 && key.size() > 6) {
      const int idx = std::atoi(key.c_str() + 6);
      if (idx >= 0 && idx < 64) {
        MapLeg leg;
        if (parsePersistedFlightPlanLeg(value, leg)) {
          if (settings.persistedFlightPlan.legs.size() <=
              static_cast<std::size_t>(idx)) {
            settings.persistedFlightPlan.legs.resize(
                static_cast<std::size_t>(idx + 1));
          }
          settings.persistedFlightPlan.legs[static_cast<std::size_t>(idx)] =
              std::move(leg);
        }
      }
    } else if (key.rfind("fplCat", 0) == 0) {
      // Stored Flight Plan Catalog entries. Keys: "fplCat<i>DestFilled" and
      // "fplCat<i>Leg<j>" (the entry index <i> always carries a DestFilled key,
      // so even an empty stored plan round-trips). Grow the catalog as needed.
      const char* rest = key.c_str() + 6;
      char* after = nullptr;
      const long entryIdx = std::strtol(rest, &after, 10);
      if (after != nullptr && after != rest && entryIdx >= 0 &&
          entryIdx < kFlightPlanCatalogMaxPlans) {
        const std::string sub(after);
        if (settings.flightPlanCatalog.size() <=
            static_cast<std::size_t>(entryIdx)) {
          settings.flightPlanCatalog.resize(
              static_cast<std::size_t>(entryIdx + 1));
        }
        PersistedFlightPlan& entry =
            settings.flightPlanCatalog[static_cast<std::size_t>(entryIdx)];
        entry.active = true;
        if (sub == "DestFilled") {
          entry.destinationFilled = ParseBool(value, false);
        } else if (sub == "DepStart") {
          try {
            entry.departureLegStart = std::stoi(value);
          } catch (...) {
          }
        } else if (sub == "DepCount") {
          try {
            entry.departureLegCount = std::stoi(value);
          } catch (...) {
          }
        } else if (sub.rfind("Dep", 0) == 0 && sub.size() > 3) {
          ApplyPersistedProcedureMetaField(entry.departureMeta, sub.substr(3),
                                           value);
        } else if (sub == "ArrStart") {
          try {
            entry.arrivalLegStart = std::stoi(value);
          } catch (...) {
          }
        } else if (sub == "ArrCount") {
          try {
            entry.arrivalLegCount = std::stoi(value);
          } catch (...) {
          }
        } else if (sub.rfind("Arr", 0) == 0 && sub.size() > 3) {
          ApplyPersistedProcedureMetaField(entry.arrivalMeta, sub.substr(3),
                                           value);
        } else if (sub == "ApprStart") {
          try {
            entry.approachLegStart = std::stoi(value);
          } catch (...) {
          }
        } else if (sub == "ApprCount") {
          try {
            entry.approachLegCount = std::stoi(value);
          } catch (...) {
          }
        } else if (sub == "ApprAirport") {
          entry.approachAirportIcao = value;
        } else if (sub.rfind("Appr", 0) == 0 && sub.size() > 4) {
          ApplyPersistedProcedureMetaField(entry.approachMeta, sub.substr(4),
                                           value);
        } else if (sub.rfind("Leg", 0) == 0) {
          const int legIdx = std::atoi(sub.c_str() + 3);
          if (legIdx >= 0 && legIdx < 256) {
            MapLeg leg;
            if (parsePersistedFlightPlanLeg(value, leg)) {
              if (entry.legs.size() <= static_cast<std::size_t>(legIdx)) {
                entry.legs.resize(static_cast<std::size_t>(legIdx + 1));
              }
              entry.legs[static_cast<std::size_t>(legIdx)] = std::move(leg);
            }
          }
        }
      }
    } else {
      // Durable avionics display preferences are owned by the shared core, so
      // it parses its own keys; anything else is silently ignored.
      applyStateLine(key, value, settings.avionics);
    }
  }
  settings.hasWindowPos = coordsValid && windowCoords == 4;
  if (settings.persistedApproach.active &&
      !settings.persistedFlightPlan.approachMeta.active) {
    settings.persistedFlightPlan.approachMeta = settings.persistedApproach;
    if (!settings.persistedApproach.airportIcao.empty()) {
      settings.persistedFlightPlan.approachAirportIcao =
          settings.persistedApproach.airportIcao;
    }
  }
  enrichPersistedFlightPlanFromLegs(settings.persistedFlightPlan);
  for (PersistedFlightPlan& entry : settings.flightPlanCatalog) {
    enrichPersistedFlightPlanFromLegs(entry);
  }
  settings.loaded = true;
  return settings;
}

void SaveAppSettings(const AppSettings& settings) {
  const std::filesystem::path path = SettingsPath();
  if (path.empty()) return;

  std::error_code ec;
  std::filesystem::create_directories(path.parent_path(), ec);

  std::ofstream out(path, std::ios::trunc);
  if (!out.is_open()) return;
  out << kKeyShowBezel << '=' << (settings.showBezel ? '1' : '0') << '\n';
  out << kKeyNavigraphRefreshToken << '=' << settings.navigraphRefreshToken
      << '\n';
  out << kKeyNavDataDir << '=' << settings.navDataDir << '\n';
  out << kKeyShowWindowChrome << '='
      << (settings.showWindowChrome ? '1' : '0') << '\n';
  out << kKeyAlwaysOnTop << '=' << (settings.alwaysOnTop ? '1' : '0') << '\n';
  out << kKeyPfdFullscreen << '=' << (settings.pfdFullscreen ? '1' : '0')
      << '\n';
  out << kKeyMfdFullscreen << '=' << (settings.mfdFullscreen ? '1' : '0')
      << '\n';
  out << kKeyPfdMonitor << '=' << settings.pfdMonitor << '\n';
  out << kKeyMfdMonitor << '=' << settings.mfdMonitor << '\n';
  out << kKeyDebugDataSource << '='
      << DebugDataSourceToken(settings.debugDataSource) << '\n';
  out << kKeyDemoMasterPower << '='
      << (settings.demoMasterPowerOn ? '1' : '0') << '\n';
  out << kKeyDemoAvionicsPower << '='
      << (settings.demoAvionicsPowerOn ? '1' : '0') << '\n';
  out << kKeyDemoCasMessages << '='
      << (settings.demoCasMessagesOn ? '1' : '0') << '\n';
  out << kKeyFplActive << '='
      << (settings.persistedFlightPlan.active ? '1' : '0') << '\n';
  out << kKeyFplDestFilled << '='
      << (settings.persistedFlightPlan.destinationFilled ? '1' : '0')
      << '\n';
  if (settings.persistedFlightPlan.approachLegCount > 0) {
    out << kKeyFplApproachStart << '='
        << settings.persistedFlightPlan.approachLegStart << '\n';
    out << kKeyFplApproachCount << '='
        << settings.persistedFlightPlan.approachLegCount << '\n';
    out << kKeyFplApproachAirport << '='
        << settings.persistedFlightPlan.approachAirportIcao << '\n';
    const PersistedLoadedApproach& am =
        settings.persistedFlightPlan.approachMeta.active
            ? settings.persistedFlightPlan.approachMeta
            : settings.persistedApproach;
    out << kKeyApproachActive << '=' << (am.active ? '1' : '0') << '\n';
    out << kKeyApproachAirport << '=' << am.airportIcao << '\n';
    out << kKeyApproachType << '=' << static_cast<int>(am.type) << '\n';
    out << kKeyApproachName << '=' << am.name << '\n';
    out << kKeyApproachTransition << '=' << am.transition << '\n';
    out << kKeyApproachRunway << '=' << am.runway << '\n';
    out << kKeyApproachKind << '=' << am.approachKind << '\n';
    out << kKeyApproachLos << '=' << am.levelOfService << '\n';
  } else {
    out << kKeyApproachActive << '='
        << (settings.persistedApproach.active ? '1' : '0') << '\n';
    out << kKeyApproachAirport << '=' << settings.persistedApproach.airportIcao
        << '\n';
    out << kKeyApproachType << '='
        << static_cast<int>(settings.persistedApproach.type) << '\n';
    out << kKeyApproachName << '=' << settings.persistedApproach.name << '\n';
    out << kKeyApproachTransition << '='
        << settings.persistedApproach.transition << '\n';
    out << kKeyApproachRunway << '=' << settings.persistedApproach.runway
        << '\n';
    out << kKeyApproachKind << '=' << settings.persistedApproach.approachKind
        << '\n';
    out << kKeyApproachLos << '='
        << settings.persistedApproach.levelOfService << '\n';
  }
  if (settings.persistedFlightPlan.departureMeta.active ||
      settings.persistedFlightPlan.departureLegCount > 0) {
    out << kKeyFplDepStart << '='
        << settings.persistedFlightPlan.departureLegStart << '\n';
    out << kKeyFplDepCount << '='
        << settings.persistedFlightPlan.departureLegCount << '\n';
    WritePersistedProcedureMeta(out, "fplDep",
                                settings.persistedFlightPlan.departureMeta);
  }
  if (settings.persistedFlightPlan.arrivalMeta.active ||
      settings.persistedFlightPlan.arrivalLegCount > 0) {
    out << kKeyFplArrStart << '='
        << settings.persistedFlightPlan.arrivalLegStart << '\n';
    out << kKeyFplArrCount << '='
        << settings.persistedFlightPlan.arrivalLegCount << '\n';
    WritePersistedProcedureMeta(out, "fplArr",
                                settings.persistedFlightPlan.arrivalMeta);
  }
  for (std::size_t i = 0; i < settings.persistedFlightPlan.legs.size(); ++i) {
    out << "fplLeg" << i << '='
        << formatPersistedFlightPlanLeg(
               settings.persistedFlightPlan.legs[i])
        << '\n';
  }
  // Stored Flight Plan Catalog entries. Each entry writes a DestFilled key
  // (so empty stored plans round-trip) plus one Leg key per leg.
  for (std::size_t i = 0;
       i < settings.flightPlanCatalog.size() &&
       i < static_cast<std::size_t>(kFlightPlanCatalogMaxPlans);
       ++i) {
    const PersistedFlightPlan& entry = settings.flightPlanCatalog[i];
    out << "fplCat" << i << "DestFilled="
        << (entry.destinationFilled ? '1' : '0') << '\n';
    if (CatalogEntryHasTerminalProcedureMeta(entry)) {
      WriteCatalogTerminalProcedureMeta(out, i, entry);
    }
    for (std::size_t j = 0; j < entry.legs.size(); ++j) {
      out << "fplCat" << i << "Leg" << j << '='
          << formatPersistedFlightPlanLeg(entry.legs[j]) << '\n';
    }
  }
  if (settings.persistedDirectTo.active &&
      !settings.persistedDirectTo.target.id.empty()) {
    out << kKeyDtoActive << "=1\n";
    out << kKeyDtoTarget << '='
        << formatPersistedFlightPlanLeg(settings.persistedDirectTo.target)
        << '\n';
    out << kKeyDtoOriginLat << '=' << settings.persistedDirectTo.originLat
        << '\n';
    out << kKeyDtoOriginLon << '=' << settings.persistedDirectTo.originLon
        << '\n';
    out << kKeyDtoOriginValid << '='
        << (settings.persistedDirectTo.originValid ? '1' : '0') << '\n';
  } else {
    out << kKeyDtoActive << "=0\n";
  }
  // Window coordinates are only written once a position has been captured, so
  // a fresh install never restores a bogus (0, 0) placement.
  if (settings.hasWindowPos) {
    out << kKeyPfdWindowX << '=' << settings.pfdWindowX << '\n';
    out << kKeyPfdWindowY << '=' << settings.pfdWindowY << '\n';
    out << kKeyMfdWindowX << '=' << settings.mfdWindowX << '\n';
    out << kKeyMfdWindowY << '=' << settings.mfdWindowY << '\n';
  }
  // Durable avionics display preferences, serialized by the shared core.
  std::string avionicsLines;
  appendStateLines(settings.avionics, avionicsLines);
  out << avionicsLines;
}

}  // namespace avionics
