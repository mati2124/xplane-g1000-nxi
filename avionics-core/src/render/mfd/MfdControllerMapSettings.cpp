#include <algorithm>

#include "avionics/MfdController.h"
#include "render/mfd/MfdMapSettings.h"

// Map Settings window (Navigation Map -> MENU -> Map Settings, Fig. 5-7): the
// Group selector plus the active group's editable controls. The shared controls
// (Orientation, Terrain, NEXRAD, Traffic) read/write the existing members so the
// window and the softkeys stay in sync.
namespace avionics {

void MfdController::openMapSettings() {
  mapSettingsOpen_ = true;
  // The window opens with the cursor on the Group selector (Pilot's Guide:
  // "Map Group Selection"), keeping the last-viewed group.
  mapSettingsCursor_ = 0;
}

int MfdController::mapSettingsFieldCount() const {
  MapSetting fields[static_cast<std::size_t>(MapSetting::Count)];
  return mfd::msEditableFields(
      mapSettingsGroup_, fields,
      static_cast<int>(MapSetting::Count));
}

MapSetting MfdController::mapSettingAtCursor(int cursor) const {
  if (cursor <= 0) return MapSetting::Count;
  MapSetting fields[static_cast<std::size_t>(MapSetting::Count)];
  const int n = mfd::msEditableFields(mapSettingsGroup_, fields,
                                      static_cast<int>(MapSetting::Count));
  if (cursor - 1 >= n) return MapSetting::Count;
  return fields[cursor - 1];
}

void MfdController::mapSettingsStepCursor(int dir) {
  const int total = 1 + mapSettingsFieldCount();  // Group selector + controls
  const int step = dir >= 0 ? 1 : -1;
  mapSettingsCursor_ = ((mapSettingsCursor_ + step) % total + total) % total;
}

void MfdController::mapSettingsEdit(int dir) {
  // The Group selector: cycle the active group (Pilot's Guide: small FMS knob
  // selects the group), and keep the cursor on the selector.
  if (mapSettingsCursor_ == 0) {
    constexpr int kGroupCount =
        static_cast<int>(MapSettingsGroup::Land) + 1;
    const int step = dir >= 0 ? 1 : -1;
    int g = (static_cast<int>(mapSettingsGroup_) + step) % kGroupCount;
    if (g < 0) g += kGroupCount;
    mapSettingsGroup_ = static_cast<MapSettingsGroup>(g);
    return;
  }

  const MapSetting id = mapSettingAtCursor(mapSettingsCursor_);
  if (id == MapSetting::Count) return;

  // Shared enums step the existing members, so the window and softkeys agree.
  if (id == MapSetting::Orientation) {
    static constexpr MapOrientation kCycle[] = {
        MapOrientation::NorthUp, MapOrientation::TrackUp,
        MapOrientation::HeadingUp};
    const int step = dir >= 0 ? 1 : -1;
    for (int i = 0; i < 3; ++i) {
      if (kCycle[i] == mapOrientation_) {
        mapOrientation_ = kCycle[((i + step) % 3 + 3) % 3];
        mapResetPointer();
        return;
      }
    }
    mapOrientation_ = MapOrientation::NorthUp;
    return;
  }
  if (id == MapSetting::TerrainMode) {
    // Off -> Topo -> REL, matching the TER softkey cycle.
    if (dir >= 0) {
      terrain_ = terrain_ == TerrainDisplay::Off   ? TerrainDisplay::Topo
                 : terrain_ == TerrainDisplay::Topo ? TerrainDisplay::Rel
                                                    : TerrainDisplay::Off;
    } else {
      terrain_ = terrain_ == TerrainDisplay::Off   ? TerrainDisplay::Rel
                 : terrain_ == TerrainDisplay::Rel ? TerrainDisplay::Topo
                                                   : TerrainDisplay::Off;
    }
    return;
  }
  if (id == MapSetting::TrafficMode) {
    const int step = dir >= 0 ? 1 : -1;
    msTrafficMode_ = ((msTrafficMode_ + step) % 3 + 3) % 3;
    return;
  }

  switch (mfd::msControlKind(id)) {
    case mfd::MsKind::Toggle:
      if (id == MapSetting::NexradOn) {
        showWeather_ = !showWeather_;
      } else if (id == MapSetting::TrafficOn) {
        showTraffic_ = !showTraffic_;
      } else {
        bool& v = msToggle_[static_cast<std::size_t>(id)];
        v = !v;
      }
      break;
    case mfd::MsKind::Range: {
      int& idx = msRange_[static_cast<std::size_t>(id)];
      idx = std::max(0, std::min(kMapRangeLadderCount - 1,
                                 idx + (dir >= 0 ? 1 : -1)));
      break;
    }
    default:
      break;
  }
}

bool MfdController::mapSettingsBezelKey(BezelKey key) {
  switch (key) {
    case BezelKey::Clr:
    case BezelKey::FmsPush:
      // Pilot's Guide: "Press FMS Knob To Return"; CLR also backs out.
      mapSettingsOpen_ = false;
      break;
    case BezelKey::FmsOuterCw:
      mapSettingsStepCursor(1);
      break;
    case BezelKey::FmsOuterCcw:
      mapSettingsStepCursor(-1);
      break;
    case BezelKey::FmsInnerCw:
    case BezelKey::Ent:
      mapSettingsEdit(1);
      break;
    case BezelKey::FmsInnerCcw:
      mapSettingsEdit(-1);
      break;
    default:
      break;
  }
  return true;
}

std::string MfdController::mapSettingText(MapSetting id) const {
  switch (id) {
    case MapSetting::Orientation:
      switch (mapOrientation_) {
        case MapOrientation::TrackUp:
          return "Track up";
        case MapOrientation::HeadingUp:
          return "HDG up";
        case MapOrientation::NorthUp:
          break;
      }
      return "North up";
    case MapSetting::TerrainMode:
      switch (terrain_) {
        case TerrainDisplay::Topo:
          return "Topo";
        case TerrainDisplay::Rel:
          return "REL";
        case TerrainDisplay::Off:
          break;
      }
      return "Off";
    case MapSetting::TrafficMode:
      return msTrafficMode_ == 1   ? "TA/PA"
             : msTrafficMode_ == 2 ? "TA Only"
                                   : "All Traffic";
    // Dependent read-outs shown without carets on the real unit (Fig. 5-7).
    case MapSetting::AutoZoomMax:
      return "All";
    case MapSetting::MaxLookFwd:
      return "30min";
    case MapSetting::MinLookFwd:
      return "5min";
    case MapSetting::TimeOut:
      return "0min";
    case MapSetting::TrackVectorTime:
      return "60 sec";
    case MapSetting::FuelRangeRsv:
      return "0+45";
    default:
      break;
  }
  switch (mfd::msControlKind(id)) {
    case mfd::MsKind::Toggle:
      return mapSettingOn(id) ? "On" : "Off";
    case mfd::MsKind::Range: {
      char buf[16];
      formatMapRange(buf, sizeof(buf),
                     mapRangeNmAt(msRange_[static_cast<std::size_t>(id)]));
      return buf;
    }
    default:
      break;
  }
  return "";
}

bool MfdController::mapSettingOn(MapSetting id) const {
  switch (id) {
    case MapSetting::NexradOn:
      return showWeather_;
    case MapSetting::TrafficOn:
      return showTraffic_;
    default:
      break;
  }
  return msToggle_[static_cast<std::size_t>(id)];
}

float MfdController::mapSettingRangeNm(MapSetting id) const {
  return mapRangeNmAt(msRange_[static_cast<std::size_t>(id)]);
}

}  // namespace avionics
