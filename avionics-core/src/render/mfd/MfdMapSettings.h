#pragma once

#include "avionics/MfdController.h"

// Shared layout descriptor for the MFD Map Settings window (Navigation Map ->
// MENU -> Map Settings, Pilot's Guide Fig. 5-7). Both the controller (cursor
// navigation + editing) and the page renderer (drawing) walk the same per-group
// row table so they never drift apart.
//
// The Map group rows are verbatim from Fig. 5-7; the Weather/Traffic/Aviation/
// Airspace/Land groups follow the WT NXi MFDMapSettings setting groups (the
// guide only figures the Map group). Row labels are reproduced exactly.
namespace avionics::mfd {

// How a control in a settings row behaves. Toggle and Enum render in the left
// (control) column; Range and the read-only Value render in the right (value)
// column. None marks an empty slot. The cursor lands only on editable controls
// (Toggle / Enum / Range); Value cells are display-only, like the real unit's
// dependent read-outs (e.g. Auto Zoom's "All", the look-ahead times).
enum class MsKind { None, Toggle, Enum, Range, Value };

struct MsField {
  MsKind kind = MsKind::None;
  MapSetting id = MapSetting::Orientation;
};

struct MsRow {
  const char* label;
  bool indent;  // dependent sub-rows (Auto Zoom look-ahead) are indented
  MsField left;
  MsField right;
};

// The active group's row table. `count` receives the row count.
const MsRow* msRows(MapSettingsGroup group, int& count);

// Group tab name as shown in the Group selector dropdown.
const char* msGroupName(MapSettingsGroup group);

// The control kind for a setting id (searches the row tables). Returns
// MsKind::None for an id that no row references.
MsKind msControlKind(MapSetting id);

// Fills `out` (capacity `maxOut`) with the editable controls of `group` in
// cursor order, so cursor position 1 maps to out[0], 2 to out[1], and so on
// (cursor 0 is the Group selector). Returns the number written.
int msEditableFields(MapSettingsGroup group, MapSetting* out, int maxOut);

}  // namespace avionics::mfd
