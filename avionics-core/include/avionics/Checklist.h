#pragma once

#include <string>
#include <vector>

namespace avionics {

// Author-supplied electronic checklists, modeled on the Garmin G1000 NXi
// Checklist page group. The definitions are static data loaded from a per-
// aircraft file by the shell (see the shell's ChecklistStore); the runtime
// "checked" state of each item is interactive UI state owned by MfdController,
// not stored here.
//
// Structure mirrors the real unit: a checklist file is a set of named groups
// (e.g. "NORMAL PROCEDURES", "EMERGENCY PROCEDURES"); each group holds named
// checklists (e.g. "BEFORE STARTING ENGINE"); each checklist is a list of
// items ("Master Switch ....... ON").

// One line of a checklist: a description (left) and the expected response
// (right). Items with an empty response render as a plain line (e.g. a note).
struct ChecklistItem {
  std::string text;
  std::string response;
};

struct Checklist {
  std::string title;
  std::vector<ChecklistItem> items;
};

struct ChecklistGroup {
  std::string name;
  std::vector<Checklist> checklists;
};

// The full set parsed from one file. The MFD pages a flat sequence of all
// checklists across all groups (each checklist is one "page"); the owning
// group name is shown in the page header.
struct ChecklistData {
  std::vector<ChecklistGroup> groups;

  bool empty() const { return totalChecklists() == 0; }

  // Number of checklists across every group (the flat page count).
  int totalChecklists() const {
    int n = 0;
    for (const ChecklistGroup& g : groups) n += static_cast<int>(g.checklists.size());
    return n;
  }

  // Resolve a flat checklist index in [0, totalChecklists()) to its checklist.
  // When groupName is non-null it is set to the owning group's name. Returns
  // null if the index is out of range.
  const Checklist* at(int flatIndex, const std::string** groupName = nullptr) const {
    if (flatIndex < 0) return nullptr;
    int remaining = flatIndex;
    for (const ChecklistGroup& g : groups) {
      const int count = static_cast<int>(g.checklists.size());
      if (remaining < count) {
        if (groupName != nullptr) *groupName = &g.name;
        return &g.checklists[static_cast<std::size_t>(remaining)];
      }
      remaining -= count;
    }
    return nullptr;
  }
};

// "Where the aircraft's checklists come from." The platform-agnostic core reads
// the checklists through this interface; the shell implements it by loading the
// author-editable file from disk (mirroring NavFeatureSource for nav data).
class ChecklistSource {
 public:
  virtual ~ChecklistSource() = default;

  // True once the file has finished loading (loading is async on most backends).
  virtual bool ready() const = 0;

  // The parsed checklists. Returns an empty set until ready(), or when no file
  // was found.
  virtual const ChecklistData& checklists() const = 0;

  // The loaded aircraft's ICAO type code (acf_ICAO) plus its .acf relative
  // path, so the store can swap to the per-aircraft checklist profile. No-op by
  // default for sources that load a fixed file.
  virtual void setAircraftIdentity(const std::string& icaoType,
                                   const std::string& acfRelativePath) {
    (void)icaoType;
    (void)acfRelativePath;
  }

  // Cheap mtime re-check so authors can edit checklist files without a restart.
  virtual void refreshIfChanged() {}
};

// Candidate paths for a per-aircraft checklist file, in search order:
//   1. `explicitSelector` (a CLI/path override) when it points at a regular
//      file.
//   2. Beside the .acf when `aircraftAcfRelativePath` is non-empty:
//      g1000_checklist.txt and <acf_stem>_checklist.txt.
//   3. `typeKeyedPath` — a user-droppable, ICAO-keyed asset
//      (assets/checklists/<icao>.checklist) resolved by the store; lets users
//      add any aircraft without a rebuild.
//   4. `defaultBundledPath` — the profile's bundled checklist.
// Empty arguments are skipped, as are paths that are not regular files.
std::vector<std::string> candidateChecklistPaths(
    const std::string& explicitSelector,
    const std::string& aircraftAcfRelativePath,
    const std::string& typeKeyedPath,
    const std::string& defaultBundledPath);

// Parses the indented-text checklist format into a ChecklistData. The format is
// line-oriented and forgiving of leading/trailing whitespace:
//
//   # comment line (ignored)
//   GROUP Normal Procedures
//     CHECKLIST Before Starting Engine
//       Preflight Inspection : COMPLETE
//       Seats, Belts : ADJUST AND LOCK
//       Brakes : TEST AND SET
//
//   - "GROUP <name>"     starts a new group.
//   - "CHECKLIST <name>" starts a new checklist in the current group (a default
//     unnamed group is created if none has been declared yet).
//   - Any other non-empty, non-comment line is an item; the description and the
//     expected response are split on the first " : " (space-colon-space). A line
//     with no " : " becomes an item with an empty response.
ChecklistData parseChecklistText(const std::string& text);

}  // namespace avionics
