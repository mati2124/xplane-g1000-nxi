#include "avionics/Checklist.h"

#include <cctype>
#include <sstream>

namespace avionics {
namespace {

constexpr const char* kGroupKeyword = "GROUP";
constexpr const char* kChecklistKeyword = "CHECKLIST";
// Separator between an item's description and its expected response.
constexpr const char* kResponseSep = " : ";

std::string trim(const std::string& s) {
  std::size_t begin = 0;
  std::size_t end = s.size();
  while (begin < end && std::isspace(static_cast<unsigned char>(s[begin]))) ++begin;
  while (end > begin && std::isspace(static_cast<unsigned char>(s[end - 1]))) --end;
  return s.substr(begin, end - begin);
}

// If `line` begins with `keyword` followed by whitespace, returns the trimmed
// remainder via `rest` and true; otherwise returns false.
bool matchKeyword(const std::string& line, const char* keyword, std::string& rest) {
  const std::string kw(keyword);
  if (line.size() < kw.size()) return false;
  if (line.compare(0, kw.size(), kw) != 0) return false;
  if (line.size() == kw.size()) {
    rest.clear();
    return true;
  }
  if (!std::isspace(static_cast<unsigned char>(line[kw.size()]))) return false;
  rest = trim(line.substr(kw.size()));
  return true;
}

}  // namespace

ChecklistData parseChecklistText(const std::string& text) {
  ChecklistData data;
  ChecklistGroup* currentGroup = nullptr;
  Checklist* currentChecklist = nullptr;

  std::istringstream in(text);
  std::string raw;
  while (std::getline(in, raw)) {
    const std::string line = trim(raw);
    if (line.empty() || line[0] == '#') continue;

    std::string rest;
    if (matchKeyword(line, kGroupKeyword, rest)) {
      data.groups.push_back(ChecklistGroup{rest, {}});
      currentGroup = &data.groups.back();
      currentChecklist = nullptr;
      continue;
    }
    if (matchKeyword(line, kChecklistKeyword, rest)) {
      if (currentGroup == nullptr) {
        data.groups.push_back(ChecklistGroup{std::string(), {}});
        currentGroup = &data.groups.back();
      }
      currentGroup->checklists.push_back(Checklist{rest, {}});
      currentChecklist = &currentGroup->checklists.back();
      continue;
    }

    // Item line: skip until a checklist exists to hold it.
    if (currentChecklist == nullptr) continue;
    const std::size_t sep = line.find(kResponseSep);
    if (sep == std::string::npos) {
      currentChecklist->items.push_back(ChecklistItem{line, std::string()});
    } else {
      currentChecklist->items.push_back(ChecklistItem{
          trim(line.substr(0, sep)),
          trim(line.substr(sep + std::string(kResponseSep).size()))});
    }
  }

  return data;
}

}  // namespace avionics
