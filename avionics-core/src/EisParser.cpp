#include "avionics/Eis.h"

#include <cctype>
#include <cstdlib>
#include <filesystem>
#include <sstream>

namespace avionics {
namespace {

std::string trim(const std::string& s) {
  std::size_t begin = 0;
  std::size_t end = s.size();
  while (begin < end && std::isspace(static_cast<unsigned char>(s[begin]))) {
    ++begin;
  }
  while (end > begin && std::isspace(static_cast<unsigned char>(s[end - 1]))) {
    --end;
  }
  return s.substr(begin, end - begin);
}

bool matchKeyword(const std::string& line, const char* keyword,
                  std::string& rest) {
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

bool parseGaugeType(const std::string& token, EisGaugeType& out) {
  if (token == "RPM_DIAL") {
    out = EisGaugeType::RpmDial;
    return true;
  }
  if (token == "BAR") {
    out = EisGaugeType::Bar;
    return true;
  }
  if (token == "FUEL_QTY") {
    out = EisGaugeType::FuelQty;
    return true;
  }
  if (token == "FUEL_QTY_VERT") {
    out = EisGaugeType::FuelQtyVert;
    return true;
  }
  if (token == "READOUT") {
    out = EisGaugeType::Readout;
    return true;
  }
  if (token == "ELECTRICAL") {
    out = EisGaugeType::Electrical;
    return true;
  }
  if (token == "DIAL") {
    out = EisGaugeType::Dial;
    return true;
  }
  return false;
}

bool parseBandColor(const std::string& token, EisBandColor& out) {
  if (token == "GREEN") {
    out = EisBandColor::Green;
    return true;
  }
  if (token == "YELLOW") {
    out = EisBandColor::Yellow;
    return true;
  }
  if (token == "RED") {
    out = EisBandColor::Red;
    return true;
  }
  return false;
}

}  // namespace

Color eisBandColor(EisBandColor color) {
  switch (color) {
    case EisBandColor::Green:
      return colors::kBandGreen;
    case EisBandColor::Yellow:
      return colors::kBandYellow;
    case EisBandColor::Red:
      return colors::kBandRed;
  }
  return colors::kBandGreen;
}

EisLayout parseEisText(const std::string& text) {
  EisLayout layout;
  EisSection* currentSection = nullptr;
  EisGauge* currentGauge = nullptr;
  bool inGauge = false;

  auto ensureSection = [&]() {
    if (currentSection == nullptr) {
      layout.sections.push_back(EisSection{});
      currentSection = &layout.sections.back();
    }
  };

  std::istringstream in(text);
  std::string raw;
  while (std::getline(in, raw)) {
    const std::string line = trim(raw);
    if (line.empty() || line[0] == '#') continue;

    std::string rest;
    if (matchKeyword(line, "TITLE", rest)) {
      layout.stripTitle = rest;
      currentGauge = nullptr;
      inGauge = false;
      continue;
    }
    if (matchKeyword(line, "STYLE", rest)) {
      if (rest == "TURBOFAN") {
        layout.style = EisStripStyle::Turbofan;
      } else if (rest == "TURBOPROP") {
        layout.style = EisStripStyle::Turboprop;
      } else if (rest == "CARAVAN") {
        layout.style = EisStripStyle::Caravan;
      } else {
        layout.style = EisStripStyle::Piston;
      }
      currentGauge = nullptr;
      inGauge = false;
      continue;
    }
    if (matchKeyword(line, "SECTION", rest)) {
      layout.sections.push_back(EisSection{rest, {}});
      currentSection = &layout.sections.back();
      currentGauge = nullptr;
      inGauge = false;
      continue;
    }
    if (matchKeyword(line, "BIND", rest)) {
      std::istringstream bind(rest);
      EisDataBinding binding;
      bind >> binding.channel >> binding.datarefPath >> binding.scale >>
          binding.offset;
      if (!binding.channel.empty() && !binding.datarefPath.empty()) {
        layout.bindings.push_back(std::move(binding));
      }
      currentGauge = nullptr;
      inGauge = false;
      continue;
    }
    if (matchKeyword(line, "GAUGE", rest)) {
      ensureSection();
      EisGaugeType type = EisGaugeType::Bar;
      std::string typeToken;
      std::istringstream gauge(rest);
      gauge >> typeToken;
      if (!parseGaugeType(typeToken, type)) continue;
      currentSection->gauges.push_back(EisGauge{});
      currentGauge = &currentSection->gauges.back();
      currentGauge->type = type;
      inGauge = true;
      continue;
    }

    if (!inGauge || currentGauge == nullptr) continue;

    if (matchKeyword(line, "CHANNEL", rest)) {
      currentGauge->channel = rest;
    } else if (matchKeyword(line, "LABEL", rest)) {
      currentGauge->label = rest;
    } else if (matchKeyword(line, "LEFT", rest)) {
      std::istringstream left(rest);
      left >> currentGauge->leftTag >> currentGauge->channel;
    } else if (matchKeyword(line, "RIGHT", rest)) {
      std::istringstream right(rest);
      right >> currentGauge->rightTag >> currentGauge->channelRight;
    } else if (matchKeyword(line, "FORMAT", rest)) {
      currentGauge->format = rest;
    } else if (matchKeyword(line, "MIN", rest)) {
      std::istringstream minLine(rest);
      minLine >> currentGauge->min;
      std::string maxToken;
      float maxVal = 0.0f;
      if (minLine >> maxToken && maxToken == "MAX") {
        minLine >> maxVal;
        currentGauge->max = maxVal;
      }
    } else if (matchKeyword(line, "MAX", rest)) {
      std::istringstream maxLine(rest);
      maxLine >> currentGauge->max;
    } else if (matchKeyword(line, "REDLINE", rest)) {
      currentGauge->redline = std::strtof(rest.c_str(), nullptr);
      currentGauge->hasRedline = true;
    } else if (matchKeyword(line, "TICKS", rest)) {
      currentGauge->ticks = std::atoi(rest.c_str());
    } else if (matchKeyword(line, "SCALE", rest)) {
      currentGauge->scale = true;
    } else if (matchKeyword(line, "PFD", rest)) {
      currentGauge->pfd = true;
    } else if (matchKeyword(line, "BUG_CHANNEL", rest)) {
      currentGauge->bugChannel = rest;
      currentGauge->hasBug = true;
    } else if (matchKeyword(line, "REDLINE_CHANNEL", rest)) {
      currentGauge->redlineChannel = rest;
      currentGauge->hasRedline = true;
    } else if (matchKeyword(line, "BUG", rest)) {
      currentGauge->bug = std::strtof(rest.c_str(), nullptr);
      currentGauge->hasBug = true;
    } else if (matchKeyword(line, "BAND", rest)) {
      std::istringstream band(rest);
      std::string colorToken;
      EisBandColor color = EisBandColor::Green;
      float lo = 0.0f;
      float hi = 0.0f;
      band >> colorToken >> lo >> hi;
      if (parseBandColor(colorToken, color)) {
        currentGauge->bands.push_back(EisBand{color, lo, hi});
      }
    }
  }

  return layout;
}

std::vector<std::string> candidateEisPaths(
    const std::string& explicitSelector,
    const std::string& aircraftAcfRelativePath,
    const std::string& typeKeyedPath,
    const std::string& defaultBundledPath) {
  std::vector<std::string> paths;
  namespace fs = std::filesystem;

  auto pushIfFile = [&](const std::string& path) {
    if (path.empty()) return;
    std::error_code ec;
    if (fs::is_regular_file(path, ec)) paths.push_back(path);
  };

  pushIfFile(explicitSelector);

  if (!aircraftAcfRelativePath.empty()) {
    const fs::path acf = fs::path(aircraftAcfRelativePath);
    const fs::path dir = acf.parent_path();
    pushIfFile((dir / "g1000_eis.txt").string());
    if (acf.has_stem()) {
      pushIfFile((dir / (acf.stem().string() + "_eis.txt")).string());
    }
  }

  pushIfFile(typeKeyedPath);
  pushIfFile(defaultBundledPath);
  return paths;
}

}  // namespace avionics
