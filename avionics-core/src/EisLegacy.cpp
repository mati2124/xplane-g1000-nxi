#include "avionics/EisLegacy.h"

#include "avionics/Eis.h"

namespace avionics {

float eisChannelValue(const FlightData& data, const std::string& channel,
                      float fallback) {
  const auto it = data.eisChannels.find(channel);
  if (it == data.eisChannels.end()) return fallback;
  float val = it->second;
  if (channel == "eng.prop_rpm" && val < 0.0f) {
    return 0.0f;
  }
  return val;
}

void syncEisLegacyFields(FlightData& data) {
  auto copy = [&](const char* channel, float& field) {
    const auto it = data.eisChannels.find(channel);
    if (it != data.eisChannels.end()) field = it->second;
  };
  copy(eis_channels::kEngRpm, data.engineRpm);
  copy(eis_channels::kFuelFlow, data.fuelFlowGph);
  copy(eis_channels::kOilPres, data.oilPressurePsi);
  copy(eis_channels::kOilTemp, data.oilTempDegF);
  copy(eis_channels::kEgt, data.egtDegF);
  copy(eis_channels::kVacuum, data.vacuumInHg);
  copy(eis_channels::kFuelQtyLeft, data.fuelQtyLeftGal);
  copy(eis_channels::kFuelQtyRight, data.fuelQtyRightGal);
  copy(eis_channels::kEngHours, data.engineHours);
  copy(eis_channels::kBusVoltsMain, data.busVoltsMain);
  copy(eis_channels::kBusVoltsEss, data.busVoltsEssential);
  copy(eis_channels::kBattAmpsMain, data.battAmpsMain);
  copy(eis_channels::kBattAmpsStandby, data.battAmpsStandby);
}

}  // namespace avionics
