#include "DatarefDataSource.h"

#include <string>

#include "XPLMNavigation.h"
#include "avionics/Datarefs.h"

namespace avionics {
namespace {

// Identifier of a flight-plan entry (e.g. "KFMY"). The flight plan is only
// reachable through the FMS SDK, not datarefs. The SDK fills outID with the
// waypoint identifier; the buffer must be generous (lat/lon entries can be
// long), so use the documented 256-byte size and force null-termination.
std::string fmsEntryId(int index) {
  XPLMNavType type = xplm_Nav_Unknown;
  char id[256] = {};
  XPLMNavRef ref = XPLM_NAV_NOT_FOUND;
  int altitude = 0;
  float lat = 0.0f;
  float lon = 0.0f;
  XPLMGetFMSEntryInfo(index, &type, id, &ref, &altitude, &lat, &lon);
  id[sizeof(id) - 1] = '\0';
  return std::string(id);
}

}  // namespace

DatarefDataSource::DatarefDataSource() {
  airspeed_ = XPLMFindDataRef(datarefs::kAirspeedKts);
  altitude_ = XPLMFindDataRef(datarefs::kAltitudeFt);
  heading_ = XPLMFindDataRef(datarefs::kHeadingDegMag);
  pitch_ = XPLMFindDataRef(datarefs::kPitchDeg);
  roll_ = XPLMFindDataRef(datarefs::kRollDeg);
  verticalSpeed_ = XPLMFindDataRef(datarefs::kVerticalSpeedFpm);
  slip_ = XPLMFindDataRef(datarefs::kSlipDeg);

  gpsDistance_ = XPLMFindDataRef(datarefs::kGpsDistanceNm);
  gpsBearing_ = XPLMFindDataRef(datarefs::kGpsBearingDegMag);
  gpsNavId_ = XPLMFindDataRef(datarefs::kGpsNavId);
}

void DatarefDataSource::update(double /*dtSeconds*/) {
  if (airspeed_) data_.airspeedKts = XPLMGetDataf(airspeed_);
  if (altitude_) data_.altitudeFt = XPLMGetDataf(altitude_);
  if (heading_) data_.headingDeg = XPLMGetDataf(heading_);
  if (pitch_) data_.pitchDeg = XPLMGetDataf(pitch_);
  if (roll_) data_.rollDeg = XPLMGetDataf(roll_);
  if (verticalSpeed_) data_.verticalSpeedFpm = XPLMGetDataf(verticalSpeed_);
  if (slip_) data_.slipSkidDeg = XPLMGetDataf(slip_);

  // Nav status box: distance + magnetic bearing to the active GPS destination.
  if (gpsDistance_) data_.fmaLegDistanceNm = XPLMGetDataf(gpsDistance_);
  if (gpsBearing_) data_.fmaLegBearingDeg = XPLMGetDataf(gpsBearing_);

  // Active flight-plan leg (FROM -> TO). The entry the FMS is flying toward is
  // the TO waypoint; the one before it is FROM (e.g. KFMY -> KLAL).
  const int fmsCount = XPLMCountFMSEntries();
  if (fmsCount > 0) {
    int dest = XPLMGetDestinationFMSEntry();
    if (dest < 0) dest = 0;
    if (dest >= fmsCount) dest = fmsCount - 1;
    data_.fmaToWpt = fmsEntryId(dest);
    data_.fmaFromWpt = (dest > 0) ? fmsEntryId(dest - 1) : std::string();
  } else if (gpsNavId_) {
    // No flight plan entered: fall back to the active GPS destination id (a
    // null-terminated byte string), rendering as a direct-to "->KXXX".
    char buf[32] = {};
    int n = XPLMGetDatab(gpsNavId_, buf, 0, static_cast<int>(sizeof(buf)) - 1);
    if (n < 0) n = 0;
    buf[n] = '\0';
    data_.fmaToWpt = buf;
    data_.fmaFromWpt.clear();
  }
}

}  // namespace avionics
