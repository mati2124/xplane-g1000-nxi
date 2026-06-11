#pragma once

#include <string>

namespace avionics {

// Currency of the loaded navigation (aviation) database. The G1000 NXi alerts
// the pilot to an out-of-date navigation database passively: the power-up page
// lists each database's cycle / effective / expires dates and shows expired
// ones in amber, and the AUX - System Status database window highlights any
// active database whose expiration date is in the past with amber text
// (G1000 NXi Pilot's Guide for Cessna Nav III, 190-02177-00, Appendix B).
struct NavDatabaseInfo {
  bool available = false;
  bool expired = false;   // current date is on/after the expires date
  std::string cycle;      // AIRAC cycle ident, e.g. "2506"
  std::string effective;  // e.g. "07-MAY-26"
  std::string expires;    // e.g. "04-JUN-26"
};

// Builds the info block for an AIRAC cycle number in YYCC form (e.g. 2506 =
// 6th 28-day cycle of 2025): computes the effective/expiration window from the
// AIRAC calendar and flags it expired against the current system UTC date.
// Returns a default (unavailable) info for nonsensical cycle numbers.
NavDatabaseInfo navDatabaseInfoForCycle(int cycleYYCC);

// The AIRAC cycle (YYCC form) in effect at the current system UTC date.
int currentAiracCycle();

}  // namespace avionics
