#pragma once

namespace avionics {

// Query GitHub for a newer release on a background thread and surface a
// platform-appropriate notice when one exists. No-op when AVIONICS_SKIP_UPDATE_CHECK
// is set in the environment.
void startUpdateCheckOnLaunch();

}  // namespace avionics
