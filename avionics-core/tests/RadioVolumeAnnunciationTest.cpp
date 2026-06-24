#include <gtest/gtest.h>

#include "avionics/SoftkeyController.h"

namespace avionics {
namespace {

constexpr double kFrameSeconds = 1.0 / 60.0;

void syncPeers(SoftkeyController& pfd, SoftkeyController& mfd) {
  syncRadioVolumeAnnunciation(pfd, mfd);
}

// Standalone shell order: PFD update + sync, then MFD update + sync.
void tickBothGdu(SoftkeyController& pfd, SoftkeyController& mfd,
                 const FlightData& d, const MapData& map) {
  pfd.update(kFrameSeconds, d, map);
  syncPeers(pfd, mfd);
  mfd.update(kFrameSeconds, d, map);
  syncPeers(pfd, mfd);
}

TEST(RadioVolumeAnnunciationTest, ClearsAfterTwoSecondsOnBothGdu) {
  SoftkeyController pfd;
  SoftkeyController mfd;
  FlightData d;
  MapData map;

  pfd.adjustNavVolume(+1, d);
  syncPeers(pfd, mfd);
  ASSERT_TRUE(pfd.radioVolumeShown(RadioBand::Nav));
  ASSERT_TRUE(mfd.radioVolumeShown(RadioBand::Nav));

  for (int i = 0; i < 180; ++i) tickBothGdu(pfd, mfd, d, map);

  EXPECT_FALSE(pfd.radioVolumeShown(RadioBand::Nav));
  EXPECT_FALSE(mfd.radioVolumeShown(RadioBand::Nav));
}

TEST(RadioVolumeAnnunciationTest, StalePeerDoesNotRefreshTimer) {
  SoftkeyController pfd;
  SoftkeyController mfd;
  FlightData d;
  MapData map;

  pfd.adjustNavVolume(+1, d);
  syncPeers(pfd, mfd);

  // X-Plane render cache: only the PFD ticks down while the MFD keeps blitting.
  for (int i = 0; i < 180; ++i) {
    pfd.update(kFrameSeconds, d, map);
    syncPeers(pfd, mfd);
  }

  EXPECT_FALSE(pfd.radioVolumeShown(RadioBand::Nav));
  EXPECT_FALSE(mfd.radioVolumeShown(RadioBand::Nav));
}

}  // namespace
}  // namespace avionics
