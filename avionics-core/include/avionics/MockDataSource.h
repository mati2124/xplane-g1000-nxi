#pragma once

#include "avionics/DataSource.h"

namespace avionics {

// Animated, dependency-free data source for bring-up and tests: lets the
// standalone shell render something believable before the live X-Plane bridge
// is wired in.
class MockDataSource : public DataSource {
 public:
  void update(double dtSeconds) override;
  const FlightData& snapshot() const override { return data_; }
  const MapData& mapSnapshot() const override { return map_; }

 private:
  FlightData data_;
  MapData map_;
  double elapsedSeconds_ = 0.0;
};

}  // namespace avionics
