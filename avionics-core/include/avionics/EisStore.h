#pragma once

#include <atomic>
#include <cstdint>
#include <string>
#include <thread>

#include "avionics/Eis.h"

namespace avionics {

// Loads the per-aircraft EIS layout from a plain-text file (see parseEisText).
// Each aircraft can ship its own g1000_eis.txt beside the .acf so the engine
// strip changes with the airframe without rebuilding the avionics.
//
// File selection (first match wins):
//   - Non-empty selector: absolute/relative path to an EIS file.
//   - aircraftAcfRelativePath set: g1000_eis.txt and <acf_stem>_eis.txt next
//     to the loaded .acf (X-Plane plugin resolves acf_relative_path).
//   - Empty selector + no aircraft path: AVIONICS_DEFAULT_EIS bundled sample.
//
// Initial load runs on a background thread; refreshIfChanged() re-parses when
// the file mtime advances so authors can iterate without a restart.
class EisStore : public EisSource {
 public:
  explicit EisStore(std::string selector = "");
  ~EisStore() override;

  EisStore(const EisStore&) = delete;
  EisStore& operator=(const EisStore&) = delete;

  bool ready() const override { return loaded_.load(std::memory_order_acquire); }
  const EisLayout& layout() const override { return layout_; }
  const std::string& sourcePath() const { return sourcePath_; }

  void setAircraftAcfRelativePath(const std::string& acfRelativePath) override;
  void setAircraftIdentity(const std::string& icaoType,
                           const std::string& acfRelativePath) override;

  void refreshIfChanged() override;

 private:
  void loadOnBackgroundThread();
  void resolveAndLoad(const std::string& acfRelativePath);
  std::string resolvePath(const std::string& acfRelativePath) const;

  std::string selector_;
  std::string aircraftIcao_;
  std::string aircraftAcfRelativePath_;
  EisLayout layout_;
  std::string sourcePath_;
  std::int64_t lastMtimeNs_ = 0;

  std::atomic<bool> loaded_{false};
  std::thread thread_;
};

}  // namespace avionics
