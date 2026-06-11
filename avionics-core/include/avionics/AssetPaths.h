#pragma once

#include <string>

// Runtime resolution of bundled assets (fonts, EIS samples, map land data,
// checklists). During development the build bakes in absolute paths into the
// source tree (the various AVIONICS_*_DIR / AVIONICS_DEFAULT_* macros), which
// only exist on the build machine. For a distributed build those paths are
// gone, so each shell registers one or more search directories derived from the
// running binary's location and the assets are looked up there first, falling
// back to the compile-time path when nothing matches (so running straight from
// the build tree still works).
namespace avionics {
namespace assets {

// Register a directory to search for bundled assets. Directories are searched
// in registration order, so add the most specific/highest-priority location
// first. Duplicate and empty directories are ignored. Safe to call from the
// shell's startup before any rendering or background loading begins.
void addSearchDir(const std::string& dir);

// Resolve a bundled asset given its path relative to a search directory
// (e.g. "fonts/Roboto-Regular.ttf"). Returns the first existing match among the
// registered search directories. If nothing matches, returns `devFallback`
// unchanged (typically a compile-time absolute path used during development);
// callers that probe for file existence handle a missing/empty result.
std::string resolve(const std::string& relativePath,
                    const std::string& devFallback = std::string());

}  // namespace assets
}  // namespace avionics
