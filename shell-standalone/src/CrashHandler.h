#pragma once

// Installs a process-wide last-chance handler that writes a symbolized stack
// trace to "crash_log.txt" beside the executable before the process dies.
//
// The standalone is a Windows GUI app (no console), so an access violation
// otherwise vanishes with no diagnostics. This captures the faulting thread's
// call stack (including crashes that surface inside the GPU driver) so we can
// see exactly where it died. No-op on non-Windows builds.
namespace avionics {
void InstallCrashHandler();

// Records a short "what was happening" string included in the crash log. The
// render loop updates this each frame (current page/state) so a deferred GPU
// crash records which screen was being drawn. Safe to call every frame.
void SetCrashBreadcrumb(const char* text);
}  // namespace avionics
