#pragma once

namespace avionics {

// Query GitHub for a newer release on a background thread and surface a
// platform-appropriate notice when one exists. No-op when AVIONICS_SKIP_UPDATE_CHECK
// is set in the environment.
void startUpdateCheckOnLaunch();

#if defined(_WIN32)
// HWND values for the PFD/MFD GLFW windows so update dialogs appear above the
// borderless full-screen displays (WS_EX_TOPMOST). Pass null for absent windows.
void setUpdateDialogOwnerWindows(void* primaryHwnd, void* secondaryHwnd);
#endif

}  // namespace avionics
