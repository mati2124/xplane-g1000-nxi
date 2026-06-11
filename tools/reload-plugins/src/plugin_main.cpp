// Dev helper: reload all X-Plane plugins without quitting the sim.
//
// Adds a single Plugins menu item ("Reload Plugins") bound to a command, so one
// click reloads every plugin immediately (no submenu, no dialog). After
// rebuilding xplane-avionics.xpl, choose Plugins -> Reload Plugins to load the
// new binary (enable/disable alone does not pick up code changes).

#include <cstring>

#include "XPLMDefs.h"
#include "XPLMMenus.h"
#include "XPLMPlugin.h"
#include "XPLMUtilities.h"

namespace {

constexpr const char* kCommandName = "xplaneavionics/reload_plugins";
constexpr const char* kMenuItem = "Reload Plugins";

XPLMCommandRef g_reloadCmd = nullptr;

int OnReloadCommand(XPLMCommandRef /*cmd*/, XPLMCommandPhase phase,
                    void* /*ref*/) {
  if (phase == xplm_CommandBegin) XPLMReloadPlugins();
  return 0;
}

}  // namespace

PLUGIN_API int XPluginStart(char* outName, char* outSig, char* outDesc) {
  std::strcpy(outName, "Reload Plugins");
  std::strcpy(outSig, "com.andrewmiller.xplaneavionics.reload");
  std::strcpy(outDesc, "Reload all plugins without restarting X-Plane.");

  g_reloadCmd = XPLMCreateCommand(kCommandName, "Reload all X-Plane plugins");
  XPLMRegisterCommandHandler(g_reloadCmd, &OnReloadCommand, /*before=*/1,
                             nullptr);
  // A single item directly in the Plugins menu: one click runs the command.
  XPLMAppendMenuItemWithCommand(XPLMFindPluginsMenu(), kMenuItem, g_reloadCmd);
  return 1;
}

PLUGIN_API void XPluginStop(void) {
  if (g_reloadCmd != nullptr) {
    XPLMUnregisterCommandHandler(g_reloadCmd, &OnReloadCommand, /*before=*/1,
                                 nullptr);
    g_reloadCmd = nullptr;
  }
}

PLUGIN_API int XPluginEnable(void) { return 1; }
PLUGIN_API void XPluginDisable(void) {}
PLUGIN_API void XPluginReceiveMessage(XPLMPluginID, int, void*) {}
