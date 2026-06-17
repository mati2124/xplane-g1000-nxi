#include "MacMenu.h"

#import <Cocoa/Cocoa.h>

#include <utility>

// Owns the Debug menu's items and forwards selections to the C++ callbacks.
// Held alive for the process lifetime by the file-static pointer below.
@interface AvDebugMenuController : NSObject
@property(nonatomic, assign) avionics::DebugSourceCallback sourceCallback;
@property(nonatomic, assign) avionics::DebugPowerCallback powerCallback;
@property(nonatomic, assign) avionics::DebugCasCallback casCallback;
@property(nonatomic, assign) void* context;
@property(nonatomic, strong) NSMenuItem* xplaneItem;
@property(nonatomic, strong) NSMenuItem* demoGroundItem;
@property(nonatomic, strong) NSMenuItem* demoFlyingItem;
@property(nonatomic, strong) NSMenuItem* demoTurbulenceItem;
@property(nonatomic, strong) NSMenuItem* masterItem;
@property(nonatomic, strong) NSMenuItem* avionicsItem;
@property(nonatomic, strong) NSMenuItem* casMessagesItem;
- (void)selectXPlane:(id)sender;
- (void)selectDemoGround:(id)sender;
- (void)selectDemoFlying:(id)sender;
- (void)selectDemoTurbulence:(id)sender;
- (void)toggleMaster:(id)sender;
- (void)toggleAvionics:(id)sender;
- (void)toggleCasMessages:(id)sender;
- (void)syncSource:(avionics::DebugDataSource)selection;
- (void)syncPowerMaster:(BOOL)masterOn avionics:(BOOL)avionicsOn;
- (void)syncCasMessages:(BOOL)enabled;
@end

static AvDebugMenuController* gDebugController = nil;

@implementation AvDebugMenuController

- (void)fireSource:(avionics::DebugDataSource)selection {
  if (self.sourceCallback) self.sourceCallback(self.context, selection);
}

- (void)selectXPlane:(id)sender {
  [self fireSource:avionics::DebugDataSource::XPlane];
}

- (void)selectDemoGround:(id)sender {
  [self fireSource:avionics::DebugDataSource::DemoGround];
}

- (void)selectDemoFlying:(id)sender {
  [self fireSource:avionics::DebugDataSource::DemoFlying];
}

- (void)selectDemoTurbulence:(id)sender {
  [self fireSource:avionics::DebugDataSource::DemoTurbulence];
}

// AppKit does not flip a menu item's checkmark on click, so compute the new
// power state as the inverse of the clicked item and report both switches.
- (void)toggleMaster:(id)sender {
  const BOOL master = (self.masterItem.state != NSControlStateValueOn);
  const BOOL avionics = (self.avionicsItem.state == NSControlStateValueOn);
  if (self.powerCallback) self.powerCallback(self.context, master, avionics);
}

- (void)toggleAvionics:(id)sender {
  const BOOL master = (self.masterItem.state == NSControlStateValueOn);
  const BOOL avionics = (self.avionicsItem.state != NSControlStateValueOn);
  if (self.powerCallback) self.powerCallback(self.context, master, avionics);
}

// Report the inverse of the clicked item's current checkmark as the new state.
- (void)toggleCasMessages:(id)sender {
  const BOOL enabled = (self.casMessagesItem.state != NSControlStateValueOn);
  if (self.casCallback) self.casCallback(self.context, enabled);
}

- (void)syncSource:(avionics::DebugDataSource)selection {
  self.xplaneItem.state = selection == avionics::DebugDataSource::XPlane
                              ? NSControlStateValueOn
                              : NSControlStateValueOff;
  self.demoGroundItem.state = selection == avionics::DebugDataSource::DemoGround
                                  ? NSControlStateValueOn
                                  : NSControlStateValueOff;
  self.demoFlyingItem.state = selection == avionics::DebugDataSource::DemoFlying
                                  ? NSControlStateValueOn
                                  : NSControlStateValueOff;
  self.demoTurbulenceItem.state =
      selection == avionics::DebugDataSource::DemoTurbulence
          ? NSControlStateValueOn
          : NSControlStateValueOff;
}

- (void)syncPowerMaster:(BOOL)masterOn avionics:(BOOL)avionicsOn {
  self.masterItem.state =
      masterOn ? NSControlStateValueOn : NSControlStateValueOff;
  self.avionicsItem.state =
      avionicsOn ? NSControlStateValueOn : NSControlStateValueOff;
}

- (void)syncCasMessages:(BOOL)enabled {
  self.casMessagesItem.state =
      enabled ? NSControlStateValueOn : NSControlStateValueOff;
}

@end

namespace avionics {

void InstallDebugMenu(const DebugMenuConfig& config) {
  @autoreleasepool {
    NSMenu* mainMenu = [NSApp mainMenu];
    if (mainMenu == nil) {
      mainMenu = [[NSMenu alloc] init];
      [NSApp setMainMenu:mainMenu];
    }

    gDebugController = [[AvDebugMenuController alloc] init];
    gDebugController.sourceCallback = config.onSource;
    gDebugController.powerCallback = config.onPower;
    gDebugController.casCallback = config.onCas;
    gDebugController.context = config.context;

    NSMenuItem* debugMenuItem = [[NSMenuItem alloc] init];
    NSMenu* debugMenu = [[NSMenu alloc] initWithTitle:@"Debug"];
    [debugMenuItem setSubmenu:debugMenu];

    NSMenuItem* xplaneItem =
        [[NSMenuItem alloc] initWithTitle:@"X-Plane (Live)"
                                   action:@selector(selectXPlane:)
                            keyEquivalent:@"1"];
    [xplaneItem setTarget:gDebugController];
    NSMenuItem* demoGroundItem =
        [[NSMenuItem alloc] initWithTitle:@"Demo: On Ground (KFMY)"
                                   action:@selector(selectDemoGround:)
                            keyEquivalent:@"2"];
    [demoGroundItem setTarget:gDebugController];
    NSMenuItem* demoFlyingItem =
        [[NSMenuItem alloc] initWithTitle:@"Demo: Flying"
                                   action:@selector(selectDemoFlying:)
                            keyEquivalent:@"3"];
    [demoFlyingItem setTarget:gDebugController];
    NSMenuItem* demoTurbulenceItem =
        [[NSMenuItem alloc] initWithTitle:@"Demo: Flying + Turbulence"
                                   action:@selector(selectDemoTurbulence:)
                            keyEquivalent:@"4"];
    [demoTurbulenceItem setTarget:gDebugController];

    [debugMenu addItem:xplaneItem];
    [debugMenu addItem:demoGroundItem];
    [debugMenu addItem:demoFlyingItem];
    [debugMenu addItem:demoTurbulenceItem];

    [debugMenu addItem:[NSMenuItem separatorItem]];

    // Power switches apply to the demo feed (the live feed's power comes from
    // the sim datarefs). Master gates the PFD; avionics additionally gates the
    // MFD.
    NSMenuItem* masterItem =
        [[NSMenuItem alloc] initWithTitle:@"Master Power"
                                   action:@selector(toggleMaster:)
                            keyEquivalent:@""];
    [masterItem setTarget:gDebugController];
    NSMenuItem* avionicsItem =
        [[NSMenuItem alloc] initWithTitle:@"Avionics Power"
                                   action:@selector(toggleAvionics:)
                            keyEquivalent:@""];
    [avionicsItem setTarget:gDebugController];
    [debugMenu addItem:masterItem];
    [debugMenu addItem:avionicsItem];

    [debugMenu addItem:[NSMenuItem separatorItem]];

    // Toggles the demo feed's cycling CAS annunciations (the periodic OIL
    // PRESSURE / LOW VOLTS / FUEL LOW etc. alerts) on and off. The live X-Plane
    // feed's CAS comes from the sim annunciator datarefs, so this only affects
    // the demo feed.
    NSMenuItem* casMessagesItem =
        [[NSMenuItem alloc] initWithTitle:@"Demo CAS Messages"
                                   action:@selector(toggleCasMessages:)
                            keyEquivalent:@""];
    [casMessagesItem setTarget:gDebugController];
    [debugMenu addItem:casMessagesItem];

    gDebugController.xplaneItem = xplaneItem;
    gDebugController.demoGroundItem = demoGroundItem;
    gDebugController.demoFlyingItem = demoFlyingItem;
    gDebugController.demoTurbulenceItem = demoTurbulenceItem;
    gDebugController.masterItem = masterItem;
    gDebugController.avionicsItem = avionicsItem;
    gDebugController.casMessagesItem = casMessagesItem;
    [gDebugController syncSource:config.source];
    [gDebugController syncPowerMaster:(config.masterPowerOn ? YES : NO)
                            avionics:(config.avionicsPowerOn ? YES : NO)];
    [gDebugController syncCasMessages:(config.casMessagesOn ? YES : NO)];

    [mainMenu addItem:debugMenuItem];
  }
}

void SyncDebugMenuSource(DebugDataSource selection) {
  if (gDebugController != nil) [gDebugController syncSource:selection];
}

void SyncDebugMenuPower(bool masterOn, bool avionicsOn) {
  if (gDebugController != nil) {
    [gDebugController syncPowerMaster:(masterOn ? YES : NO)
                            avionics:(avionicsOn ? YES : NO)];
  }
}

void SyncDebugMenuCas(bool enabled) {
  if (gDebugController != nil) {
    [gDebugController syncCasMessages:(enabled ? YES : NO)];
  }
}

void ShowUpdateAvailableAlert(const char* currentVersion,
                              const char* latestVersion,
                              const char* primaryButtonTitle,
                              std::function<void()> onAccept) {
  NSString* const current =
      currentVersion ? [NSString stringWithUTF8String:currentVersion] : @"";
  NSString* const latest =
      latestVersion ? [NSString stringWithUTF8String:latestVersion] : @"";
  NSString* const primary = primaryButtonTitle
                                ? [NSString stringWithUTF8String:primaryButtonTitle]
                                : @"OK";

  __block std::function<void()> accept = std::move(onAccept);
  dispatch_async(dispatch_get_main_queue(), ^{
    @autoreleasepool {
      NSString* message = [NSString
          stringWithFormat:
              @"A newer G1000 NXi release is available.\n\nInstalled: %@\n"
              @"Latest: %@",
              current, latest];
      NSAlert* alert = [[NSAlert alloc] init];
      alert.messageText = @"Update Available";
      alert.informativeText = message;
      [alert addButtonWithTitle:primary];
      [alert addButtonWithTitle:@"Not Now"];
      // Borderless full-screen GLFW displays use NSMainMenuWindowLevel + 1,
      // which sits above NSModalPanelWindowLevel. Raise the alert so it is not
      // hidden behind the PFD/MFD windows.
      [NSApp activateIgnoringOtherApps:YES];
      [alert.window setLevel:NSPopUpMenuWindowLevel];
      const NSModalResponse response = [alert runModal];
      if (response == NSAlertFirstButtonReturn && accept) {
        // Run the (network/file) work off the main thread so the UI is not
        // blocked while the update downloads and applies.
        std::function<void()> work = accept;
        dispatch_async(
            dispatch_get_global_queue(QOS_CLASS_UTILITY, 0), ^{ work(); });
      }
    }
  });
}

}  // namespace avionics
