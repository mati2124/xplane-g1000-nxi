#include "MacMenu.h"

#import <Cocoa/Cocoa.h>

// Owns the two menu items and forwards selections to the C++ callback. Held
// alive for the lifetime of the process by the file-static pointer below.
@interface AvDataSourceMenuController : NSObject
@property(nonatomic, assign) avionics::DataSourceMenuCallback callback;
@property(nonatomic, assign)
    avionics::TurbulenceMenuToggleCallback turbulenceCallback;
@property(nonatomic, assign) void* context;
@property(nonatomic, strong) NSMenuItem* mockItem;
@property(nonatomic, strong) NSMenuItem* mockGroundItem;
@property(nonatomic, strong) NSMenuItem* xplaneItem;
@property(nonatomic, strong) NSMenuItem* turbulenceItem;
- (void)selectMock:(id)sender;
- (void)selectMockGround:(id)sender;
- (void)selectXPlane:(id)sender;
- (void)toggleTurbulence:(id)sender;
- (void)syncSelection:(avionics::DataSourceSelection)selection;
@end

static AvDataSourceMenuController* gController = nil;

// Owns the View menu's checkable items and forwards toggles to the C++
// callbacks. Held alive for the process lifetime by the static pointer below.
@interface AvViewMenuController : NSObject
@property(nonatomic, assign) avionics::ViewMenuToggleCallback bezelCallback;
@property(nonatomic, assign)
    avionics::ViewMenuToggleCallback windowChromeCallback;
@property(nonatomic, assign)
    avionics::ViewMenuToggleCallback alwaysOnTopCallback;
@property(nonatomic, assign)
    avionics::ViewMenuToggleCallback rememberPosCallback;
@property(nonatomic, assign) void* context;
@property(nonatomic, strong) NSMenuItem* showBezelItem;
@property(nonatomic, strong) NSMenuItem* showWindowChromeItem;
@property(nonatomic, strong) NSMenuItem* alwaysOnTopItem;
@property(nonatomic, strong) NSMenuItem* rememberPosItem;
- (void)toggleBezel:(id)sender;
- (void)toggleWindowChrome:(id)sender;
- (void)toggleAlwaysOnTop:(id)sender;
- (void)toggleRememberPos:(id)sender;
- (void)syncBezelSelection:(BOOL)showBezel;
@end

static AvViewMenuController* gViewController = nil;

// AppKit does not flip an NSMenuItem's checkmark on click, so compute the new
// state as the inverse of the current one and apply it ourselves.
static BOOL FlipMenuItemState(NSMenuItem* item) {
  const BOOL enabled = (item.state != NSControlStateValueOn);
  item.state = enabled ? NSControlStateValueOn : NSControlStateValueOff;
  return enabled;
}

@implementation AvDataSourceMenuController

- (void)selectMock:(id)sender {
  if (self.callback) {
    self.callback(self.context, avionics::DataSourceSelection::MockFlying);
  }
}

- (void)selectMockGround:(id)sender {
  if (self.callback) {
    self.callback(self.context, avionics::DataSourceSelection::MockGround);
  }
}

- (void)selectXPlane:(id)sender {
  if (self.callback) {
    self.callback(self.context, avionics::DataSourceSelection::XPlane);
  }
}

- (void)toggleTurbulence:(id)sender {
  const BOOL enabled = FlipMenuItemState(self.turbulenceItem);
  if (self.turbulenceCallback) {
    self.turbulenceCallback(self.context, enabled ? true : false);
  }
}

- (void)syncSelection:(avionics::DataSourceSelection)selection {
  self.mockItem.state =
      selection == avionics::DataSourceSelection::MockFlying
          ? NSControlStateValueOn
          : NSControlStateValueOff;
  self.mockGroundItem.state =
      selection == avionics::DataSourceSelection::MockGround
          ? NSControlStateValueOn
          : NSControlStateValueOff;
  self.xplaneItem.state =
      selection == avionics::DataSourceSelection::XPlane
          ? NSControlStateValueOn
          : NSControlStateValueOff;
}

@end

@implementation AvViewMenuController

- (void)toggleBezel:(id)sender {
  const BOOL enabled = FlipMenuItemState(self.showBezelItem);
  if (self.bezelCallback) {
    self.bezelCallback(self.context, enabled ? true : false);
  }
}

- (void)toggleWindowChrome:(id)sender {
  const BOOL enabled = FlipMenuItemState(self.showWindowChromeItem);
  if (self.windowChromeCallback) {
    self.windowChromeCallback(self.context, enabled ? true : false);
  }
}

- (void)toggleAlwaysOnTop:(id)sender {
  const BOOL enabled = FlipMenuItemState(self.alwaysOnTopItem);
  if (self.alwaysOnTopCallback) {
    self.alwaysOnTopCallback(self.context, enabled ? true : false);
  }
}

- (void)toggleRememberPos:(id)sender {
  const BOOL enabled = FlipMenuItemState(self.rememberPosItem);
  if (self.rememberPosCallback) {
    self.rememberPosCallback(self.context, enabled ? true : false);
  }
}

- (void)syncBezelSelection:(BOOL)showBezel {
  self.showBezelItem.state =
      showBezel ? NSControlStateValueOn : NSControlStateValueOff;
}

@end

namespace avionics {

void InstallDataSourceMenu(DataSourceSelection initialSelection,
                           bool initiallyTurbulent,
                           DataSourceMenuCallback sourceCallback,
                           TurbulenceMenuToggleCallback turbulenceCallback,
                           void* context) {
  @autoreleasepool {
    NSMenu* mainMenu = [NSApp mainMenu];
    if (mainMenu == nil) {
      mainMenu = [[NSMenu alloc] init];
      [NSApp setMainMenu:mainMenu];
    }

    gController = [[AvDataSourceMenuController alloc] init];
    gController.callback = sourceCallback;
    gController.turbulenceCallback = turbulenceCallback;
    gController.context = context;

    NSMenuItem* dataMenuItem = [[NSMenuItem alloc] init];
    NSMenu* dataMenu = [[NSMenu alloc] initWithTitle:@"Data Source"];
    [dataMenuItem setSubmenu:dataMenu];

    NSMenuItem* mockItem =
        [[NSMenuItem alloc] initWithTitle:@"Mock Data (Flying)"
                                   action:@selector(selectMock:)
                            keyEquivalent:@"1"];
    [mockItem setTarget:gController];

    NSMenuItem* mockGroundItem =
        [[NSMenuItem alloc] initWithTitle:@"Mock Data (On Ground at KFMY)"
                                   action:@selector(selectMockGround:)
                            keyEquivalent:@"2"];
    [mockGroundItem setTarget:gController];

    NSMenuItem* xplaneItem =
        [[NSMenuItem alloc] initWithTitle:@"X-Plane"
                                   action:@selector(selectXPlane:)
                            keyEquivalent:@"3"];
    [xplaneItem setTarget:gController];

    [dataMenu addItem:mockItem];
    [dataMenu addItem:mockGroundItem];
    [dataMenu addItem:xplaneItem];

    [dataMenu addItem:[NSMenuItem separatorItem]];

    NSMenuItem* turbulenceItem =
        [[NSMenuItem alloc] initWithTitle:@"Simulate Turbulence"
                                   action:@selector(toggleTurbulence:)
                            keyEquivalent:@""];
    [turbulenceItem setTarget:gController];
    turbulenceItem.state =
        initiallyTurbulent ? NSControlStateValueOn : NSControlStateValueOff;
    [dataMenu addItem:turbulenceItem];

    gController.mockItem = mockItem;
    gController.mockGroundItem = mockGroundItem;
    gController.xplaneItem = xplaneItem;
    gController.turbulenceItem = turbulenceItem;
    [gController syncSelection:initialSelection];

    [mainMenu addItem:dataMenuItem];
  }
}

void SetDataSourceMenuSelection(DataSourceSelection selection) {
  if (gController != nil) {
    [gController syncSelection:selection];
  }
}

void InstallViewMenu(const ViewMenuConfig& config) {
  @autoreleasepool {
    NSMenu* mainMenu = [NSApp mainMenu];
    if (mainMenu == nil) {
      mainMenu = [[NSMenu alloc] init];
      [NSApp setMainMenu:mainMenu];
    }

    gViewController = [[AvViewMenuController alloc] init];
    gViewController.bezelCallback = config.onToggleBezel;
    gViewController.windowChromeCallback = config.onToggleWindowChrome;
    gViewController.alwaysOnTopCallback = config.onToggleAlwaysOnTop;
    gViewController.rememberPosCallback = config.onToggleRememberWindowPos;
    gViewController.context = config.context;

    NSMenuItem* viewMenuItem = [[NSMenuItem alloc] init];
    NSMenu* viewMenu = [[NSMenu alloc] initWithTitle:@"View"];
    [viewMenuItem setSubmenu:viewMenu];

    NSMenuItem* showBezelItem =
        [[NSMenuItem alloc] initWithTitle:@"Show Bezel Keys"
                                   action:@selector(toggleBezel:)
                            keyEquivalent:@"b"];
    [showBezelItem setTarget:gViewController];
    [viewMenu addItem:showBezelItem];

    NSMenuItem* showWindowChromeItem =
        [[NSMenuItem alloc] initWithTitle:@"Show Window Title Bar"
                                   action:@selector(toggleWindowChrome:)
                            keyEquivalent:@"t"];
    [showWindowChromeItem setTarget:gViewController];
    [viewMenu addItem:showWindowChromeItem];

    NSMenuItem* alwaysOnTopItem =
        [[NSMenuItem alloc] initWithTitle:@"Always on Top"
                                   action:@selector(toggleAlwaysOnTop:)
                            keyEquivalent:@""];
    [alwaysOnTopItem setTarget:gViewController];
    [viewMenu addItem:alwaysOnTopItem];

    [viewMenu addItem:[NSMenuItem separatorItem]];

    NSMenuItem* rememberPosItem =
        [[NSMenuItem alloc] initWithTitle:@"Remember Window Position"
                                   action:@selector(toggleRememberPos:)
                            keyEquivalent:@""];
    [rememberPosItem setTarget:gViewController];
    [viewMenu addItem:rememberPosItem];

    gViewController.showBezelItem = showBezelItem;
    gViewController.showWindowChromeItem = showWindowChromeItem;
    gViewController.alwaysOnTopItem = alwaysOnTopItem;
    gViewController.rememberPosItem = rememberPosItem;
    [gViewController syncBezelSelection:(config.showBezel ? YES : NO)];
    showWindowChromeItem.state = config.showWindowChrome
                                     ? NSControlStateValueOn
                                     : NSControlStateValueOff;
    alwaysOnTopItem.state = config.alwaysOnTop ? NSControlStateValueOn
                                               : NSControlStateValueOff;
    rememberPosItem.state = config.rememberWindowPos ? NSControlStateValueOn
                                                     : NSControlStateValueOff;

    [mainMenu addItem:viewMenuItem];
  }
}

void SetBezelVisibilityMenuSelection(bool showBezel) {
  if (gViewController != nil) {
    [gViewController syncBezelSelection:(showBezel ? YES : NO)];
  }
}

void ShowUpdateAvailableAlert(const char* currentVersion,
                              const char* latestVersion,
                              const char* releaseUrl) {
  NSString* const current =
      currentVersion ? [NSString stringWithUTF8String:currentVersion] : @"";
  NSString* const latest =
      latestVersion ? [NSString stringWithUTF8String:latestVersion] : @"";
  NSString* const url =
      releaseUrl ? [NSString stringWithUTF8String:releaseUrl] : @"";

  dispatch_async(dispatch_get_main_queue(), ^{
    @autoreleasepool {
      NSString* message = [NSString
          stringWithFormat:
              @"A newer G1000 NXi release is available.\n\nInstalled: %@\n"
              @"Latest: %@\n\nOpen the release page in your browser to "
              @"download it.",
              current, latest];
      NSAlert* alert = [[NSAlert alloc] init];
      alert.messageText = @"Update Available";
      alert.informativeText = message;
      [alert addButtonWithTitle:@"Open Download Page"];
      [alert addButtonWithTitle:@"Not Now"];
      const NSModalResponse response = [alert runModal];
      if (response == NSAlertFirstButtonReturn && url.length > 0) {
        NSURL* const release = [NSURL URLWithString:url];
        if (release != nil) {
          [[NSWorkspace sharedWorkspace] openURL:release];
        }
      }
    }
  });
}

}  // namespace avionics
