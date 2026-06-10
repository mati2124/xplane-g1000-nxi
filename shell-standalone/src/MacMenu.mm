#include "MacMenu.h"

#import <Cocoa/Cocoa.h>

// Owns the two menu items and forwards selections to the C++ callback. Held
// alive for the lifetime of the process by the file-static pointer below.
@interface AvDataSourceMenuController : NSObject
@property(nonatomic, assign) avionics::DataSourceMenuCallback callback;
@property(nonatomic, assign) void* context;
@property(nonatomic, strong) NSMenuItem* mockItem;
@property(nonatomic, strong) NSMenuItem* xplaneItem;
- (void)selectMock:(id)sender;
- (void)selectXPlane:(id)sender;
- (void)syncSelection:(BOOL)useXPlane;
@end

static AvDataSourceMenuController* gController = nil;

// Owns the single checkable "Show Bezel Keys" item and forwards toggles to the
// C++ callback. Held alive for the process lifetime by the static pointer below.
@interface AvBezelVisibilityMenuController : NSObject
@property(nonatomic, assign) avionics::BezelVisibilityMenuCallback callback;
@property(nonatomic, assign) void* context;
@property(nonatomic, strong) NSMenuItem* showItem;
- (void)toggleBezel:(id)sender;
- (void)syncSelection:(BOOL)showBezel;
@end

static AvBezelVisibilityMenuController* gBezelController = nil;

@implementation AvDataSourceMenuController

- (void)selectMock:(id)sender {
  if (self.callback) self.callback(self.context, false);
}

- (void)selectXPlane:(id)sender {
  if (self.callback) self.callback(self.context, true);
}

- (void)syncSelection:(BOOL)useXPlane {
  self.mockItem.state =
      useXPlane ? NSControlStateValueOff : NSControlStateValueOn;
  self.xplaneItem.state =
      useXPlane ? NSControlStateValueOn : NSControlStateValueOff;
}

@end

@implementation AvBezelVisibilityMenuController

- (void)toggleBezel:(id)sender {
  // AppKit does not flip an NSMenuItem's checkmark on click, so compute the new
  // state as the inverse of the current one and apply it ourselves.
  const BOOL showBezel = (self.showItem.state != NSControlStateValueOn);
  [self syncSelection:showBezel];
  if (self.callback) self.callback(self.context, showBezel ? true : false);
}

- (void)syncSelection:(BOOL)showBezel {
  self.showItem.state =
      showBezel ? NSControlStateValueOn : NSControlStateValueOff;
}

@end

namespace avionics {

void InstallDataSourceMenu(bool initiallyXPlane, DataSourceMenuCallback callback,
                           void* context) {
  @autoreleasepool {
    NSMenu* mainMenu = [NSApp mainMenu];
    if (mainMenu == nil) {
      mainMenu = [[NSMenu alloc] init];
      [NSApp setMainMenu:mainMenu];
    }

    gController = [[AvDataSourceMenuController alloc] init];
    gController.callback = callback;
    gController.context = context;

    NSMenuItem* dataMenuItem = [[NSMenuItem alloc] init];
    NSMenu* dataMenu = [[NSMenu alloc] initWithTitle:@"Data Source"];
    [dataMenuItem setSubmenu:dataMenu];

    NSMenuItem* mockItem =
        [[NSMenuItem alloc] initWithTitle:@"Mock Data"
                                   action:@selector(selectMock:)
                            keyEquivalent:@"1"];
    [mockItem setTarget:gController];

    NSMenuItem* xplaneItem =
        [[NSMenuItem alloc] initWithTitle:@"X-Plane"
                                   action:@selector(selectXPlane:)
                            keyEquivalent:@"2"];
    [xplaneItem setTarget:gController];

    [dataMenu addItem:mockItem];
    [dataMenu addItem:xplaneItem];

    gController.mockItem = mockItem;
    gController.xplaneItem = xplaneItem;
    [gController syncSelection:(initiallyXPlane ? YES : NO)];

    [mainMenu addItem:dataMenuItem];
  }
}

void SetDataSourceMenuSelection(bool useXPlane) {
  if (gController != nil) {
    [gController syncSelection:(useXPlane ? YES : NO)];
  }
}

void InstallBezelVisibilityMenu(bool initiallyShow,
                                BezelVisibilityMenuCallback callback,
                                void* context) {
  @autoreleasepool {
    NSMenu* mainMenu = [NSApp mainMenu];
    if (mainMenu == nil) {
      mainMenu = [[NSMenu alloc] init];
      [NSApp setMainMenu:mainMenu];
    }

    gBezelController = [[AvBezelVisibilityMenuController alloc] init];
    gBezelController.callback = callback;
    gBezelController.context = context;

    NSMenuItem* viewMenuItem = [[NSMenuItem alloc] init];
    NSMenu* viewMenu = [[NSMenu alloc] initWithTitle:@"View"];
    [viewMenuItem setSubmenu:viewMenu];

    NSMenuItem* showItem =
        [[NSMenuItem alloc] initWithTitle:@"Show Bezel Keys"
                                   action:@selector(toggleBezel:)
                            keyEquivalent:@"b"];
    [showItem setTarget:gBezelController];
    [viewMenu addItem:showItem];

    gBezelController.showItem = showItem;
    [gBezelController syncSelection:(initiallyShow ? YES : NO)];

    [mainMenu addItem:viewMenuItem];
  }
}

void SetBezelVisibilityMenuSelection(bool showBezel) {
  if (gBezelController != nil) {
    [gBezelController syncSelection:(showBezel ? YES : NO)];
  }
}

}  // namespace avionics
