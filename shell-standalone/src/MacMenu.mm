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

}  // namespace avionics
