#include "MacMenu.h"

#import <Cocoa/Cocoa.h>

#include <utility>

namespace avionics {

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
