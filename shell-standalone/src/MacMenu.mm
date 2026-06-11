#include "MacMenu.h"

#import <Cocoa/Cocoa.h>

namespace avionics {

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
