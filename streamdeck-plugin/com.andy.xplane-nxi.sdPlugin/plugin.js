const ACTION_UUID = "com.andy.xplane-nxi.restart";
const RESTART_URL = "nxirestart://run";

const restartAction = new Action(ACTION_UUID);

restartAction.onWillAppear(({ context }) => {
  $SD.setTitle(context, "NXI\nRESTART");
});

restartAction.onKeyDown(({ context }) => {
  $SD.setTitle(context, "BUILD...");
  $SD.openUrl(RESTART_URL);
  setTimeout(() => {
    $SD.setTitle(context, "NXI\nRESTART");
  }, 4000);
});
