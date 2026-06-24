import { spawn } from "node:child_process";
import WebSocket from "ws";
import { runKfmyRnav05LongFinal } from "./scenarios/kfmy-rnav05-long-final.js";

const PLUGIN_UUID = "com.andy.xplane-nxi";
const ACTION_RESTART = "com.andy.xplane-nxi.restart";
const ACTION_KFMY_FINAL = "com.andy.xplane-nxi.kfmy-rnav05-final";
const DEFAULT_SCRIPT =
  "C:\\Users\\Andy\\xplane-g1000-nxi\\scripts\\restart-xplane-release.ps1";

const ACTION_UI = {
  [ACTION_RESTART]: {
    idleTitle: "NXI\nRESTART",
    busyTitle: "BUILD...",
    busyMs: 4000,
  },
  [ACTION_KFMY_FINAL]: {
    idleTitle: "KFMY\nRNAV05",
    busyTitle: "SETUP...",
    busyMs: 2500,
  },
};

function parseArgs(argv) {
  const args = {};
  for (let i = 2; i < argv.length; i += 2) {
    const key = argv[i]?.replace(/^-/, "");
    args[key] = argv[i + 1];
  }
  return args;
}

function send(ws, event, context, payload = {}) {
  ws.send(JSON.stringify({ event, context, payload }));
}

function runRestart(scriptPath) {
  spawn(
    "powershell.exe",
    ["-NoProfile", "-ExecutionPolicy", "Bypass", "-File", scriptPath],
    { detached: true, stdio: "ignore", windowsHide: true }
  ).unref();
}

function setBusyTitle(ws, context, actionUuid, busy) {
  const ui = ACTION_UI[actionUuid];
  if (!ui) return;
  send(ws, "setTitle", context, {
    title: busy ? ui.busyTitle : ui.idleTitle,
    target: 0,
  });
}

const args = parseArgs(process.argv);
const port = args.port;
const pluginUUID = args.pluginUUID;
const registerEvent = args.registerEvent;

if (!port || !pluginUUID || !registerEvent) {
  console.error("Missing Stream Deck launch arguments", args);
  process.exit(1);
}

const ws = new WebSocket(`ws://127.0.0.1:${port}`);

ws.on("open", () => {
  ws.send(JSON.stringify({ event: registerEvent, uuid: pluginUUID }));
});

ws.on("message", (data) => {
  let message;
  try {
    message = JSON.parse(String(data));
  } catch {
    return;
  }

  const { event, action, context, payload } = message;
  const ui = ACTION_UI[action];
  if (!ui) {
    return;
  }

  if (event === "willAppear") {
    send(ws, "setTitle", context, { title: ui.idleTitle, target: 0 });
    return;
  }

  if (event !== "keyDown") {
    return;
  }

  if (action === ACTION_RESTART) {
    const scriptPath = payload?.settings?.scriptPath || DEFAULT_SCRIPT;
    setBusyTitle(ws, context, action, true);
    runRestart(scriptPath);
    setTimeout(() => {
      setBusyTitle(ws, context, action, false);
    }, ui.busyMs);
    return;
  }

  if (action === ACTION_KFMY_FINAL) {
    setBusyTitle(ws, context, action, true);
    runKfmyRnav05LongFinal({
      host: payload?.settings?.xplaneHost,
      port: payload?.settings?.xplanePort
        ? Number(payload.settings.xplanePort)
        : undefined,
      aircraftPath: payload?.settings?.aircraftPath,
      iasKts: payload?.settings?.iasKts
        ? Number(payload.settings.iasKts)
        : undefined,
    })
      .then((result) => {
        console.log("KFMY RNAV05 long final:", result);
      })
      .catch((err) => {
        console.error("KFMY RNAV05 long final failed:", err);
      })
      .finally(() => {
        setTimeout(() => {
          setBusyTitle(ws, context, action, false);
        }, ui.busyMs);
      });
  }
});

ws.on("error", (err) => {
  console.error("Stream Deck websocket error:", err);
});
