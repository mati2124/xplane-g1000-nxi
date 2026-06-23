import { spawn } from "node:child_process";
import WebSocket from "ws";

const PLUGIN_UUID = "com.andy.xplane-nxi";
const ACTION_UUID = "com.andy.xplane-nxi.restart";
const DEFAULT_SCRIPT =
  "C:\\Users\\Andy\\xplane-g1000-nxi\\scripts\\restart-xplane-release.ps1";

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
  if (action !== ACTION_UUID) {
    return;
  }

  if (event === "willAppear") {
    send(ws, "setTitle", context, { title: "NXI\nRESTART", target: 0 });
    return;
  }

  if (event === "keyDown") {
    const scriptPath = payload?.settings?.scriptPath || DEFAULT_SCRIPT;
    send(ws, "setTitle", context, { title: "BUILD...", target: 0 });
    runRestart(scriptPath);
    setTimeout(() => {
      send(ws, "setTitle", context, { title: "NXI\nRESTART", target: 0 });
    }, 4000);
  }
});

ws.on("error", (err) => {
  console.error("Stream Deck websocket error:", err);
});
