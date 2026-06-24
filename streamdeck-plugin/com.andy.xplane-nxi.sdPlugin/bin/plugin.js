import { spawn } from 'node:child_process';
import WebSocket from 'ws';
import dgram from 'node:dgram';

const DEFAULT_XPLANE_HOST = "127.0.0.1";
const DEFAULT_XPLANE_PORT = 49000;

const DREF_NAME_SIZE = 500;
const ACPR_PATH_SIZE = 150;
const CMND_NAME_SIZE = 500;

function writeLe32(buffer, offset, value) {
  buffer.writeInt32LE(value, offset);
}

function writeLeFloat(buffer, offset, value) {
  buffer.writeFloatLE(value, offset);
}

function writeLeDouble(buffer, offset, value) {
  buffer.writeDoubleLE(value, offset);
}

function writeCString(buffer, offset, maxLen, value) {
  const bytes = Buffer.from(value, "utf8");
  const len = Math.min(bytes.length, maxLen - 1);
  bytes.copy(buffer, offset, 0, len);
}

function createUdpSender(host = DEFAULT_XPLANE_HOST, port = DEFAULT_XPLANE_PORT) {
  const socket = dgram.createSocket("udp4");

  function send(buffer) {
    return new Promise((resolve, reject) => {
      socket.send(buffer, port, host, (err) => {
        if (err) reject(err);
        else resolve();
      });
    });
  }

  function close() {
    return new Promise((resolve) => {
      socket.close(() => resolve());
    });
  }

  async function sendDref(path, value) {
    const msg = Buffer.alloc(5 + 4 + DREF_NAME_SIZE);
    msg.write("DREF", 0, 4, "ascii");
    writeLeFloat(msg, 5, value);
    writeCString(msg, 9, DREF_NAME_SIZE, path);
    await send(msg);
  }

  async function sendCommand(path) {
    const msg = Buffer.alloc(5 + CMND_NAME_SIZE);
    msg.write("CMND", 0, 4, "ascii");
    writeCString(msg, 5, CMND_NAME_SIZE, path);
    await send(msg);
  }

  // ACPR: load aircraft and place it (X-Plane UDP, little-endian).
  async function sendAcpr({
    aircraftPath,
    planeIndex = 0,
    livery = 0,
    typeStart = 6, // loc_specify_lle
    latDeg,
    lonDeg,
    elevationMeters,
    headingTrueDeg,
    speedMetersPerSec,
  }) {
    const msg = Buffer.alloc(4 + 1 + 4 + ACPR_PATH_SIZE + 2 + 4 + 4 + 4 + 8 + 4 + 4 + 8 + 8 + 8 + 8 + 8);
    let offset = 0;
    msg.write("ACPR", offset, 4, "ascii");
    offset += 4;
    offset += 1; // pad byte
    writeLe32(msg, offset, planeIndex);
    offset += 4;
    writeCString(msg, offset, ACPR_PATH_SIZE, aircraftPath);
    offset += ACPR_PATH_SIZE;
    offset += 2; // struct padding
    writeLe32(msg, offset, livery);
    offset += 4;
    writeLe32(msg, offset, typeStart);
    offset += 4;
    writeLe32(msg, offset, planeIndex);
    offset += 4;
    writeCString(msg, offset, 8, "");
    offset += 8;
    writeLe32(msg, offset, 0);
    offset += 4;
    writeLe32(msg, offset, 0);
    offset += 4;
    writeLeDouble(msg, offset, latDeg);
    offset += 8;
    writeLeDouble(msg, offset, lonDeg);
    offset += 8;
    writeLeDouble(msg, offset, elevationMeters);
    offset += 8;
    writeLeDouble(msg, offset, headingTrueDeg);
    offset += 8;
    writeLeDouble(msg, offset, speedMetersPerSec);
    await send(msg);
  }

  return { send, sendDref, sendCommand, sendAcpr, close };
}

function pointFrom(latDeg, lonDeg, bearingDeg, distanceNm) {
  const earthRadiusNm = 3440.065;
  const lat1 = (latDeg * Math.PI) / 180;
  const lon1 = (lonDeg * Math.PI) / 180;
  const brng = (bearingDeg * Math.PI) / 180;
  const dist = distanceNm / earthRadiusNm;
  const lat2 = Math.asin(
    Math.sin(lat1) * Math.cos(dist) +
      Math.cos(lat1) * Math.sin(dist) * Math.cos(brng)
  );
  const lon2 =
    lon1 +
    Math.atan2(
      Math.sin(brng) * Math.sin(dist) * Math.cos(lat1),
      Math.cos(dist) - Math.sin(lat1) * Math.sin(lat2)
    );
  return {
    lat: (lat2 * 180) / Math.PI,
    lon: (lon2 * 180) / Math.PI,
  };
}

function knotsToMetersPerSecond(knots) {
  return knots * 0.514444;
}

function feetToMeters(feet) {
  return feet * 0.3048;
}

// KFMY RNAV (GPS) RWY 05 — published threshold and final approach course from
// X-Plane CIFP (R05). Long final is placed ~8 NM from the threshold on the
// extended centerline at the FAF-ish altitude band.
const KFMY_RW05_THRESHOLD_LAT = 26 + 34 / 60 + 51.08 / 3600;
const KFMY_RW05_THRESHOLD_LON = -81.87005277777777;
const KFMY_RW05_TRUE_HEADING_DEG = 51.0;
const KFMY_RW05_ELEVATION_FT = 13;

const LONG_FINAL_DISTANCE_NM = 8.0;
const LONG_FINAL_ALTITUDE_FT = 2200;
const TARGET_IAS_KTS = 90;

const DEFAULT_C172_G1000_PATH =
  "Aircraft/Laminar Research/Cessna 172 SP/Cessna_172SP_G1000.acf";

async function runKfmyRnav05LongFinal({
  host,
  port,
  aircraftPath = DEFAULT_C172_G1000_PATH,
  iasKts = TARGET_IAS_KTS,
} = {}) {
  const finalPos = pointFrom(
    KFMY_RW05_THRESHOLD_LAT,
    KFMY_RW05_THRESHOLD_LON,
    KFMY_RW05_TRUE_HEADING_DEG + 180,
    LONG_FINAL_DISTANCE_NM
  );

  const xp = createUdpSender(host, port);

  try {
    await xp.sendAcpr({
      aircraftPath,
      latDeg: finalPos.lat,
      lonDeg: finalPos.lon,
      elevationMeters: feetToMeters(KFMY_RW05_ELEVATION_FT + LONG_FINAL_ALTITUDE_FT),
      headingTrueDeg: KFMY_RW05_TRUE_HEADING_DEG,
      speedMetersPerSec: knotsToMetersPerSecond(iasKts),
    });

    // Give X-Plane a moment to load the aircraft before tweaking engine state.
    await delay(800);

    await xp.sendDref("sim/time/paused", 0);
    await xp.sendDref("sim/flightmodel/engine/ENGN_thro[0]", 0.62);
    await xp.sendDref("sim/flightmodel2/engines/engine_is_burning_fuel[0]", 1);
    await xp.sendDref("sim/flightmodel2/engines/starter_is_running[0]", 0);
    await xp.sendDref("sim/cockpit2/fuel/fuel_tank_selector", 3); // BOTH
  } finally {
    await xp.close();
  }

  return {
    lat: finalPos.lat,
    lon: finalPos.lon,
    altitudeFt: LONG_FINAL_ALTITUDE_FT,
    headingTrueDeg: KFMY_RW05_TRUE_HEADING_DEG,
    iasKts,
    aircraftPath,
  };
}

function delay(ms) {
  return new Promise((resolve) => setTimeout(resolve, ms));
}

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
