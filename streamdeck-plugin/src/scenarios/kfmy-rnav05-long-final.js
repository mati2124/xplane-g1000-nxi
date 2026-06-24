import {
  createUdpSender,
  feetToMeters,
  knotsToMetersPerSecond,
  pointFrom,
} from "../xplane.js";

// KFMY RNAV (GPS) RWY 05 — published threshold and final approach course from
// X-Plane CIFP (R05). Long final is placed ~8 NM from the threshold on the
// extended centerline at the FAF-ish altitude band.
const KFMY_RW05_THRESHOLD_LAT = 26 + 34 / 60 + 51.08 / 3600;
const KFMY_RW05_THRESHOLD_LON = -(81 + 52 / 60 + 12.19 / 3600);
const KFMY_RW05_TRUE_HEADING_DEG = 51.0;
const KFMY_RW05_ELEVATION_FT = 13;

const LONG_FINAL_DISTANCE_NM = 8.0;
const LONG_FINAL_ALTITUDE_FT = 2200;
const TARGET_IAS_KTS = 90;

const DEFAULT_C172_G1000_PATH =
  "Aircraft/Laminar Research/Cessna 172 SP/Cessna_172SP_G1000.acf";

export async function runKfmyRnav05LongFinal({
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

    await xp.sendDref("sim/flightmodel/engine/ENGN_thro[0]", 0.62);
    await xp.sendDref("sim/flightmodel2/engines/engine_is_burning_fuel[0]", 1);
    await xp.sendDref("sim/flightmodel2/engines/starter_is_running[0]", 0);
    await xp.sendDref("sim/cockpit2/fuel/fuel_tank_selector", 3);
    await xp.sendDref("sim/time/paused", 1);
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
