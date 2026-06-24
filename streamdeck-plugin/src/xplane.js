import dgram from "node:dgram";

export const DEFAULT_XPLANE_HOST = "127.0.0.1";
export const DEFAULT_XPLANE_PORT = 49000;

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

export function createUdpSender(host = DEFAULT_XPLANE_HOST, port = DEFAULT_XPLANE_PORT) {
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

export function pointFrom(latDeg, lonDeg, bearingDeg, distanceNm) {
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

export function knotsToMetersPerSecond(knots) {
  return knots * 0.514444;
}

export function feetToMeters(feet) {
  return feet * 0.3048;
}
