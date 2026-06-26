#pragma once

#include <cstddef>
#include <cstdint>
#include <cstring>
#include <string>
#include <utility>
#include <vector>

#include "avionics/MapData.h"

// Wire protocol for the X-Plane flight-plan bridge.
//
// X-Plane exposes the active FMS flight plan only through the in-sim plugin SDK
// (XPLMGetFMSEntryInfo / XPLMSetFMSEntry*), not over the UDP RREF telemetry
// stream or the Web API. The networked standalone shell therefore cannot see or
// program an in-cockpit route on its own. The bridge closes that gap: the
// in-sim plugin (shell-xplane) reads and writes the FMS, and the standalone
// shell talks to it over a tiny UDP channel.
//
// This header is the single shared definition of that channel -- the port, the
// message tags, and the little-endian (de)serialization of routes -- so the
// plugin and the standalone client can never drift apart.
//
// Messages (each a single datagram, client <-> plugin):
//   FPLQ  client->plugin  request the active route        -> plugin replies FPLR
//   FPLR  plugin->client   the serialized active route + Direct-To display state
//   FPLS  client->plugin   program this route into the FMS -> plugin replies FPLA
//   FPLD  client->plugin   set/clear a Direct-To target (+ origin) -> plugin FPLA
//   FPLX  client->plugin   clear stored Direct-To display only (FMS untouched) -> FPLA
//   FPLG  client->plugin   set the FMS destination leg index -> plugin replies FPLA
//   FPLA  plugin->client   acknowledgement of an FPLS/FPLD/FPLG/FPLX write
namespace avionics {
namespace fpbridge {

// Default UDP port the in-sim bridge listens on. Sits just above X-Plane's
// default RREF telemetry port (49000) and clear of the Web API (8086).
constexpr std::uint16_t kDefaultPort = 49100;

// Datagram tags (4 bytes each).
constexpr char kRequestMagic[4] = {'F', 'P', 'L', 'Q'};  // request route
constexpr char kReplyMagic[4] = {'F', 'P', 'L', 'R'};    // route payload (reply)
constexpr char kSetPlanMagic[4] = {'F', 'P', 'L', 'S'};  // program route
constexpr char kSetDtoMagic[4] = {'F', 'P', 'L', 'D'};   // set/clear Direct-To
constexpr char kClearDtoDisplayMagic[4] = {'F', 'P', 'L', 'X'};  // display-only clear
constexpr char kSetActiveLegMagic[4] = {'F', 'P', 'L', 'G'};  // set FMS destination
constexpr char kAckMagic[4] = {'F', 'P', 'L', 'A'};      // write acknowledgement

// Bump when a payload layout changes; a mismatched version is ignored by the
// receiver so an old plugin/new shell (or vice versa) fails closed rather than
// mis-parsing.
constexpr std::int32_t kProtocolVersion = 3;

// Sanity caps so a malformed or hostile datagram cannot drive an unbounded
// allocation. X-Plane's FMS holds at most ~100 entries; identifiers are short.
constexpr std::int32_t kMaxLegs = 512;
constexpr std::int32_t kMaxIdLength = 64;

// Largest reply the client will receive: kMaxLegs * (lat+lon+idLen+id) plus the
// fixed header, rounded up to a comfortable buffer.
constexpr std::size_t kMaxDatagramBytes = 65536;

// Fixed sizes of the reply framing, in bytes.
constexpr std::size_t kReplyHeaderBytes = 4 /*magic*/ + 4 /*version*/ + 4 /*count*/;
constexpr std::size_t kLegFixedBytes = 8 /*lat*/ + 8 /*lon*/ + 4 /*idLen*/;
constexpr std::size_t kDirectToOriginBytes = 4 /*originValid*/ + 8 /*originLat*/ +
                                             8 /*originLon*/;

// Direct-To display state carried in FPLR replies and stored by the plugin.
struct DirectToState {
  bool active = false;
  MapLeg target;
  bool originValid = false;
  double originLat = 0.0;
  double originLon = 0.0;
};

// --- little-endian primitives (no dependency on host byte order) ---

inline void putI32(std::vector<unsigned char>& out, std::int32_t v) {
  const auto u = static_cast<std::uint32_t>(v);
  out.push_back(static_cast<unsigned char>(u & 0xFF));
  out.push_back(static_cast<unsigned char>((u >> 8) & 0xFF));
  out.push_back(static_cast<unsigned char>((u >> 16) & 0xFF));
  out.push_back(static_cast<unsigned char>((u >> 24) & 0xFF));
}

inline std::int32_t getI32(const unsigned char* p) {
  const auto u = static_cast<std::uint32_t>(p[0]) |
                 (static_cast<std::uint32_t>(p[1]) << 8) |
                 (static_cast<std::uint32_t>(p[2]) << 16) |
                 (static_cast<std::uint32_t>(p[3]) << 24);
  return static_cast<std::int32_t>(u);
}

inline void putF64(std::vector<unsigned char>& out, double v) {
  std::uint64_t bits = 0;
  std::memcpy(&bits, &v, sizeof(bits));
  for (int i = 0; i < 8; ++i) {
    out.push_back(static_cast<unsigned char>((bits >> (8 * i)) & 0xFF));
  }
}

inline double getF64(const unsigned char* p) {
  std::uint64_t bits = 0;
  for (int i = 0; i < 8; ++i) {
    bits |= static_cast<std::uint64_t>(p[i]) << (8 * i);
  }
  double v = 0.0;
  std::memcpy(&v, &bits, sizeof(v));
  return v;
}

// --- shared primitives ---

inline bool hasMagic(const unsigned char* data, std::size_t len,
                     const char magic[4]) {
  return len >= 4 && std::memcmp(data, magic, 4) == 0;
}

// Append one route leg: lat(f64) lon(f64) idLen(i32) id[idLen]. The id is
// truncated to kMaxIdLength so a pathological waypoint still fits the cap.
inline void putLeg(std::vector<unsigned char>& out, const MapLeg& leg) {
  putF64(out, leg.lat);
  putF64(out, leg.lon);
  std::int32_t idLen = static_cast<std::int32_t>(leg.id.size());
  if (idLen > kMaxIdLength) idLen = kMaxIdLength;
  putI32(out, idLen);
  out.insert(out.end(), leg.id.begin(),
             leg.id.begin() + static_cast<std::ptrdiff_t>(idLen));
}

// Read one leg starting at `off`, advancing it. Returns false on truncation or
// an out-of-range id length.
inline bool getLeg(const unsigned char* data, std::size_t len, std::size_t& off,
                   MapLeg& leg) {
  if (off + kLegFixedBytes > len) return false;
  leg.lat = getF64(data + off);
  off += 8;
  leg.lon = getF64(data + off);
  off += 8;
  const std::int32_t idLen = getI32(data + off);
  off += 4;
  if (idLen < 0 || idLen > kMaxIdLength) return false;
  if (off + static_cast<std::size_t>(idLen) > len) return false;
  leg.id.assign(reinterpret_cast<const char*>(data + off),
                static_cast<std::size_t>(idLen));
  off += static_cast<std::size_t>(idLen);
  return true;
}

// Serialize a route as `magic | version | count | legs`. Shared by the reply
// (FPLR) and the program-route command (FPLS). Legs beyond kMaxLegs are dropped.
inline std::vector<unsigned char> encodePlan(const char magic[4],
                                             const std::vector<MapLeg>& legs) {
  std::vector<unsigned char> out;
  out.insert(out.end(), magic, magic + 4);
  putI32(out, kProtocolVersion);
  const std::int32_t count =
      legs.size() > static_cast<std::size_t>(kMaxLegs)
          ? kMaxLegs
          : static_cast<std::int32_t>(legs.size());
  putI32(out, count);
  for (std::int32_t i = 0; i < count; ++i) {
    putLeg(out, legs[static_cast<std::size_t>(i)]);
  }
  return out;
}

// Parse a route datagram of the given `magic`. Returns false (out left empty)
// on any malformed or version-mismatched datagram.
inline bool decodePlan(const unsigned char* data, std::size_t len,
                       const char magic[4], std::vector<MapLeg>& out) {
  out.clear();
  if (len < kReplyHeaderBytes) return false;
  if (!hasMagic(data, len, magic)) return false;

  std::size_t off = 4;
  const std::int32_t version = getI32(data + off);
  off += 4;
  if (version != kProtocolVersion) return false;

  const std::int32_t count = getI32(data + off);
  off += 4;
  if (count < 0 || count > kMaxLegs) return false;

  out.reserve(static_cast<std::size_t>(count));
  for (std::int32_t i = 0; i < count; ++i) {
    MapLeg leg;
    if (!getLeg(data, len, off, leg)) {
      out.clear();
      return false;
    }
    out.push_back(std::move(leg));
  }
  return true;
}

// --- request (FPLQ) ---

inline std::vector<unsigned char> encodeRequest() {
  return std::vector<unsigned char>(kRequestMagic, kRequestMagic + 4);
}
inline bool isRequest(const unsigned char* data, std::size_t len) {
  return hasMagic(data, len, kRequestMagic);
}

// --- route reply (FPLR) and program-route command (FPLS) ---

inline void putDirectToOrigin(std::vector<unsigned char>& out,
                              bool originValid, double originLat,
                              double originLon) {
  putI32(out, originValid ? 1 : 0);
  putF64(out, originLat);
  putF64(out, originLon);
}

inline bool getDirectToOrigin(const unsigned char* data, std::size_t len,
                              std::size_t& off, bool& originValid,
                              double& originLat, double& originLon) {
  if (off + kDirectToOriginBytes > len) return false;
  originValid = getI32(data + off) != 0;
  off += 4;
  originLat = getF64(data + off);
  off += 8;
  originLon = getF64(data + off);
  off += 8;
  return true;
}

inline void putDirectToTrailer(std::vector<unsigned char>& out,
                               const DirectToState& dto) {
  putI32(out, dto.active ? 1 : 0);
  if (!dto.active) return;
  putLeg(out, dto.target);
  putDirectToOrigin(out, dto.originValid, dto.originLat, dto.originLon);
}

inline bool getDirectToTrailer(const unsigned char* data, std::size_t len,
                               std::size_t& off, DirectToState& dto) {
  dto = {};
  if (off + 4 > len) return false;
  dto.active = getI32(data + off) != 0;
  off += 4;
  if (!dto.active) return true;
  if (!getLeg(data, len, off, dto.target)) return false;
  return getDirectToOrigin(data, len, off, dto.originValid, dto.originLat,
                           dto.originLon);
}

// FPLR: magic | version | count | legs | dtoActive | [target | origin].
inline std::vector<unsigned char> encodeReply(const std::vector<MapLeg>& legs,
                                              const DirectToState& dto = {}) {
  std::vector<unsigned char> out = encodePlan(kReplyMagic, legs);
  putDirectToTrailer(out, dto);
  return out;
}

inline bool decodeReply(const unsigned char* data, std::size_t len,
                        std::vector<MapLeg>& out,
                        DirectToState* dtoOut = nullptr) {
  out.clear();
  DirectToState dto;
  if (len < kReplyHeaderBytes) return false;
  if (!hasMagic(data, len, kReplyMagic)) return false;

  std::size_t off = 4;
  const std::int32_t version = getI32(data + off);
  off += 4;
  if (version != kProtocolVersion) return false;

  const std::int32_t count = getI32(data + off);
  off += 4;
  if (count < 0 || count > kMaxLegs) return false;

  out.reserve(static_cast<std::size_t>(count));
  for (std::int32_t i = 0; i < count; ++i) {
    MapLeg leg;
    if (!getLeg(data, len, off, leg)) {
      out.clear();
      return false;
    }
    out.push_back(std::move(leg));
  }

  if (off >= len) {
    if (dtoOut != nullptr) *dtoOut = dto;
    return true;
  }
  if (!getDirectToTrailer(data, len, off, dto)) {
    out.clear();
    return false;
  }
  if (dtoOut != nullptr) *dtoOut = dto;
  return true;
}

inline std::vector<unsigned char> encodeSetPlan(
    const std::vector<MapLeg>& legs) {
  return encodePlan(kSetPlanMagic, legs);
}
inline bool isSetPlan(const unsigned char* data, std::size_t len) {
  return hasMagic(data, len, kSetPlanMagic);
}
inline bool decodeSetPlan(const unsigned char* data, std::size_t len,
                          std::vector<MapLeg>& out) {
  return decodePlan(data, len, kSetPlanMagic, out);
}

// --- Direct-To command (FPLD) ---
//
// Layout: magic | version | active(i32, 1 = set / 0 = clear) |
//         [leg | originValid | originLat | originLon | programFms if active] |
//         [programFms if !active].
// programFms: when set, also program/clear the sim FMS; when clear on active=0,
// controls whether the sim FMS Direct-To is cleared. Display state is always
// updated from FPLD; use FPLX to clear display without touching the FMS.

inline std::vector<unsigned char> encodeSetDirectTo(
    bool active, const MapLeg& target, bool originValid = false,
    double originLat = 0.0, double originLon = 0.0, bool programFms = true) {
  std::vector<unsigned char> out;
  out.insert(out.end(), kSetDtoMagic, kSetDtoMagic + 4);
  putI32(out, kProtocolVersion);
  putI32(out, active ? 1 : 0);
  if (active) {
    putLeg(out, target);
    putDirectToOrigin(out, originValid, originLat, originLon);
  }
  putI32(out, programFms ? 1 : 0);
  return out;
}
inline bool isSetDirectTo(const unsigned char* data, std::size_t len) {
  return hasMagic(data, len, kSetDtoMagic);
}
inline bool decodeSetDirectTo(const unsigned char* data, std::size_t len,
                              bool& active, MapLeg& target,
                              bool& originValid, double& originLat,
                              double& originLon, bool& programFms) {
  active = false;
  originValid = false;
  originLat = 0.0;
  originLon = 0.0;
  programFms = true;
  if (len < kReplyHeaderBytes + 4) return false;
  if (!hasMagic(data, len, kSetDtoMagic)) return false;
  std::size_t off = 4;
  if (getI32(data + off) != kProtocolVersion) return false;
  off += 4;
  active = getI32(data + off) != 0;
  off += 4;
  if (active) {
    if (!getLeg(data, len, off, target)) return false;
    if (!getDirectToOrigin(data, len, off, originValid, originLat, originLon)) {
      return false;
    }
  }
  if (off + 4 > len) return false;
  programFms = getI32(data + off) != 0;
  return true;
}

// --- display-only Direct-To clear (FPLX) ---
//
// Layout: magic | version. Clears the plugin's stored Direct-To display state
// without reprogramming the sim FMS (used when AP NAV is coupled and the sim
// FMS must stay untouched through Direct-To capture).

inline std::vector<unsigned char> encodeClearDirectToDisplay() {
  std::vector<unsigned char> out;
  out.insert(out.end(), kClearDtoDisplayMagic, kClearDtoDisplayMagic + 4);
  putI32(out, kProtocolVersion);
  return out;
}
inline bool isClearDirectToDisplay(const unsigned char* data, std::size_t len) {
  return hasMagic(data, len, kClearDtoDisplayMagic) && len >= 8 &&
         getI32(data + 4) == kProtocolVersion;
}

// --- active-leg command (FPLG) ---
//
// Layout: magic | version | legIndex(i32).

inline std::vector<unsigned char> encodeSetActiveLeg(int legIndex) {
  std::vector<unsigned char> out;
  out.insert(out.end(), kSetActiveLegMagic, kSetActiveLegMagic + 4);
  putI32(out, kProtocolVersion);
  putI32(out, legIndex);
  return out;
}
inline bool isSetActiveLeg(const unsigned char* data, std::size_t len) {
  return hasMagic(data, len, kSetActiveLegMagic);
}
inline bool decodeSetActiveLeg(const unsigned char* data, std::size_t len,
                               int& legIndex) {
  legIndex = -1;
  if (len < kReplyHeaderBytes) return false;
  if (!hasMagic(data, len, kSetActiveLegMagic)) return false;
  std::size_t off = 4;
  if (getI32(data + off) != kProtocolVersion) return false;
  off += 4;
  legIndex = getI32(data + off);
  return legIndex >= 0;
}

// --- write acknowledgement (FPLA) ---

inline std::vector<unsigned char> encodeAck() {
  std::vector<unsigned char> out;
  out.insert(out.end(), kAckMagic, kAckMagic + 4);
  putI32(out, kProtocolVersion);
  return out;
}
inline bool isAck(const unsigned char* data, std::size_t len) {
  return hasMagic(data, len, kAckMagic) && len >= 8 &&
         getI32(data + 4) == kProtocolVersion;
}

}  // namespace fpbridge
}  // namespace avionics
