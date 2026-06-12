#pragma once

#include <cstddef>
#include <cstdint>
#include <cstring>
#include <vector>

// Wire protocol for forwarding G1000 bezel / softkey / radio input from the
// in-sim plugin to the networked standalone shell.
//
// X-Plane button commands are only visible inside the sim process. The
// standalone shell runs in its own process and normally has no access to them.
// When the pilot presses a cockpit key, the plugin intercepts the command and
// pushes a compact UDP event to the standalone, which applies it to its own
// PFD/MFD engines on the next frame.
//
// Registration (standalone -> plugin):
//   CMDR  announces the standalone's return address so the plugin knows where
//         to send events. Re-sent periodically so a restart is picked up.
//
// Acknowledgement (plugin -> standalone):
//   CMDA  confirms the plugin received the latest CMDR so the shell can tell
//         the bridge is linked before the first bezel press.
//
// Events (plugin -> standalone):
//   BTNE  one bezel key, softkey, diagonal pan, or radio-knob action.
namespace avionics {
namespace cmdbridge {

// Plugin listens here for standalone registration datagrams.
constexpr std::uint16_t kRegistrationPort = 49102;

// Default port the standalone binds for incoming events. The plugin replies to
// the source address of the latest CMDR, which is normally this port.
constexpr std::uint16_t kDefaultListenPort = 49101;

constexpr std::int32_t kProtocolVersion = 1;

constexpr char kRegisterMagic[4] = {'C', 'M', 'D', 'R'};
constexpr char kAckMagic[4] = {'C', 'M', 'D', 'A'};
constexpr char kEventMagic[4] = {'B', 'T', 'N', 'E'};

// Wire size of one event datagram: magic(4) + version(4) + device(1) +
// phase(1) + kind(1) + pad(1) + value(4) + value2(4).
constexpr std::size_t kEventBytes = 20;

enum class Device : std::uint8_t { Pfd = 0, Mfd = 1 };

enum class Phase : std::uint8_t {
  Begin = 0,
  Continue = 1,  // CLR hold -> Default Map (fired once)
  End = 2,
};

enum class Kind : std::uint8_t {
  Softkey = 0,
  Bezel = 1,
  BezelDiagonal = 2,
  Radio = 3,
};

// NAV/COM knob actions. Ordinal values must stay stable across releases.
enum class RadioAction : std::uint8_t {
  ComToggle = 0,
  ComFlip,
  ComOuterUp,
  ComOuterDown,
  ComInnerUp,
  ComInnerDown,
  NavToggle,
  NavFlip,
  NavOuterUp,
  NavOuterDown,
  NavInnerUp,
  NavInnerDown,
};

struct Event {
  Device device = Device::Pfd;
  Phase phase = Phase::Begin;
  Kind kind = Kind::Bezel;
  std::int32_t value = 0;
  std::int32_t value2 = -1;
};

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

inline bool hasMagic(const unsigned char* data, std::size_t len,
                     const char magic[4]) {
  return len >= 4 && std::memcmp(data, magic, 4) == 0;
}

inline std::vector<unsigned char> encodeRegister() {
  std::vector<unsigned char> out;
  out.insert(out.end(), kRegisterMagic, kRegisterMagic + 4);
  putI32(out, kProtocolVersion);
  return out;
}

inline bool isRegister(const unsigned char* data, std::size_t len) {
  return hasMagic(data, len, kRegisterMagic) && len >= 8 &&
         getI32(data + 4) == kProtocolVersion;
}

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

inline std::vector<unsigned char> encodeEvent(const Event& ev) {
  std::vector<unsigned char> out;
  out.reserve(kEventBytes);
  out.insert(out.end(), kEventMagic, kEventMagic + 4);
  putI32(out, kProtocolVersion);
  out.push_back(static_cast<unsigned char>(ev.device));
  out.push_back(static_cast<unsigned char>(ev.phase));
  out.push_back(static_cast<unsigned char>(ev.kind));
  out.push_back(0);
  putI32(out, ev.value);
  putI32(out, ev.value2);
  return out;
}

inline bool decodeEvent(const unsigned char* data, std::size_t len, Event& ev) {
  if (len < kEventBytes) return false;
  if (!hasMagic(data, len, kEventMagic)) return false;
  if (getI32(data + 4) != kProtocolVersion) return false;
  ev.device = static_cast<Device>(data[8]);
  ev.phase = static_cast<Phase>(data[9]);
  ev.kind = static_cast<Kind>(data[10]);
  ev.value = getI32(data + 12);
  ev.value2 = getI32(data + 16);
  return true;
}

}  // namespace cmdbridge
}  // namespace avionics
