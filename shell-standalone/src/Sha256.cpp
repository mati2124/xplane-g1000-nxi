#include "Sha256.h"

#include <algorithm>
#include <array>
#include <cstdint>
#include <cstring>
#include <vector>

namespace avionics {
namespace {

constexpr std::uint32_t kK[64] = {
    0x428a2f98, 0x71374491, 0xb5c0fbcf, 0xe9b5dba5, 0x3956c25b, 0x59f111f1,
    0x923f82a4, 0xab1c5ed5, 0xd807aa98, 0x12835b01, 0x243185be, 0x550c7dc3,
    0x72be5d74, 0x80deb1fe, 0x9bdc06a7, 0xc19bf174, 0xe49b69c1, 0xefbe4786,
    0x0fc19dc6, 0x240ca1cc, 0x2de92c6f, 0x4a7484aa, 0x5cb0a9dc, 0x76f988da,
    0x983e5152, 0xa831c66d, 0xb00327c8, 0xbf597fc7, 0xc6e00bf3, 0xd5a79147,
    0x06ca6351, 0x14292967, 0x27b70a85, 0x2e1b2138, 0x4d2c6dfc, 0x53380d13,
    0x650a7354, 0x766a0abb, 0x81c2c92e, 0x92722c85, 0xa2bfe8a1, 0xa81a664b,
    0xc24b8b70, 0xc76c51a3, 0xd192e819, 0xd6990624, 0xf40e3585, 0x106aa070,
    0x19a4c116, 0x1e376c08, 0x2748774c, 0x34b0bcb5, 0x391c0cb3, 0x4ed8aa4a,
    0x5b9cca4f, 0x682e6ff3, 0x748f82ee, 0x78a5636f, 0x84c87814, 0x8cc70208,
    0x90befffa, 0xa4506ceb, 0xbef9a3f7, 0xc67178f2};

inline std::uint32_t rotr(std::uint32_t x, std::uint32_t n) {
  return (x >> n) | (x << (32 - n));
}

struct Sha256Ctx {
  std::array<std::uint32_t, 8> h = {0x6a09e667, 0xbb67ae85, 0x3c6ef372,
                                    0xa54ff53a, 0x510e527f, 0x9b05688c,
                                    0x1f83d9ab, 0x5be0cd19};
  std::uint64_t totalBytes = 0;

  void transform(const unsigned char* p) {
    std::uint32_t w[64];
    for (int i = 0; i < 16; ++i) {
      w[i] = (static_cast<std::uint32_t>(p[i * 4]) << 24) |
             (static_cast<std::uint32_t>(p[i * 4 + 1]) << 16) |
             (static_cast<std::uint32_t>(p[i * 4 + 2]) << 8) |
             (static_cast<std::uint32_t>(p[i * 4 + 3]));
    }
    for (int i = 16; i < 64; ++i) {
      const std::uint32_t s0 =
          rotr(w[i - 15], 7) ^ rotr(w[i - 15], 18) ^ (w[i - 15] >> 3);
      const std::uint32_t s1 =
          rotr(w[i - 2], 17) ^ rotr(w[i - 2], 19) ^ (w[i - 2] >> 10);
      w[i] = w[i - 16] + s0 + w[i - 7] + s1;
    }
    std::uint32_t a = h[0], b = h[1], c = h[2], d = h[3];
    std::uint32_t e = h[4], f = h[5], g = h[6], hh = h[7];
    for (int i = 0; i < 64; ++i) {
      const std::uint32_t S1 = rotr(e, 6) ^ rotr(e, 11) ^ rotr(e, 25);
      const std::uint32_t ch = (e & f) ^ (~e & g);
      const std::uint32_t t1 = hh + S1 + ch + kK[i] + w[i];
      const std::uint32_t S0 = rotr(a, 2) ^ rotr(a, 13) ^ rotr(a, 22);
      const std::uint32_t maj = (a & b) ^ (a & c) ^ (b & c);
      const std::uint32_t t2 = S0 + maj;
      hh = g;
      g = f;
      f = e;
      e = d + t1;
      d = c;
      c = b;
      b = a;
      a = t1 + t2;
    }
    h[0] += a;
    h[1] += b;
    h[2] += c;
    h[3] += d;
    h[4] += e;
    h[5] += f;
    h[6] += g;
    h[7] += hh;
  }
};

}  // namespace

std::string sha256HexOfStream(std::istream& in) {
  Sha256Ctx ctx;
  std::vector<unsigned char> buffer(64 * 1024);
  unsigned char block[64];
  std::size_t blockFill = 0;

  while (in) {
    in.read(reinterpret_cast<char*>(buffer.data()),
            static_cast<std::streamsize>(buffer.size()));
    const std::streamsize got = in.gcount();
    if (got <= 0) break;
    ctx.totalBytes += static_cast<std::uint64_t>(got);
    const unsigned char* p = buffer.data();
    std::size_t remaining = static_cast<std::size_t>(got);
    // Feed any partially filled block first, then full 64-byte blocks.
    while (remaining > 0) {
      if (blockFill > 0 || remaining < 64) {
        const std::size_t take =
            std::min<std::size_t>(64 - blockFill, remaining);
        std::memcpy(block + blockFill, p, take);
        blockFill += take;
        p += take;
        remaining -= take;
        if (blockFill == 64) {
          ctx.transform(block);
          blockFill = 0;
        }
      } else {
        ctx.transform(p);
        p += 64;
        remaining -= 64;
      }
    }
  }
  if (in.bad()) return std::string();

  // Padding: 0x80, zeros, then the 64-bit big-endian bit length.
  const std::uint64_t bitLen = ctx.totalBytes * 8;
  unsigned char pad = 0x80;
  block[blockFill++] = pad;
  if (blockFill > 56) {
    while (blockFill < 64) block[blockFill++] = 0;
    ctx.transform(block);
    blockFill = 0;
  }
  while (blockFill < 56) block[blockFill++] = 0;
  for (int i = 7; i >= 0; --i) {
    block[blockFill++] =
        static_cast<unsigned char>((bitLen >> (i * 8)) & 0xFF);
  }
  ctx.transform(block);

  static const char* kHex = "0123456789abcdef";
  std::string out;
  out.reserve(64);
  for (std::uint32_t word : ctx.h) {
    for (int i = 3; i >= 0; --i) {
      const unsigned char byte =
          static_cast<unsigned char>((word >> (i * 8)) & 0xFF);
      out += kHex[byte >> 4];
      out += kHex[byte & 0x0F];
    }
  }
  return out;
}

std::string sha256Raw(const std::string& data) {
  Sha256Ctx ctx;
  ctx.totalBytes = data.size();

  const unsigned char* p =
      reinterpret_cast<const unsigned char*>(data.data());
  std::size_t remaining = data.size();
  while (remaining >= 64) {
    ctx.transform(p);
    p += 64;
    remaining -= 64;
  }

  // Final partial block: copy the tail, then append the 0x80/zero/bit-length
  // padding exactly as the streaming finalizer above does.
  unsigned char block[64];
  std::size_t blockFill = remaining;
  std::memcpy(block, p, remaining);
  block[blockFill++] = 0x80;
  if (blockFill > 56) {
    while (blockFill < 64) block[blockFill++] = 0;
    ctx.transform(block);
    blockFill = 0;
  }
  while (blockFill < 56) block[blockFill++] = 0;
  const std::uint64_t bitLen = ctx.totalBytes * 8;
  for (int i = 7; i >= 0; --i) {
    block[blockFill++] =
        static_cast<unsigned char>((bitLen >> (i * 8)) & 0xFF);
  }
  ctx.transform(block);

  std::string out;
  out.resize(32);
  for (int wi = 0; wi < 8; ++wi) {
    for (int i = 0; i < 4; ++i) {
      out[static_cast<std::size_t>(wi * 4 + i)] = static_cast<char>(
          (ctx.h[static_cast<std::size_t>(wi)] >> ((3 - i) * 8)) & 0xFF);
    }
  }
  return out;
}

}  // namespace avionics
