#pragma once

#include <istream>
#include <string>

// Self-contained SHA-256 (no OS crypto dependency), used by the in-app updater
// to verify a downloaded installer against the release's SHA256SUMS on every
// platform. Implementation is the standard FIPS 180-4 algorithm.
namespace avionics {

// Lowercase hex SHA-256 of all bytes read from the stream. Empty on read
// failure. Reads in chunks, so it is safe on large installer files.
std::string sha256HexOfStream(std::istream& in);

}  // namespace avionics
