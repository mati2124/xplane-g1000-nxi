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

// Raw 32-byte SHA-256 digest of the given bytes (binary, not hex). Used to
// build the PKCE code-challenge for the Navigraph device-authorization flow.
std::string sha256Raw(const std::string& data);

}  // namespace avionics
