#include "XPlaneWebApi.h"

#include <cctype>
#include <cerrno>
#include <chrono>
#include <cstdlib>
#include <cstring>
#include <string>

#include "avionics/Datarefs.h"

#ifdef _WIN32
#include <winsock2.h>
#include <ws2tcpip.h>
#pragma comment(lib, "Ws2_32.lib")
using SocketHandle = SOCKET;
#else
#include <arpa/inet.h>
#include <fcntl.h>
#include <netinet/in.h>
#include <sys/select.h>
#include <sys/socket.h>
#include <unistd.h>
using SocketHandle = int;
#endif

namespace avionics {
namespace {

#ifdef _WIN32
constexpr SocketHandle kInvalidSocket = INVALID_SOCKET;
#else
constexpr SocketHandle kInvalidSocket = -1;
#endif

// X-Plane web server is local and very responsive, so these can be short; the
// point is to fail fast when it is absent rather than block the poll thread.
constexpr int kConnectTimeoutMs = 800;
constexpr int kIoTimeoutMs = 800;
constexpr int kPollIntervalMs = 1000;
constexpr int kPollSliceMs = 100;  // shutdown responsiveness within a poll wait
constexpr std::size_t kMaxResponseBytes = 1u << 20;  // 1 MiB safety cap
constexpr int kRecvChunk = 4096;

// X-Plane Web API v3 REST routes (versioned under /api/v3).
constexpr const char* kDatarefsByName = "/api/v3/datarefs?filter[name]=";
constexpr const char* kDatarefsPrefix = "/api/v3/datarefs/";
constexpr const char* kValueSuffix = "/value";

void closeSocket(SocketHandle sock) {
#ifdef _WIN32
  closesocket(sock);
#else
  ::close(sock);
#endif
}

void setNonBlocking(SocketHandle sock, bool nonBlocking) {
#ifdef _WIN32
  u_long mode = nonBlocking ? 1 : 0;
  ioctlsocket(sock, FIONBIO, &mode);
#else
  int flags = fcntl(sock, F_GETFL, 0);
  if (flags < 0) return;
  fcntl(sock, F_SETFL, nonBlocking ? (flags | O_NONBLOCK) : (flags & ~O_NONBLOCK));
#endif
}

bool lastErrorWouldBlock() {
#ifdef _WIN32
  const int e = WSAGetLastError();
  return e == WSAEWOULDBLOCK || e == WSAEINPROGRESS;
#else
  return errno == EINPROGRESS || errno == EWOULDBLOCK || errno == EAGAIN;
#endif
}

void setIoTimeout(SocketHandle sock, int timeoutMs) {
#ifdef _WIN32
  DWORD tv = static_cast<DWORD>(timeoutMs);
  setsockopt(sock, SOL_SOCKET, SO_RCVTIMEO, reinterpret_cast<const char*>(&tv),
             sizeof(tv));
  setsockopt(sock, SOL_SOCKET, SO_SNDTIMEO, reinterpret_cast<const char*>(&tv),
             sizeof(tv));
#else
  timeval tv;
  tv.tv_sec = timeoutMs / 1000;
  tv.tv_usec = (timeoutMs % 1000) * 1000;
  setsockopt(sock, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));
  setsockopt(sock, SOL_SOCKET, SO_SNDTIMEO, &tv, sizeof(tv));
#endif
}

// Non-blocking connect with a select() timeout so an absent web server fails
// quickly instead of blocking on the OS default connect timeout.
SocketHandle connectWithTimeout(const std::string& host, std::uint16_t port,
                                int timeoutMs) {
  SocketHandle sock = ::socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
  if (sock == kInvalidSocket) return kInvalidSocket;

  sockaddr_in addr{};
  addr.sin_family = AF_INET;
  addr.sin_port = htons(port);
  if (inet_pton(AF_INET, host.c_str(), &addr.sin_addr) != 1) {
    closeSocket(sock);
    return kInvalidSocket;
  }

  setNonBlocking(sock, true);
  const int rc = ::connect(sock, reinterpret_cast<sockaddr*>(&addr),
                           sizeof(addr));
  if (rc != 0 && !lastErrorWouldBlock()) {
    closeSocket(sock);
    return kInvalidSocket;
  }
  if (rc != 0) {
    fd_set wfds;
    FD_ZERO(&wfds);
    FD_SET(sock, &wfds);
    timeval tv;
    tv.tv_sec = timeoutMs / 1000;
    tv.tv_usec = (timeoutMs % 1000) * 1000;
    if (::select(static_cast<int>(sock) + 1, nullptr, &wfds, nullptr, &tv) <= 0) {
      closeSocket(sock);
      return kInvalidSocket;
    }
    int err = 0;
    socklen_t len = sizeof(err);
    getsockopt(sock, SOL_SOCKET, SO_ERROR, reinterpret_cast<char*>(&err), &len);
    if (err != 0) {
      closeSocket(sock);
      return kInvalidSocket;
    }
  }
  setNonBlocking(sock, false);
  return sock;
}

bool sendAll(SocketHandle sock, const std::string& data) {
  std::size_t sent = 0;
  while (sent < data.size()) {
    const int n = ::send(sock, data.data() + sent,
                         static_cast<int>(data.size() - sent), 0);
    if (n <= 0) return false;
    sent += static_cast<std::size_t>(n);
  }
  return true;
}

int recvSome(SocketHandle sock, char* buf, int cap) {
  return static_cast<int>(::recv(sock, buf, cap, 0));
}

std::string toLower(std::string s) {
  for (char& c : s) c = static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
  return s;
}

// Reassemble a chunked transfer-encoded body. Responses here are tiny (a single
// base64 string), so a straightforward size-prefixed walk is sufficient.
std::string dechunk(const std::string& in) {
  std::string out;
  std::size_t i = 0;
  while (i < in.size()) {
    const std::size_t eol = in.find("\r\n", i);
    if (eol == std::string::npos) break;
    std::string hex = in.substr(i, eol - i);
    const std::size_t semi = hex.find(';');  // strip chunk extensions
    if (semi != std::string::npos) hex = hex.substr(0, semi);
    const long len = std::strtol(hex.c_str(), nullptr, 16);
    if (len <= 0) break;
    const std::size_t dataStart = eol + 2;
    if (dataStart + static_cast<std::size_t>(len) > in.size()) break;
    out.append(in, dataStart, static_cast<std::size_t>(len));
    i = dataStart + static_cast<std::size_t>(len) + 2;  // skip data + CRLF
  }
  return out;
}

// Issue a GET and return the (de-framed) response body. Honors Content-Length
// and chunked encoding, falling back to read-until-close.
bool httpGet(const std::string& host, std::uint16_t port,
             const std::string& path, std::string& bodyOut) {
  SocketHandle sock = connectWithTimeout(host, port, kConnectTimeoutMs);
  if (sock == kInvalidSocket) return false;
  setIoTimeout(sock, kIoTimeoutMs);

  const std::string req = "GET " + path + " HTTP/1.1\r\nHost: " + host +
                          "\r\nAccept: application/json\r\nConnection: close\r\n\r\n";
  if (!sendAll(sock, req)) {
    closeSocket(sock);
    return false;
  }

  std::string data;
  char chunk[kRecvChunk];
  std::size_t headerEnd = std::string::npos;
  while (headerEnd == std::string::npos) {
    const int n = recvSome(sock, chunk, sizeof(chunk));
    if (n <= 0) break;
    data.append(chunk, static_cast<std::size_t>(n));
    headerEnd = data.find("\r\n\r\n");
    if (data.size() > kMaxResponseBytes) break;
  }
  if (headerEnd == std::string::npos) {
    closeSocket(sock);
    return false;
  }

  const std::string headers = data.substr(0, headerEnd);
  std::string body = data.substr(headerEnd + 4);

  // Status line must be 2xx.
  if (headers.compare(0, 7, "HTTP/1.") != 0 || headers.size() < 12 ||
      headers[9] != '2') {
    closeSocket(sock);
    return false;
  }

  const std::string h = toLower(headers);
  long contentLength = -1;
  const std::size_t clPos = h.find("content-length:");
  if (clPos != std::string::npos) {
    contentLength = std::strtol(h.c_str() + clPos + 15, nullptr, 10);
  }
  const bool chunked = h.find("transfer-encoding: chunked") != std::string::npos;

  if (chunked) {
    while (body.find("0\r\n\r\n") == std::string::npos &&
           body.size() < kMaxResponseBytes) {
      const int n = recvSome(sock, chunk, sizeof(chunk));
      if (n <= 0) break;
      body.append(chunk, static_cast<std::size_t>(n));
    }
    body = dechunk(body);
  } else if (contentLength >= 0) {
    while (body.size() < static_cast<std::size_t>(contentLength)) {
      const int n = recvSome(sock, chunk, sizeof(chunk));
      if (n <= 0) break;
      body.append(chunk, static_cast<std::size_t>(n));
    }
    body.resize(static_cast<std::size_t>(contentLength));
  } else {
    while (body.size() < kMaxResponseBytes) {
      const int n = recvSome(sock, chunk, sizeof(chunk));
      if (n <= 0) break;
      body.append(chunk, static_cast<std::size_t>(n));
    }
  }

  closeSocket(sock);
  bodyOut = std::move(body);
  return true;
}

// Pull the first integer "id" field out of a datarefs list response.
bool extractId(const std::string& body, long long& idOut) {
  const std::size_t key = body.find("\"id\"");
  if (key == std::string::npos) return false;
  std::size_t p = body.find(':', key);
  if (p == std::string::npos) return false;
  ++p;
  while (p < body.size() &&
         std::isspace(static_cast<unsigned char>(body[p]))) {
    ++p;
  }
  char* end = nullptr;
  const long long v = std::strtoll(body.c_str() + p, &end, 10);
  if (end == body.c_str() + p) return false;
  idOut = v;
  return true;
}

// Pull the base64 string out of a {"data":"..."} value response. Returns false
// if "data" is absent or not a string (e.g. JSON null when no value).
bool extractDataString(const std::string& body, std::string& base64Out) {
  const std::size_t key = body.find("\"data\"");
  if (key == std::string::npos) return false;
  std::size_t p = body.find(':', key);
  if (p == std::string::npos) return false;
  ++p;
  while (p < body.size() &&
         std::isspace(static_cast<unsigned char>(body[p]))) {
    ++p;
  }
  if (p >= body.size() || body[p] != '"') return false;
  ++p;
  std::string out;
  while (p < body.size() && body[p] != '"') {
    if (body[p] == '\\' && p + 1 < body.size()) ++p;  // skip escape marker
    out.push_back(body[p]);
    ++p;
  }
  base64Out = std::move(out);
  return true;
}

std::string base64Decode(const std::string& in) {
  auto sextet = [](char c) -> int {
    if (c >= 'A' && c <= 'Z') return c - 'A';
    if (c >= 'a' && c <= 'z') return c - 'a' + 26;
    if (c >= '0' && c <= '9') return c - '0' + 52;
    if (c == '+') return 62;
    if (c == '/') return 63;
    return -1;
  };
  std::string out;
  int buffer = 0;
  int bits = 0;
  for (const char c : in) {
    if (c == '=') break;
    const int v = sextet(c);
    if (v < 0) continue;  // skip whitespace / newlines
    buffer = (buffer << 6) | v;
    bits += 6;
    if (bits >= 8) {
      bits -= 8;
      out.push_back(static_cast<char>((buffer >> bits) & 0xFF));
    }
  }
  return out;
}

// Decode a byte[] string dataref value: base64 -> bytes, then take the
// null-terminated identifier and trim trailing whitespace.
std::string decodeIdentifier(const std::string& base64) {
  std::string bytes = base64Decode(base64);
  const std::size_t nul = bytes.find('\0');
  if (nul != std::string::npos) bytes.resize(nul);
  while (!bytes.empty() &&
         std::isspace(static_cast<unsigned char>(bytes.back()))) {
    bytes.pop_back();
  }
  return bytes;
}

// Resolve a dataref's numeric Web API id (once per session) and read its value.
// Returns false when the server is unreachable so the caller can drop the id.
bool pollStringDataref(const std::string& host, std::uint16_t port,
                       const char* path, long long& datarefId,
                       std::string& valueOut) {
  if (datarefId < 0) {
    std::string body;
    if (httpGet(host, port, std::string(kDatarefsByName) + path, body)) {
      long long id = -1;
      if (extractId(body, id)) datarefId = id;
    }
  }
  if (datarefId < 0) {
    valueOut.clear();
    return false;
  }

  std::string body;
  const std::string valuePath = std::string(kDatarefsPrefix) +
                                std::to_string(datarefId) + kValueSuffix;
  if (!httpGet(host, port, valuePath, body)) {
    datarefId = -1;
    valueOut.clear();
    return false;
  }

  std::string base64;
  if (extractDataString(body, base64)) {
    valueOut = decodeIdentifier(base64);
  } else {
    valueOut.clear();
  }
  return true;
}

std::string readNavStationIdent(const std::string& host, std::uint16_t port,
                                const char* navPath, const char* dmePath,
                                long long& navId, long long& dmeId) {
  std::string id;
  pollStringDataref(host, port, navPath, navId, id);
  if (!id.empty()) return id;
  pollStringDataref(host, port, dmePath, dmeId, id);
  return id;
}

}  // namespace

XPlaneWebApi::XPlaneWebApi(std::string host, std::uint16_t port)
    : host_(std::move(host)), port_(port) {
#ifdef _WIN32
  WSADATA wsa;
  WSAStartup(MAKEWORD(2, 2), &wsa);
#endif
  thread_ = std::thread(&XPlaneWebApi::run, this);
}

XPlaneWebApi::~XPlaneWebApi() {
  stop_.store(true);
  if (thread_.joinable()) thread_.join();
#ifdef _WIN32
  WSACleanup();
#endif
}

std::string XPlaneWebApi::destinationId() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return destinationId_;
}

std::string XPlaneWebApi::nav1Ident() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return nav1Ident_;
}

std::string XPlaneWebApi::nav2Ident() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return nav2Ident_;
}

void XPlaneWebApi::run() {
  long long gpsNavId = -1;
  long long nav1NavId = -1;
  long long nav1DmeId = -1;
  long long nav2NavId = -1;
  long long nav2DmeId = -1;
  while (!stop_.load()) {
    std::string gps;
    std::string nav1;
    std::string nav2;
    const bool gpsOk =
        pollStringDataref(host_, port_, datarefs::kGpsNavId, gpsNavId, gps);
    nav1 = readNavStationIdent(host_, port_, datarefs::kNav1NavId,
                               datarefs::kNav1DmeId, nav1NavId, nav1DmeId);
    nav2 = readNavStationIdent(host_, port_, datarefs::kNav2NavId,
                               datarefs::kNav2DmeId, nav2NavId, nav2DmeId);

    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (gpsOk) {
        destinationId_ = std::move(gps);
      } else {
        destinationId_.clear();
      }
      nav1Ident_ = std::move(nav1);
      nav2Ident_ = std::move(nav2);
    }

    for (int waited = 0; waited < kPollIntervalMs && !stop_.load();
         waited += kPollSliceMs) {
      std::this_thread::sleep_for(std::chrono::milliseconds(kPollSliceMs));
    }
  }
}

}  // namespace avionics
