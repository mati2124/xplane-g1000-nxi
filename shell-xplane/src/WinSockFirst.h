#pragma once

// Force-included (/FI) on the xplane-avionics target on Windows so winsock2.h
// is always seen before windows.h (pulled in transitively by XPLM headers).
#ifdef _WIN32
#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#include <winsock2.h>
#include <ws2tcpip.h>
#endif
