// stb_image is compiled into NexradWeatherRadar.cpp with STB_IMAGE_STATIC, so
// BootScreen / MfdChartsPage symbols are not exported from avionics-core.lib.
// The test executable links the full library (including AvionicsEngine) and
// needs its own implementation when AVIONICS_HAS_STBIMAGE is defined.
#if defined(AVIONICS_HAS_STBIMAGE)
#define STB_IMAGE_IMPLEMENTATION
#define STBI_ONLY_PNG
#define STBI_NO_STDIO
#include "stb_image.h"
#endif
