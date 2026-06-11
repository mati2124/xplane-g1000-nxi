# Tiny dev helper: Plugins → Reload Plugins calls XPLMReloadPlugins() so you can
# pick up a rebuilt xplane-avionics.xpl without restarting the sim.

if(NOT XPLANE_SDK_DIR)
  message(FATAL_ERROR
    "reload-plugins requires XPLANE_SDK_DIR (set when BUILD_XPLANE_SHELL=ON)")
endif()

add_library(reload-plugins MODULE src/plugin_main.cpp)

target_include_directories(reload-plugins PRIVATE
  ${XPLANE_SDK_DIR}/CHeaders/XPLM
  ${XPLANE_SDK_DIR}/CHeaders/Widgets
)

target_compile_definitions(reload-plugins PRIVATE XPLM200 XPLM210 XPLM300)
if(APPLE)
  target_compile_definitions(reload-plugins PRIVATE APL=1 IBM=0 LIN=0)
elseif(WIN32)
  target_compile_definitions(reload-plugins PRIVATE APL=0 IBM=1 LIN=0)
else()
  target_compile_definitions(reload-plugins PRIVATE APL=0 IBM=0 LIN=1)
endif()

set_target_properties(reload-plugins PROPERTIES
  PREFIX ""
  OUTPUT_NAME "reload-plugins"
  SUFFIX ".xpl"
)

if(APPLE)
  target_link_libraries(reload-plugins PRIVATE
    "-F${XPLANE_SDK_DIR}/Libraries/Mac"
    "-framework XPLM"
  )
elseif(WIN32)
  target_link_libraries(reload-plugins PRIVATE
    ${XPLANE_SDK_DIR}/Libraries/Win/XPLM_64.lib
  )
endif()
