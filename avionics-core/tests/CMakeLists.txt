include(FetchContent)

FetchContent_Declare(
  googletest
  URL https://github.com/google/googletest/archive/refs/tags/v1.14.0.zip
  DOWNLOAD_EXTRACT_TIMESTAMP TRUE)
set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
FetchContent_MakeAvailable(googletest)

# stb is fetched later for avionics-core when a shell is enabled; pull it in here
# too so test targets that link AvionicsEngine resolve BootScreen's stbi symbols.
if(BUILD_XPLANE_SHELL OR BUILD_STANDALONE_SHELL)
  FetchContent_Declare(stb
    GIT_REPOSITORY https://github.com/nothings/stb.git
    GIT_TAG f0569113c93ad095470c54bf34a17b36646bbbb5)
  FetchContent_MakeAvailable(stb)
endif()

add_executable(avionics-core-tests
  GpsLegCourseTest.cpp
  FmsNavigatorTest.cpp
  HoldNavigationTest.cpp
  GlidepathGuidanceTest.cpp
  VnavGuidanceTest.cpp
  MissedApproachGuidanceTest.cpp
  CifpParserTest.cpp
  MetarParserTest.cpp
  NearbyFeatureSelectTest.cpp
  RadioVolumeAnnunciationTest.cpp
  FlightPlanPersistenceTest.cpp
  FlightPlanBridgeProtocolTest.cpp
  FplHoldRowTest.cpp
  FplCursorLegIndexTest.cpp
  AirwayLoadTest.cpp
  DepartureLoadTest.cpp
  ArrivalLoadTest.cpp
  StarApproachParentTest.cpp
  DirectToFplSelectionTest.cpp
  ChecklistPageTest.cpp
  WptAirportInfoPageTest.cpp
  ProcedureMenuCourseReversalTest.cpp
  DeleteFlightPlanTest.cpp
  RemoveWaypointTest.cpp
  ProcedureRemoveTest.cpp
  FlightPlanPeerSyncTest.cpp
  FlightPlanCatalogTest.cpp
  SimBriefParseTest.cpp
  FplLayoutDestinationTest.cpp
  FplSectionCommitCursorTest.cpp
  MapRouteLegsTest.cpp
  AptDatPavementTest.cpp
  DirectToRestoreTest.cpp
  stb_image_impl.cpp
)

if(DEFINED stb_SOURCE_DIR)
  target_include_directories(avionics-core-tests PRIVATE ${stb_SOURCE_DIR})
  target_compile_definitions(avionics-core-tests PRIVATE AVIONICS_HAS_STBIMAGE)
endif()

target_compile_definitions(avionics-core-tests PRIVATE
  AVIONICS_TEST_FIXTURE_DIR="${CMAKE_CURRENT_SOURCE_DIR}/fixtures")

target_link_libraries(avionics-core-tests PRIVATE avionics-core GTest::gtest_main)
target_include_directories(avionics-core-tests PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}
  ${CMAKE_SOURCE_DIR}/avionics-core/src)

include(GoogleTest)
gtest_discover_tests(avionics-core-tests)
