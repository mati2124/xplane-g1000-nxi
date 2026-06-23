include(FetchContent)

FetchContent_Declare(
  googletest
  URL https://github.com/google/googletest/archive/refs/tags/v1.14.0.zip
  DOWNLOAD_EXTRACT_TIMESTAMP TRUE)
set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
FetchContent_MakeAvailable(googletest)

add_executable(avionics-core-tests
  GpsLegCourseTest.cpp
  GlidepathGuidanceTest.cpp
  CifpParserTest.cpp
)

target_compile_definitions(avionics-core-tests PRIVATE
  AVIONICS_TEST_FIXTURE_DIR="${CMAKE_CURRENT_SOURCE_DIR}/fixtures")

target_link_libraries(avionics-core-tests PRIVATE avionics-core GTest::gtest_main)
target_include_directories(avionics-core-tests PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})

include(GoogleTest)
gtest_discover_tests(avionics-core-tests)
