include(FetchContent)
FetchContent_Declare(
    googletest
    GIT_REPOSITORY https://github.com/google/googletest.git
    GIT_TAG        v1.14.0
)
# Prevent installing gtest alongside the project
set(INSTALL_GTEST OFF CACHE BOOL "" FORCE)
FetchContent_MakeAvailable(googletest)

add_executable(haldex_tests
    unit/testSignalProcessing.cpp
    unit/testStateEstimation.cpp
    unit/testLockCalculation.cpp
    unit/testSafetyOverrides.cpp
    simulation/frozenSensorTest.cpp
)

target_link_libraries(haldex_tests PRIVATE haldex_core GTest::gtest_main)
target_include_directories(haldex_tests PRIVATE ${CMAKE_SOURCE_DIR}/src)

include(GoogleTest)
# PRE_TEST: ctest enumerates the tests by running the binary at `ctest` time, not at
# build time. Avoids the POST_BUILD discovery race (binary built, but ctest sees no tests)
# and behaves identically on Windows (multi-config) and macOS/Linux (single-config).
gtest_discover_tests(haldex_tests DISCOVERY_MODE PRE_TEST)
