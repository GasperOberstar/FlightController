include(FetchContent)

FetchContent_Declare(
    googletest
    URL https://github.com/google/googletest/archive/refs/tags/v1.15.2.zip
    DOWNLOAD_EXTRACT_TIMESTAMP TRUE
)
FetchContent_MakeAvailable(googletest)

add_executable(fc_tests
    test_pt1.cpp
)
target_link_libraries(fc_tests PRIVATE fc_core fc_warnings GTest::gtest_main)

include(GoogleTest)
gtest_discover_tests(fc_tests)