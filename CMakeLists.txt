cmake_minimum_required(VERSION 3.20)
project(FlightController LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Warning flags live on an interface target so only our code gets them,
# not GoogleTest.
add_library(fc_warnings INTERFACE)
target_compile_options(fc_warnings INTERFACE
    -Wall -Wextra -Wconversion -Wdouble-promotion)

enable_testing()

add_subdirectory(core)
add_subdirectory(tests)