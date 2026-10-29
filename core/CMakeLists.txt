add_library(fc_core STATIC
    src/pt1.cpp
)

target_include_directories(fc_core PUBLIC include)

# Embedded-style restrictions apply to core only, and are PRIVATE so they
# don't leak into targets that link against it (GoogleTest needs exceptions).
target_compile_options(fc_core PRIVATE -fno-exceptions -fno-rtti)
target_link_libraries(fc_core PRIVATE fc_warnings)