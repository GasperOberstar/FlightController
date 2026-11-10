add_library(fc_sim STATIC src/plant_SingleAxis.cpp)
target_include_directories(fc_sim PUBLIC include)
target_link_libraries(fc_sim PUBLIC fc_core)

add_executable(rate_loop_sim apps/rate_loop_sim.cpp)
target_link_libraries(rate_loop_sim PRIVATE fc_sim)