cmake_minimum_required(VERSION 3.15)
project(drone_optimizer LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

add_compile_options(-Wall -Wextra -Wpedantic)

include_directories(
  ${CMAKE_SOURCE_DIR}/src
  ${CMAKE_SOURCE_DIR}/third_party
)

# --- core library: graph + algorithms ---
add_library(drone_core
  src/graph/Graph.cpp
  src/algorithms/ShortestPath.cpp
  src/algorithms/Tsp.cpp
  src/algorithms/MaxFlow.cpp
  src/algorithms/Mst.cpp
)

# --- CLI engine ---
add_executable(route_engine src/main.cpp)
target_link_libraries(route_engine PRIVATE drone_core)

# --- tests ---
enable_testing()
add_executable(unit_tests tests/test_main.cpp)
target_link_libraries(unit_tests PRIVATE drone_core)
add_test(NAME unit_tests COMMAND unit_tests)
