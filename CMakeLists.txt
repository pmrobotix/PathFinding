cmake_minimum_required(VERSION 3.21)
project(pmr-pathfinding LANGUAGES CXX)

add_library(pmr-pathfinding STATIC
    src/pmr_edge.cpp
    src/pmr_node.cpp
    src/pmr_pathfinding.cpp
    src/pmr_playground.cpp
    src/pmr_point.cpp
    src/pmr_symmetrical_pg.cpp
    src/pmr_tools.cpp
    src/pmr_zone.cpp
)

target_include_directories(pmr-pathfinding PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/src
)
