
set(APP_NAME pmr_pathfinding)
project(${APP_NAME})
cmake_minimum_required(VERSION 2.8)

file(GLOB CPP_FILES src/*.cpp src/*.h)
add_library(${PROJECT_NAME} STATIC ${CPP_FILES})

include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include)
