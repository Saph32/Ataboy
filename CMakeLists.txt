cmake_minimum_required(VERSION 3.1.0 FATAL_ERROR)

project (ataboy)

set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)

find_package(SDL2 REQUIRED)
include_directories(${SDL2_INCLUDE_DIR})

set(SOURCE_FILES Main.cpp GB.cpp GB.h)

set(GCC_COMPILE_FLAGS "-Wall")
set(GCC_LINK_FLAGS    "")

set( CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS} ${GCC_COVERAGE_COMPILE_FLAGS}" )
set( CMAKE_EXE_LINKER_FLAGS  "${CMAKE_EXE_LINKER_FLAGS} ${GCC_COVERAGE_LINK_FLAGS}" )

add_executable(ataboy ${SOURCE_FILES})
target_link_libraries(ataboy ${SDL2_LIBRARY})
set_property(TARGET ataboy PROPERTY CXX_STANDARD 14)

