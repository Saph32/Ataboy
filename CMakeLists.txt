cmake_minimum_required(VERSION 2.8.7 FATAL_ERROR)

project (ataboy)

set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)

find_package(SDL2 REQUIRED)
include_directories(${SDL2_INCLUDE_DIR})

set(Boost_USE_STATIC_LIBS ON)
FIND_PACKAGE( Boost 1.54 COMPONENTS filesystem system REQUIRED )
INCLUDE_DIRECTORIES( ${Boost_INCLUDE_DIR} )

find_package(PNG REQUIRED)
include_directories(${PNG_INCLUDE_DIR})

include_directories(${PROJECT_SOURCE_DIR}/extern)

set(SOURCE_FILES
    Main.cpp
    GB.cpp
    GB.h
	GB_CPU.h
	GB_Video.h
	GB_Audio.h
	GB_IO.h
	GB_System.h
	GB_impl.hpp
	GB_CPU_impl.hpp
	GB_Video_impl.hpp
	GB_Audio_impl.hpp
	GB_IO_impl.hpp
	GB_System_impl.hpp
    FileIo.cpp
    FileIo.h
	ImageIo.cpp
	ImageIo.h
	at_scope_exit.h
	ProgramOptions.h
	ProgramOptions.cpp
	extern/cxxopts/cxxopts.hpp)

set(GCC_COMPILE_FLAGS "-Wall")
set(GCC_LINK_FLAGS    "")

set( CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS} ${GCC_COVERAGE_COMPILE_FLAGS} -std=c++1z" )
set( CMAKE_EXE_LINKER_FLAGS  "${CMAKE_EXE_LINKER_FLAGS} ${GCC_COVERAGE_LINK_FLAGS}" )

# the compiler flags for compiling C++ sources
MESSAGE( STATUS "CMAKE_CXX_FLAGS: " ${CMAKE_CXX_FLAGS} )

add_executable(ataboy ${SOURCE_FILES})
target_link_libraries(ataboy ${SDL2_LIBRARY} ${Boost_LIBRARIES} ${PNG_LIBRARY})
#set_property(TARGET ataboy PROPERTY CXX_STANDARD 17)

