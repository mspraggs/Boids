cmake_minimum_required (VERSION 2.8.11)
enable_testing()
project (boids)

include (CPack)

set(CMAKE_AUTOMOC ON)

find_package(Qt5Gui REQUIRED)

if (CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID STREQUAL "Clang")
  set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g -Wall -std=c++11 -fPIC")
endif ()
if (CMAKE_COMPILER_IS_MSVC)
  set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /O2 /Wall")
endif ()

#TODO: Fix this to use FindEigen3.cmake
include_directories (${INC_DIR} ${Qt5Gui_INCLUDE_DIRS} /usr/include/eigen3)

add_subdirectory(src)
