cmake_minimum_required (VERSION 2.8.11)
enable_testing()
project (boids)

set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_AUTOMOC ON)

find_package(Qt5Gui REQUIRED)

if (CMAKE_COMPILER_IS_GNUCXX)
  set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -std=c++11 -O3 -fPIC")
endif ()
if (CMAKE_COMPILER_IS_MSVC)
  set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /O2 /Wall")
endif ()

set (SRC_DIR ${PROJECT_BINARY_DIR}/src)
set (INC_DIR ${PROJECT_BINARY_DIR}/include)
set (TEST_DIR ${PROJECT_BINARY_DIR}/tests)

file (GLOB
  lib_SRC
  ${SRC_DIR}/boid.cpp
  ${SRC_DIR}/utils.cpp)

file (GLOB
  exec_SRC
  ${SRC_DIR}/main.cpp
  ${SRC_DIR}/raster_window.cpp)

file (GLOB
  test_SRC
  RELATIVE
  ${PROJECT_BINARY_DIR}
  ${TEST_DIR}/*.cpp)

include_directories (${INC_DIR} ${Qt5Gui_INCLUDE_DIRS})

add_library (boids-core OBJECT ${lib_SRC})

add_library (boids-shared SHARED $<TARGET_OBJECTS:boids-core>)
target_link_libraries (boids-shared)
set_target_properties(boids-shared PROPERTIES OUTPUT_NAME boids)

add_library (boids-static STATIC $<TARGET_OBJECTS:boids-core>)
target_link_libraries (boids-static)
set_target_properties(boids-static PROPERTIES OUTPUT_NAME boids)

add_executable (boids ${exec_SRC})
target_link_libraries (boids boids-shared ${Qt5Gui_LIBRARIES})

foreach ( testsourcefile ${test_SRC} )
  string( REPLACE ".cpp" "" testname ${testsourcefile} )
  add_executable( ${testname} ${testsourcefile} )
  target_link_libraries( ${testname} ${Boost_LIBRARIES} boids)
  add_test( NAME ${testname} COMMAND ${testname} --log_level=all)
endforeach ( testsourcefile ${test_SRC} )
