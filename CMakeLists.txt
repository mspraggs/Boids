cmake_minimum_required (VERSION 2.8.11)
enable_testing()
project (boids)

set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_AUTOMOC ON)

find_package(Qt5Gui REQUIRED)

set(CMAKE_CXX_FLAGS "-Wall -std=c++11 -O3")

set (SRC_DIR ${PROJECT_BINARY_DIR}/src)
set (INC_DIR ${PROJECT_BINARY_DIR}/include)
set (TEST_DIR ${PROJECT_BINARY_DIR}/tests)

file (GLOB
  lib_SRC
  ${SRC_DIR}/boid.cpp
  ${SRC_DIR}/utils.cpp)

file (GLOB
  exec_SRC
  ${SRC_DIR}/main.cpp)

file (GLOB
  test_SRC
  RELATIVE
  ${PROJECT_BINARY_DIR}
  ${TEST_DIR}/*.cpp)

include_directories (${INC_DIR})

add_library (boids-lib SHARED ${lib_SRC})
target_link_libraries (boids-lib)
set_target_properties(boids-lib PROPERTIES OUTPUT_NAME boids)

add_executable (boids ${exec_SRC})
target_link_libraries (boids boids-lib)

foreach ( testsourcefile ${test_SRC} )
  string( REPLACE ".cpp" "" testname ${testsourcefile} )
  add_executable( ${testname} ${testsourcefile} )
  target_link_libraries( ${testname} ${Boost_LIBRARIES} boids)
  add_test( NAME ${testname} COMMAND ${testname} --log_level=all)
endforeach ( testsourcefile ${test_SRC} )

