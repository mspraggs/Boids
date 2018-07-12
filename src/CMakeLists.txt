cmake_minimum_required(VERSION 2.8.11)
project(boids)

set(SRC
    boid.hpp
    raster_window.hpp
    swarm_application.hpp
    utils.hpp
    world.hpp

    boid.cpp
    main.cpp
    raster_window.cpp
    utils.cpp
    world.cpp)

add_executable(boids ${SRC})
target_link_libraries(boids ${Qt5Gui_LIBRARIES})