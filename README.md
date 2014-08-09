Boids
=====

Boids is a simplified version of the original Boids artificial life program
written by Craig Reynolds in 1986.

Installation
------------

Currently the only way to get the application running is to compile from source.
So far I've only tested this on Linux Ubuntu 14.04, but I expect this should
also work on other distros and OSes. To get started compiling you'll need:

* CMake 2.8.11 or greater
* A C++ compiler that can handle C++11 (I'm using GCC 4.8.2)
* Qt5 GUI 5.3 or greater (install qtbase5-dev on Debian-based distros, install qt5-qtbase-devel for Fedora, etc., or download and install from the Qt website)

Once you have these, you should be able to run:

    cmake .
    make
