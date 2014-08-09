Boids
=====

Boids is a simplified version of the original Boids artificial life program
written by Craig Reynolds in 1986.

Compiling on Linux
------------------

Currently the only way to get the application running is to compile from source.
So far I've only tested this on Ubuntu 14.04, but I expect this should also work
on other distros. To get started compiling you'll need:

* CMake 2.8.11 or greater
* A C++ compiler that can handle C++11 (I'm using GCC 4.8.2)
* Qt5 GUI 5.3 or greater (install qtbase5-dev on Debian-based distros, install qt5-qtbase-devel for Fedora, etc., or download and install from the Qt website)

Once you have these, you should be able to run:

    cmake .
    make

Compling on Windows
-------------------

Compiling on Windows is a bit more involved because of the way Windows handles
(or rather, doesn't handle) dependencies. You'll need all the packages described
in the "Compiling on Linux" section above. By far the easiest way to compile
is by using Visual Studio (the express edition is free). Once you've downloaded
this and the above packages do the following:

1. Install Visual Studio.
2. Install Qt. Make sure to select the package in the list you're given that best matches the version of Visual Studio you're using (i.e. version number, 32/65 bit etc.)
3. Install CMake.
4. Run CMake. Select the source folder and specify an output binary folder. Hit generate and a dialogue should appear. Select the Visual Studio version that most closely matches the one you installed then hit ok.
5. CMake will probably fail to find Qt at this point, so in the empty field next to the variable that isn't set, specify the path to the cmake folder in Qt (usually this looks something like C:\Path\To\Qt\5.3\msvcXXXX\lib\cmake\Qt5Gui). You might need to adjust the last folder in this path to point to the library the variable corresponds to. Again msvcXXXX should roughly match the version of Visual Studio you're using.
6. Hit generate again. A Visual Studio solution should be generated.
7. Open the solution and build the project.

Since Visual Studio 2012, the paths set up in the solution are not set up
correctly, so when you run the executable it won't find the required Qt DLLs.
To fix this, you'll have to find the DLLs in C:\Path\To\Qt\5.3\msvcXXXX\bin and
copy each of the required DLLs into the directory the executable is in.

Compiling with Mingw is also possible, but you need to know which compiler
packages to install (at least g++), and when I did it I had to disable my
antivirus to allow CMake to execute batch files when testing the compiler.