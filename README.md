# CLICK-A-RPi
CPU software for the CLICK-A mission.

# ZMQ Build instructions for Test RPi
## Python

1. Install Prerequisites
   - \# ``pip3 install pyzmq``

2. Make scripts executable
   - via \# ``chmod +x *.py``

3. Force the Python 3
   - add ``#!/usr/bin/env python3`` as first line in your scripts

## C++
###Prerequisites
- libbsd-dev
- libsodium-dev
- asciidoc
- cmake
- libtool
- libzmq ([github.com/zeromq/libzmq](https://github.com/zeromq/libzmq))
- cppzmq ([github.com/zeromq/cppzmq](https://github.com/zeromq/cppzmq))

####Build Steps

1. Install Prerequisites
   - \# ``sudo apt-get install libbsd-dev libsodium-dev asciidoc cmake libtool``

2. Build [libzmq](https://github.com/zeromq/libzmq) via cmake. This does an out of source build and installs the build files
   - download and unzip the lib, cd to directory
   - mkdir build
   - cd build
   - cmake ..
   - sudo make -j4 install

3. Build [cppzmq](https://github.com/zeromq/cppzmq) via cmake. This does an out of source build and installs the build files
   - download and unzip the lib, cd to directory
   - mkdir build
   - cd build
   - cmake .. 
   - if you get this error, "CMake Error at tests/CMakeLists.txt", try: cmake -DCPPZMQ_BUILD_TESTS=OFF ..
   - sudo make -j4 install
