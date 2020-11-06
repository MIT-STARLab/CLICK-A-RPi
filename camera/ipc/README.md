Reference project for Inter-process Communication (C++) using ZeroMQ
C++ implementation: cppzmq

# Build instructions

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
   
####Compilation

Via CMake (configuration in CMakeLists.txt)

```
cmake_minimum_required(VERSION 3.0 FATAL_ERROR)
   
   project(click-cpp-demo CXX)
   
   find_package(cppzmq)
   
   enable_testing()
   add_executable(
       click-cpp-demo
       main.cpp
       )
   
   target_link_libraries(
       click-cpp-demo
       cppzmq
       )
   
   add_test(
     NAME
       click-cpp-demo
     COMMAND
       ${CMAKE_BINARY_DIR}/${CMAKE_INSTALL_BINDIR}/click-cpp-demo
     )
```

- \# ``mkdir build``
- \# ``cd build/``
- \# ``cmake ..``
- \# ``make``

will create ``click-cpp-demo`` executable

