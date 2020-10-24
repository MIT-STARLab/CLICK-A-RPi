Reference project for Inter-process Communication (Python, C++) using ZeroMQ

Python implementation: zmq

C++ implementation: cppzmq

Main Python script creates multiple sockets to communicate to other processes.

### Files

- **chat.py** \
  send manual commands \
  interfaces with ``click-demo.py``

- **click-demo.py** \
  command handler process \
  interfaces with ``click-py-demo.py`` and ``click-cpp-demo``
  
- **python-process/click-py-demo.py** \
  example Python process

- **cpp-process/build/click-cpp-demo** \
  example C++ process

# Build instructions
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

# Keep-alive

Configure executables as services

1. Create a user-level systemd config folder
   - \# ``mkdir ~/.config/systemd/user/``

2. Create systemd scripts 
   - for each executable (``click-cpp-demo.service``)
    ```
     [Unit]
     Description=Manage Service click-cpp-demo
     
     [Service]
     ExecStart=/home/pi/CLICK-A/github/IPC-demo/cpp-process/build/click-cpp-demo
     Restart=always
     RestartSec=3
     
     [Install]
     WantedBy=default.target
   ```
      
   - and each python script (``click-py-demo.service``)
    ```
     [Unit]
     Description=Manage Service click-py-demo
     
     [Service]
     ExecStart=/usr/bin/python3 /home/pi/CLICK-A/github/IPC-demo/python-process/click-py-demo.py
     Restart=always
     RestartSec=3
     
     [Install]
     WantedBy=default.target
   ```
   
   - and click-demo.py script (``click-demo.service``)
    ```
     [Unit]
     Description=Manage Service click-demo
     
     [Service]
     ExecStart=/usr/bin/python3 /home/pi/CLICK-A/github/IPC-demo/click-demo.py
     Restart=always
     RestartSec=3
     
     [Install]
     WantedBy=default.target
   ```
   
   (please adjust folder structure accordingly)

3. Auto-start services on boot
   - enable lingering for user pi \
   \# ``loginctl enable-linger pi``
   - enable each service \
   \# ``systemctl --user enable click-cpp-demo``
   \# ``systemctl --user enable click-py-demo``
   \# ``systemctl --user enable click-demo``, ...

4. Start the services
   - \# ``systemctl --user start click-cpp-demo``
   - \# ``systemctl --user start click-py-demo``
   - \# ``systemctl --user start click-demo``
   - ...
   
# Inter-process Communication

