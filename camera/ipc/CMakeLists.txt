cmake_minimum_required(VERSION 3.0 FATAL_ERROR)

project(camera-ipc CXX)

find_package(cppzmq)

enable_testing()
add_executable(
    camera-ipc
    main.cpp
    )

target_link_libraries(
    camera-ipc
    cppzmq
    )

add_test(
  NAME
    camera-ipc
  COMMAND
    ${CMAKE_BINARY_DIR}/${CMAKE_INSTALL_BINDIR}/camera-ipc
  )
