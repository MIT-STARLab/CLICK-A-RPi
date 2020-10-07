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
