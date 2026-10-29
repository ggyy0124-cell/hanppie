cmake_minimum_required(VERSION 3.10)

project(hanppie VERSION 1.0.0 LANGUAGES CXX)

set(HANPPIE_TOP_LEVEL OFF)
if(CMAKE_SOURCE_DIR STREQUAL PROJECT_SOURCE_DIR)
  set(HANPPIE_TOP_LEVEL ON)
endif()

option(HANPPIE_BUILD_TESTS "Build the offline and loopback tests" ${HANPPIE_TOP_LEVEL})
option(HANPPIE_BUILD_EXAMPLES "Build the example programs" ${HANPPIE_TOP_LEVEL})

find_package(Threads REQUIRED)

add_library(hanppie
  src/connection.cpp
  src/ctrl.cpp
  src/lab.cpp
  src/md5.cpp
  src/media.cpp
  src/net.cpp
  src/product.cpp
  src/protocol.cpp
  src/robot.cpp
)
add_library(hanppie::hanppie ALIAS hanppie)
target_include_directories(hanppie PUBLIC
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_compile_features(hanppie PUBLIC cxx_std_11)
set_target_properties(hanppie PROPERTIES CXX_EXTENSIONS OFF)
target_link_libraries(hanppie PUBLIC Threads::Threads)
if(WIN32)
  target_link_libraries(hanppie PUBLIC ws2_32)
endif()
if(MSVC)
  target_compile_options(hanppie PRIVATE /W4 /utf-8)
elseif(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  target_compile_options(hanppie PRIVATE -Wall -Wextra -Wpedantic)
endif()

if(HANPPIE_BUILD_EXAMPLES)
  add_executable(hanppie_drive examples/drive.cpp)
  target_link_libraries(hanppie_drive PRIVATE hanppie::hanppie)
  add_executable(hanppie_lab examples/lab.cpp)
  target_link_libraries(hanppie_lab PRIVATE hanppie::hanppie)
endif()

if(HANPPIE_BUILD_TESTS)
  enable_testing()
  add_executable(hanppie_tests tests/test_main.cpp tests/test_loopback.cpp)
  target_include_directories(hanppie_tests PRIVATE tests src)
  target_link_libraries(hanppie_tests PRIVATE hanppie::hanppie)
  add_test(NAME hanppie_tests COMMAND hanppie_tests)
endif()

include(GNUInstallDirs)
install(TARGETS hanppie EXPORT hanppieTargets
  ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
  LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
)
install(DIRECTORY include/hanppie DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})
install(EXPORT hanppieTargets NAMESPACE hanppie:: DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/hanppie)
