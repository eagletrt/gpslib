cmake_minimum_required(VERSION 3.22)

project(test)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
add_compile_options(-Wall -Wextra -Wpedantic -Werror)

include_directories(${CMAKE_CURRENT_LIST_DIR}/..)

find_package(Protobuf REQUIRED)

add_executable(test test.cpp ../gps.c ../gps_proto.cpp ../gps.pb.cc ../gps_interface.c)
target_link_libraries(test ${PROTOBUF_LIBRARIES})
