cmake_minimum_required(VERSION 3.15)

project(MiniRenderer)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14")

# 设置 Debug时允许 gdb 调试 并且关闭编译器优化
SET(CMAKE_BUILD_TYPE "Debug")
SET(CMAKE_CXX_FLAGS_DEBUG "$ENV{CXXFLAGS} -O0 -Wall -g2 -ggdb")
SET(CMAKE_CXX_FLAGS_RELEASE "$ENV{CXXFLAGS} -O3 -Wall")

aux_source_directory(src DIR_SRCS)

aux_source_directory(src/canvas DIR_CANVAS)
aux_source_directory(src/util DIR_UTIL)
aux_source_directory(src/model DIR_MODEL)

add_subdirectory(src/lib/tga)

add_executable(
  MiniRenderer ${DIR_SRCS} ${DIR_CANVAS} ${DIR_MODEL} ${DIR_UTIL}
)

target_link_libraries(MiniRenderer TGA)
