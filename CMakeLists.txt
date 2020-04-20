cmake_minimum_required(VERSION 3.16)
project(UltraTracer)

set(CMAKE_CXX_STANDARD 20)

add_executable(UltraTracer main.cpp)

set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -Wl,-stack_size -Wl,0x1000000")