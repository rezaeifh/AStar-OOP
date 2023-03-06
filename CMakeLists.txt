cmake_minimum_required (VERSION 2.8.11)
project (a-star)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
set(CMAKE_EXE_LINKER_FLAGS "-static-libgcc -static-libstdc++ -static")
include_directories(source)
add_library(a-star source/AStar.cpp source/Euclidean.cpp source/Manhattan.cpp source/Octagonal.cpp source/Diagonal.cpp)

add_executable(main main.cpp)
target_link_libraries(main a-star)
