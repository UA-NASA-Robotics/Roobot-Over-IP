cmake_minimum_required(VERSION 3.5)

project(roobot-o-eth LANGUAGES CXX)

set(CMAKE_INCLUDE_CURRENT_DIR ON)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_INCLUDE_CURRENT_DIR ON)


include_directories(lib) #lib library

file(GLOB lib_fls "../lib/*.cpp" "../lib/*.h" "../lib/*.hpp")
set(LIB_SRCS ${lib_fls})

include_directories(SBC) #src

file(GLOB src_fls "lib/*.cpp" "lib/*.h" "lib/*.hpp" "src/*.cpp" "src/*.h" "src/*.hpp" "lib/moduleVirtualizations/*.cpp" "lib/moduleVirtualizations/*.h" "lib/moduleVirtualizations/*.hpp")
set(SRC_SRCS ${src_fls})

add_library(lib ${LIB_SRCS})
add_library(SBC ${SRC_SRCS})
target_link_libraries(lib rt) #some kind of dependency tree
target_link_libraries(SBC lib) #on god im confused

add_executable(arduino-communication main.cpp)
target_link_libraries(arduino-communication serial)
target_link_libraries(arduino-communication lib)
target_link_libraries(arduino-communication src)