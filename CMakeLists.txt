cmake_minimum_required(VERSION 2.8)
project(GAMMA)

include(CTest)

add_subdirectory(src)
add_subdirectory(examples)

include(CPack)
