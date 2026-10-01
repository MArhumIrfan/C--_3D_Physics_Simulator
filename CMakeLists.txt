cmake_minimum_required(VERSION 3.16)
project(PhysicsSandbox CXX)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(raylib REQUIRED)

add_executable(sandbox main.cpp)
target_link_libraries(sandbox raylib)

add_executable(physics_tests physics_tests.cpp)   # no raylib needed

if(MSVC)
  target_compile_options(sandbox PRIVATE /W4)
else()
  target_compile_options(sandbox PRIVATE -Wall -Wextra)
  target_compile_options(physics_tests PRIVATE -Wall -Wextra)
endif()

enable_testing()
add_test(NAME physics COMMAND physics_tests WORKING_DIRECTORY ${CMAKE_SOURCE_DIR})
