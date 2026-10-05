cmake_minimum_required(VERSION 3.16)
project(PhysicsSandbox LANGUAGES CXX VERSION 0.1)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)     # for clangd

# Default to Release in single-config generators
if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "" FORCE)
endif()

# Optional sanitizer build
option(SANITIZE "Enable ASan/UBSan" OFF)

# ... project() etc ...

if(MSVC)
  target_compile_options(sandbox PRIVATE /W4 /permissive- /Zc:__cplusplus)
else()
  target_compile_options(sandbox PRIVATE -Wall -Wextra -Wpedantic -Wshadow)
  target_compile_options(physics_tests PRIVATE -Wall -Wextra -Wpedantic -Wshadow)
endif()

if(SANITIZE AND NOT MSVC)
  target_compile_options(sandbox PRIVATE -fsanitize=address,undefined -fno-omit-frame-pointer)
  target_link_options(sandbox PRIVATE -fsanitize=address,undefined)
endif()

# Optional: FetchContent raylib if not found
# find_package(raylib QUIET)
# if(NOT raylib_FOUND) ... FetchContent_Declare(raylib ...) ... endif()