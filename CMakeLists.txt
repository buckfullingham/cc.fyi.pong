cmake_minimum_required(VERSION 3.20)
project(pong LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

set(CMAKE_THREAD_PREFER_PTHREAD TRUE)
set(THREADS_PREFER_PTHREAD_FLAG TRUE)
find_package(Threads REQUIRED)

set(CMAKE_CXX_FLAGS "-Wall -Wextra -Werror")
set(CMAKE_CXX_FLAGS_DEBUG "-O0 -g")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -DNDEBUG")
set(CMAKE_MODULE_PATH ${CMAKE_BINARY_DIR})

if (EMSCRIPTEN)
    add_compile_options(-fwasm-exceptions -sASSERTIONS)
    add_link_options(-fwasm-exceptions -sASSERTIONS)
endif()

enable_testing()
add_subdirectory(src)
