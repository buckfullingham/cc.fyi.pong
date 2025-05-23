cmake_minimum_required(VERSION 3.20)
project(pong LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

set(CMAKE_THREAD_PREFER_PTHREAD TRUE)
set(THREADS_PREFER_PTHREAD_FLAG TRUE)
find_package(Threads REQUIRED)

set(CMAKE_INSTALL_PREFIX "/")
set(CMAKE_CXX_FLAGS "-Wall -Wextra -pedantic -Werror")
set(CMAKE_CXX_FLAGS_DEBUG "-O0 -g")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -DNDEBUG")
set(CMAKE_MODULE_PATH ${CMAKE_BINARY_DIR})

# conan / emscripten incompatibility : they both use cmake toolchain files
# so we need to provide our own method of distinguishing between emscripten
# and linux builds, for which we use the BUILD_PROFILE variable
add_compile_options(
        "$<$<STREQUAL:${BUILD_PROFILE},emscripten>:-fwasm-exceptions>"
)

add_link_options(
        "$<$<STREQUAL:${BUILD_PROFILE},emscripten>:-fwasm-exceptions>"
        "$<$<STREQUAL:${BUILD_PROFILE},emscripten>:-sASSERTIONS=2>"
        "$<$<STREQUAL:${BUILD_PROFILE},emscripten>:-Wno-linkflags>"
)

string(RANDOM LENGTH 1 ALPHABET "123456789" PRNG_SEED_PREFIX)
string(RANDOM LENGTH 5 ALPHABET "0123456789" PRNG_SEED_SUFFIX)
set(PRNG_SEED "${PRNG_SEED_PREFIX}${PRNG_SEED_SUFFIX}")
message(NOTICE "PRNG_SEED = ${PRNG_SEED}")

enable_testing()
add_subdirectory(src)
include(CPack)
