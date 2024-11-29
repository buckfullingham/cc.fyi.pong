[settings]
os=Emscripten
arch=wasm
compiler=clang
compiler.version=17
compiler.cppstd=23
compiler.libcxx=libc++

[buildenv]
CC=emcc
CXX=em++
# work around https://github.com/glfw/glfw/issues/2139
CFLAGS=-DPOSIX_REQUIRED_STANDARD=199309L -D_POSIX_C_SOURCE=POSIX_REQUIRED_STANDARD -D_POSIX_SOURCE=POSIX_REQUIRED_STANDARD
