#!/usr/bin/env bash

# exit on error
set -e
set -x

BUILD_ROOT="${BUILD_ROOT:-"$(readlink -f "$(dirname "$0")/..")"}"
BUILD_TYPE="${BUILD_TYPE:-Release}"
BUILD_TYPE_LC="$(echo "$BUILD_TYPE" | tr '[:upper:]' '[:lower:]')"
BUILD_DIR="${BUILD_DIR:-"cmake-build-$BUILD_TYPE_LC"}"

export CC=emcc
export CXX=em++

# check python, pip & cmake are installed
python3 --version
pip3 --version
cmake --version

pip3 install -U -r "$BUILD_ROOT/ci/requirements.txt" || true

# create default profile if missing
conan profile detect || true

CONAN_SETTINGS=(
  -s os=Emscripten
  -s arch=wasm
  -s compiler=clang
  -s compiler.version=17
  -s compiler.libcxx=libc++
  -s build_type="$BUILD_TYPE"
)

cd "$BUILD_ROOT"
conan install -of "$BUILD_DIR" --build=missing "${CONAN_SETTINGS[@]}" .

# configure build
cd "$BUILD_DIR"
cmake .. -G "Unix Makefiles" -DCMAKE_TOOLCHAIN_FILE=conan_toolchain.cmake  -DCMAKE_POLICY_DEFAULT_CMP0091=NEW "-DCMAKE_BUILD_TYPE=$BUILD_TYPE"

# put dependencies' dll's on LD_LIBRARY_PATH etc
source conanrun.sh

# build
cmake --build . --parallel

# run tests
ctest .
