#!/usr/bin/env bash

set -e

BUILD_TYPE="${BUILD_TYPE:-Release}"
BUILD_PROFILE="${BUILD_PROFILE:-emscripten}"
BUILD_ROOT="${BUILD_ROOT:-"$(readlink -f "$(dirname "$0")/..")"}"
BUILD_TYPE_LC="$(echo "$BUILD_TYPE" | tr '[:upper:]' '[:lower:]')"
BUILD_DIR="${BUILD_DIR:-"cmake-build-${BUILD_TYPE_LC}-${BUILD_PROFILE}"}"

if [ "$BUILD_PROFILE" == emscripten ]; then
  CMAKE="emcmake cmake"
  CMAKE_EMULATOR_SETTINGS=(
    -DCMAKE_CROSSCOMPILING_EMULATOR=node
  )
else
  CMAKE=cmake
  CMAKE_EMULATOR_SETTINGS=(
  )
fi

python3 --version
pip3 --version
$CMAKE --version

conan profile detect || true

CONAN_SETTINGS=(
  --profile="$BUILD_ROOT/ci/$BUILD_PROFILE/conan.profile"
  -s build_type="$BUILD_TYPE"
  -c tools.cmake.cmake_layout:build_folder_vars="['settings.build_type', 'settings.os']"
)

cd "$BUILD_ROOT"
conan install -of "$BUILD_DIR" --build=missing "${CONAN_SETTINGS[@]}" .

source ${BUILD_DIR}/conanbuildenv-${BUILD_TYPE_LC}*.sh

"$CC" --version
"$CXX" --version

# configure build
cd "$BUILD_DIR"
$CMAKE .. \
  -G "Unix Makefiles" \
  -DCMAKE_TOOLCHAIN_FILE=conan_toolchain.cmake \
  -DCMAKE_POLICY_DEFAULT_CMP0091=NEW \
  -DBUILD_PROFILE=$BUILD_PROFILE \
  -DCMAKE_BUILD_TYPE="$BUILD_TYPE" \
  "${CMAKE_EMULATOR_SETTINGS[@]}"

# put dependencies' dll's on LD_LIBRARY_PATH etc
source conanrun.sh

# build
cmake --build . --parallel

# run tests
ctest .

# package
cpack -G TGZ .

# explode package into github-pages dir for later deployment
mkdir -p github-pages
rm -rf github-pages/*
tar xvfz pong*.tar.gz -C github-pages --strip-components=1
