#!/usr/bin/env bash

set -e

BUILD_TYPE="${BUILD_TYPE:-Release}"
BUILD_PROFILE="${BUILD_PROFILE:-emscripten}"
BUILD_ROOT="${BUILD_ROOT:-"$(readlink -f "$(dirname "$0")/..")"}"
BUILD_TYPE_LC="$(echo "$BUILD_TYPE" | tr '[:upper:]' '[:lower:]')"
BUILD_DIR="${BUILD_DIR:-"cmake-build-${BUILD_TYPE_LC}-${BUILD_PROFILE}"}"

if [ "$BUILD_PROFILE" == emscripten ]; then
  CMAKE="emcmake cmake"
else
  CMAKE=cmake
fi

python3 --version
pip3 --version
$CMAKE --version

pip3 install -U -r "$BUILD_ROOT/ci/requirements.txt"

conan profile detect || true

CONAN_SETTINGS=(
  --profile="$BUILD_ROOT/ci/$BUILD_PROFILE/conan.profile"
  -s build_type="$BUILD_TYPE"
  -c tools.cmake.cmake_layout:build_folder_vars="['settings.build_type', 'settings.os']"
  -c tools.system.package_manager:mode=install
)

cd "$BUILD_ROOT"
conan install -of "$BUILD_DIR" --build=missing "${CONAN_SETTINGS[@]}" .

source ${BUILD_DIR}/conanbuildenv-${BUILD_TYPE_LC}*.sh

$CC --version
$CXX --version

# configure build
cd "$BUILD_DIR"
$CMAKE .. \
  -G "Unix Makefiles" \
  -DCMAKE_TOOLCHAIN_FILE=conan_toolchain.cmake \
  -DCMAKE_POLICY_DEFAULT_CMP0091=NEW \
  -DCMAKE_BUILD_TYPE="$BUILD_TYPE" \
  -DCMAKE_C_COMPILER=$CC \
  -DCMAKE_CXX_COMPILER=$CXX

# put dependencies' dll's on LD_LIBRARY_PATH etc
source conanrun.sh

# build
cmake --build . --parallel

# run tests
ctest .
