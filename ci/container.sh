#!/usr/bin/env bash

set -e

BUILD_ROOT="${BUILD_ROOT:-"$(readlink -f "$(dirname "$0")/..")"}"
BUILD_PROFILE="${BUILD_PROFILE:-emscripten}"
BUILD_CONAN_VOLUME=${CONAN_VOLUME:-conan}
BUILD_ROOT_IN_CONTAINER=/tmp/"$(basename "$BUILD_ROOT")"
BUILD_IMAGE="${BUILD_IMAGE:-pong-"$BUILD_PROFILE"}"

test -d "$BUILD_ROOT"
test -d "$BUILD_ROOT/ci/$BUILD_PROFILE"

apt update -vy || true
apt install -vy docker || true

docker build \
  -t "$BUILD_IMAGE" \
  -f "$BUILD_ROOT/ci/$BUILD_PROFILE/Dockerfile" \
  "$BUILD_ROOT/ci"

docker volume create "${BUILD_CONAN_VOLUME}" || true

exec docker run \
  -i \
  -u 0 \
  -v "${BUILD_ROOT}:${BUILD_ROOT_IN_CONTAINER}" \
  -v "${BUILD_CONAN_VOLUME}:/mnt/conan" \
  -e CONAN_HOME=/mnt/conan \
  -e "BUILD_ROOT=${BUILD_ROOT_IN_CONTAINER}" \
  -e BUILD_TYPE \
  "$BUILD_IMAGE" \
  "$@"
