#!/usr/bin/env bash

set -e

BUILD_ROOT="${BUILD_ROOT:-"$(readlink -f "$(dirname "$0")/..")"}"
BUILD_PROFILE="${BUILD_PROFILE:-emscripten}"
BUILD_CONAN_VOLUME=${CONAN_VOLUME:-conan}
BUILD_ROOT_IN_CONTAINER=/tmp/"$(basename "$BUILD_ROOT")"
BUILD_IMAGE="${BUILD_IMAGE:-pong-"$BUILD_PROFILE"}"

test -d "$BUILD_ROOT"
test -d "$BUILD_ROOT/ci/$BUILD_PROFILE"

docker --version || (
  apt -y update && apt -y install docker
)

EMSDK_IMAGE_TAG=4.0.2

if [ "$(uname -m)" == arm64 ]; then
  EMSDK_IMAGE_TAG="${EMSDK_IMAGE_TAG}-arm64"
fi

BUILD_ARGS=(--build-arg EMSDK_IMAGE_TAG="$EMSDK_IMAGE_TAG")

if [ -t 1 ]; then
  TTY_ARG=(--tty)
else
  TTY_ARG=()
fi

docker build \
  -t "$BUILD_IMAGE" \
  "${BUILD_ARGS[@]}" \
  -f "$BUILD_ROOT/ci/$BUILD_PROFILE/Dockerfile" \
  "$BUILD_ROOT/ci"

docker volume create "${BUILD_CONAN_VOLUME}" || true

exec docker run \
  --rm \
  --interactive \
  --network=host \
  "${TTY_ARG[@]}" \
  -v "${BUILD_ROOT}:${BUILD_ROOT_IN_CONTAINER}" \
  -v "${BUILD_CONAN_VOLUME}:/mnt/conan" \
  -e CONAN_HOME=/mnt/conan \
  -e "BUILD_ROOT=${BUILD_ROOT_IN_CONTAINER}" \
  -e BUILD_TYPE \
  "$BUILD_IMAGE" \
  "$@"
