#!/usr/bin/env bash

set -e

export BUILD_ROOT="${BUILD_ROOT:-"$(readlink -f "$(dirname "$0")/..")"}"
export BUILD_PROFILE="${BUILD_PROFILE:-emscripten}"

test -d "$BUILD_ROOT"
test -d "$BUILD_ROOT/ci/$BUILD_PROFILE"

apt update -vy
apt install -vy docker

docker build \
  -t "$BUILD_PROFILE" \
  -f "$BUILD_ROOT/ci/$BUILD_PROFILE/Dockerfile" \
  "$BUILD_ROOT/ci"

exec docker run \
  -i \
  -v "${BUILD_ROOT}:${BUILD_ROOT}" \
  -u 0 \
  -e BUILD_ROOT \
  -e BUILD_PROFILE \
  "$BUILD_PROFILE" \
  "$@"
