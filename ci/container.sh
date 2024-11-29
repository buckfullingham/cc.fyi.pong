#!/usr/bin/env bash

set -e

BUILD_ROOT="${BUILD_ROOT:-"$(readlink -f "$(dirname "$0")/..")"}"
BUILD_PROFILE="${BUILD_PROFILE:-emscripten}"

test -d "$BUILD_ROOT"
test -d "$BUILD_ROOT/ci/$BUILD_PROFILE"

apt update -vy
apt install -vy docker

docker build -t "$BUILD_PROFILE" "$BUILD_ROOT/ci/$BUILD_PROFILE"

exec docker run --interactive -v "${BUILD_ROOT}:${BUILD_ROOT}" -u 0 -e 'BUILD_*' "$BUILD_PROFILE" "$@"
