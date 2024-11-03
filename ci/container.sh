#!/usr/bin/env bash

BUILD_ROOT="${BUILD_ROOT:-"$(readlink -f "$(dirname "$0")/..")"}"

apt update -vy
apt install -vy docker

exec docker run --interactive -v "${BUILD_ROOT}:${BUILD_ROOT}" -u 0 emscripten/emsdk "$@"
