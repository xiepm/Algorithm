#!/usr/bin/env bash

set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
BUILD_DIR="${BUILD_DIR:-${ROOT_DIR}/build}"
BUILD_TYPE="${BUILD_TYPE:-Release}"
GENERATOR="${CMAKE_GENERATOR:-}"

cmake_args=(
  -S "${ROOT_DIR}"
  -B "${BUILD_DIR}"
  -DCMAKE_BUILD_TYPE="${BUILD_TYPE}"
)

if [[ -n "${GENERATOR}" ]]; then
  cmake_args+=(-G "${GENERATOR}")
fi

cmake "${cmake_args[@]}"
cmake --build "${BUILD_DIR}" --parallel "${BUILD_JOBS:-$(getconf _NPROCESSORS_ONLN 2>/dev/null || echo 2)}"

echo
echo "Build complete."
echo "  Shared library: ${BUILD_DIR}/libCmpHansCobotAlgorithm.so"
echo "  Test executable: ${BUILD_DIR}/testDemo"
