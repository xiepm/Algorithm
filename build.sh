#!/usr/bin/env bash

set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
BUILD_DIR="${BUILD_DIR:-${ROOT_DIR}/build}"
BUILD_TYPE="${BUILD_TYPE:-Release}"
GENERATOR="${CMAKE_GENERATOR:-}"
INSTALL_DEPS="${INSTALL_DEPS:-0}"

log() {
  printf '[build] %s\n' "$*"
}

die() {
  printf '[build] %s\n' "$*" >&2
  exit 1
}

command_exists() {
  command -v "$1" >/dev/null 2>&1
}

is_ubuntu_or_debian() {
  [[ -r /etc/os-release ]] && grep -Eq '^(ID|ID_LIKE)=(.*debian|.*ubuntu)' /etc/os-release
}

have_eigen() {
  [[ -f "${ROOT_DIR}/external/eigen3/Eigen/Eigen" ]] \
    || [[ -f "${ROOT_DIR}/external/include/eigen3/Eigen/Eigen" ]] \
    || [[ -f "${ROOT_DIR}/include/eigen3/Eigen/Eigen" ]] \
    || [[ -f /usr/include/eigen3/Eigen/Eigen ]] \
    || [[ -f /usr/local/include/eigen3/Eigen/Eigen ]]
}

print_debian_help() {
  cat <<'EOF' >&2
[build] Missing build prerequisites.
[build] On Ubuntu/Debian, install them with:
[build]   sudo apt update
[build]   sudo apt install -y build-essential cmake libeigen3-dev
[build] Then rerun:
[build]   ./build.sh
[build] Or let the script install them automatically:
[build]   INSTALL_DEPS=1 ./build.sh
EOF
}

install_debian_deps() {
  command_exists sudo || die "Automatic dependency install requires 'sudo'."
  log "Installing build dependencies with apt."
  sudo apt update
  sudo apt install -y build-essential cmake libeigen3-dev
}

ensure_prerequisites() {
  local missing=0

  if ! command_exists cmake; then
    log "Missing required command: cmake"
    missing=1
  fi

  if ! command_exists c++; then
    log "Missing required command: c++"
    missing=1
  fi

  if ! command_exists cc; then
    log "Missing required command: cc"
    missing=1
  fi

  if ! have_eigen; then
    log "Missing Eigen headers (expected Eigen/Eigen)."
    missing=1
  fi

  if [[ "${missing}" -eq 0 ]]; then
    return 0
  fi

  if is_ubuntu_or_debian; then
    if [[ "${INSTALL_DEPS}" == "1" ]]; then
      install_debian_deps
      have_eigen || die "Eigen is still unavailable after dependency installation."
      command_exists cmake || die "cmake is still unavailable after dependency installation."
      command_exists c++ || die "c++ is still unavailable after dependency installation."
      command_exists cc || die "cc is still unavailable after dependency installation."
      return 0
    fi

    print_debian_help
  fi

  die "Build prerequisites are missing."
}

ensure_prerequisites

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
