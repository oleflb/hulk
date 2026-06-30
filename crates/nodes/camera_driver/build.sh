#!/usr/bin/env bash

set -euo pipefail

SCRIPT_DIRECTORY="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
REPOSITORY_ROOT="$(cd -- "${SCRIPT_DIRECTORY}/../../.." && pwd)"

CONTAINER_RUNTIME="${CONTAINER_RUNTIME:-podman}"
IMAGE_NAME="${CAMERA_DRIVER_IMAGE:-camera-driver-x5}"
CARGO_CACHE_HOME="${CARGO_CACHE_HOME:-$("${REPOSITORY_ROOT}/scripts/resolve_data_home")/container-cargo-home}"

mkdir -p "${CARGO_CACHE_HOME}/git" "${CARGO_CACHE_HOME}/registry"

"${CONTAINER_RUNTIME}" build \
    -t "${IMAGE_NAME}" \
    -f "${SCRIPT_DIRECTORY}/Containerfile" \
    "${REPOSITORY_ROOT}"

"${CONTAINER_RUNTIME}" run --rm \
    --volume "${REPOSITORY_ROOT}":/work:z \
    --volume "${CARGO_CACHE_HOME}/git":/root/.cargo/git:z \
    --volume "${CARGO_CACHE_HOME}/registry":/root/.cargo/registry:z \
    "${IMAGE_NAME}" \
    cargo build -r -p camera_driver --target aarch64-unknown-linux-gnu "$@"
