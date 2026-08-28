#!/usr/bin/env bash
# Fail if the OpenGL renderer is CPU software (llvmpipe) instead of NVIDIA.
set -euo pipefail

export DISPLAY="${DISPLAY:-:1}"

if ! command -v nvidia-smi >/dev/null 2>&1; then
  echo "GPU hard gate FAILED: nvidia-smi not found" >&2
  exit 1
fi
if ! command -v glxinfo >/dev/null 2>&1; then
  echo "GPU hard gate FAILED: glxinfo not found (install mesa-utils)" >&2
  exit 1
fi

nvidia-smi --query-gpu=name,driver_version,memory.total,memory.used --format=csv
echo "DISPLAY=${DISPLAY}"

renderer="$(glxinfo -B | sed -n 's/.*OpenGL renderer string: //p')"
vendor="$(glxinfo -B | sed -n 's/.*OpenGL vendor string: //p')"
echo "OpenGL renderer: ${renderer}"
echo "OpenGL vendor: ${vendor}"

if [[ "${renderer}" == *llvmpipe* ]] || [[ "${renderer}" != *NVIDIA* ]]; then
  echo "GPU hard gate FAILED: renderer must be NVIDIA, not llvmpipe" >&2
  exit 1
fi

echo "GPU hard gate passed"
