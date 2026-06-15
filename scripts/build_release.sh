#!/usr/bin/env bash
set -euo pipefail

if [[ -z "${ORBBECSDK_ROOT:-}" ]]; then
    for candidate in /opt/OrbbecSDK_v2.8.6 /tmp/orbbecsdk_deb/opt/OrbbecSDK_v2.8.6; do
        if [[ -f "$candidate/lib/OrbbecSDKConfig.cmake" ]]; then
            export ORBBECSDK_ROOT="$candidate"
            break
        fi
    done
fi

cmake_args=(-S . -B build -DCMAKE_BUILD_TYPE=Release)
if [[ -n "${ORBBECSDK_ROOT:-}" ]]; then
    cmake_args+=("-DORBBECSDK_ROOT=$ORBBECSDK_ROOT")
    cmake_args+=("-DOrbbecSDK_DIR=$ORBBECSDK_ROOT/lib")
fi

cmake "${cmake_args[@]}"
cmake --build build -j"$(nproc)"
