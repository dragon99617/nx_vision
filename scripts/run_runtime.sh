#!/usr/bin/env bash
set -euo pipefail

if [[ -z "${ORBBECSDK_ROOT:-}" ]]; then
    for candidate in /opt/OrbbecSDK_v2.8.6 /tmp/orbbecsdk_deb/opt/OrbbecSDK_v2.8.6; do
        if [[ -d "$candidate/lib" ]]; then
            export ORBBECSDK_ROOT="$candidate"
            break
        fi
    done
fi
if [[ -n "${ORBBECSDK_ROOT:-}" ]]; then
    export LD_LIBRARY_PATH="$ORBBECSDK_ROOT/lib:$ORBBECSDK_ROOT/lib/extensions/depthengine:$ORBBECSDK_ROOT/lib/extensions/filters:$ORBBECSDK_ROOT/lib/extensions/firmwareupdater:$ORBBECSDK_ROOT/lib/extensions/frameprocessor:${LD_LIBRARY_PATH:-}"
fi

./build/nx_runtime config
