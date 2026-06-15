#!/usr/bin/env bash
set -euo pipefail

if [[ -z "${ORBBECSDK_ROOT:-}" ]]; then
    for candidate in /opt/OrbbecSDK_v2.8.6 /tmp/orbbecsdk_deb/opt/OrbbecSDK_v2.8.6; do
        if [[ -f "$candidate/shared/install_udev_rules.sh" ]]; then
            export ORBBECSDK_ROOT="$candidate"
            break
        fi
    done
fi

if [[ -z "${ORBBECSDK_ROOT:-}" || ! -f "$ORBBECSDK_ROOT/shared/install_udev_rules.sh" ]]; then
    echo "Orbbec SDK root not found. Set ORBBECSDK_ROOT to the SDK install path." >&2
    exit 1
fi

sudo "$ORBBECSDK_ROOT/shared/install_udev_rules.sh"
sudo udevadm control --reload
sudo udevadm trigger

echo "Orbbec udev rules installed. Replug the Gemini 336L if it is already connected."
