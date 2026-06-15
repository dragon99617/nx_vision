#!/usr/bin/env bash
set -euo pipefail

sudo apt-get update
sudo apt-get install -y build-essential cmake pkg-config libopencv-dev

echo "Install Orbbec SDK v2 separately if you want native depth capture."

