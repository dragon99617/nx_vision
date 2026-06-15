#!/usr/bin/env bash
set -euo pipefail

sudo usermod -aG dialout "$USER"
echo "Log out and log back in for dialout permissions to take effect."

