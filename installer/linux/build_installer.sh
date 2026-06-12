#!/usr/bin/env bash
# Assemble the Linux installer archive from staged build outputs.
set -euo pipefail

VERSION="${1:-0.0.0}"
REPO_ROOT="$(cd "$(dirname "$0")/../.." && pwd)"
STAGE_PLUGIN="${REPO_ROOT}/stage/plugin"
STAGE_STANDALONE="${REPO_ROOT}/stage/standalone"
WORK="${REPO_ROOT}/installer/linux/.work"
DIST="${REPO_ROOT}/dist"
ARCHIVE="g1000nxi-installer-linux-${VERSION}.tar.gz"

rm -rf "${WORK}"
mkdir -p "${WORK}/installer" "${DIST}"

cp -a "${STAGE_PLUGIN}/xplane-avionics" "${WORK}/installer/plugin/"
mkdir -p "${WORK}/installer/standalone"
cp -a "${STAGE_STANDALONE}/." "${WORK}/installer/standalone/"
cp "${REPO_ROOT}/installer/linux/install.sh" "${WORK}/installer/"
cp "${REPO_ROOT}/installer/linux/g1000-nxi.desktop" "${WORK}/installer/"
cp "${REPO_ROOT}/installer/linux/g1000-nxi.png" "${WORK}/installer/"
chmod +x "${WORK}/installer/install.sh"

tar -C "${WORK}" -czf "${DIST}/${ARCHIVE}" installer
echo "Wrote ${DIST}/${ARCHIVE}"
