#!/usr/bin/env bash
# Decompress packaged X-Plane nav fixtures (apt.dat.gz, CIFP.tar.gz). Safe to
# run repeatedly; skips files that are already extracted.
set -euo pipefail

ROOT="$(cd "$(dirname "$0")/.." && pwd)"
FIXTURE="$ROOT/test-fixtures/xplane-navdata"
DEFAULT_DATA="$FIXTURE/Resources/default data"
APT_DIR="$FIXTURE/Global Scenery/Global Airports/Earth nav data"

if [[ ! -d "$FIXTURE" ]]; then
  echo "No fixture at $FIXTURE — run tools/package_xplane_navdata.sh locally first." >&2
  exit 1
fi

APT_GZ="$APT_DIR/apt.dat.gz"
APT="$APT_DIR/apt.dat"
if [[ -f "$APT_GZ" && ! -f "$APT" ]]; then
  echo "Decompressing apt.dat..."
  gunzip -k "$APT_GZ"
fi

CIFP_TAR="$DEFAULT_DATA/CIFP.tar.gz"
CIFP_DIR="$DEFAULT_DATA/CIFP"
if [[ -f "$CIFP_TAR" && ! -d "$CIFP_DIR" ]]; then
  echo "Extracting CIFP procedure database..."
  tar -xzf "$CIFP_TAR" -C "$DEFAULT_DATA"
fi

echo "Nav fixtures ready at: $FIXTURE"
