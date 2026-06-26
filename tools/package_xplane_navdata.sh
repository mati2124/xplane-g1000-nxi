#!/usr/bin/env bash
# Copy X-Plane 12 navigation databases into test-fixtures/xplane-navdata for
# standalone-shell / cloud-agent use (--nav-data-dir). Large blobs are stored
# compressed (apt.dat.gz, CIFP.tar.gz) to stay under git size limits.
set -euo pipefail

ROOT="$(cd "$(dirname "$0")/.." && pwd)"
DEST="$ROOT/test-fixtures/xplane-navdata"
XP_ROOT="${1:-}"

if [[ -z "$XP_ROOT" ]]; then
  PREFS="$HOME/Library/Preferences/x-plane_install_12.txt"
  if [[ -f "$PREFS" ]]; then
    XP_ROOT="$(head -1 "$PREFS" | tr -d '\r')"
  fi
fi

if [[ -z "$XP_ROOT" || ! -d "$XP_ROOT" ]]; then
  echo "Usage: $0 [X-Plane-12-install-root]" >&2
  echo "Could not find X-Plane install (pass path or set x-plane_install_12.txt)." >&2
  exit 1
fi

DEFAULT_DATA="$XP_ROOT/Resources/default data"
APT_SRC="$XP_ROOT/Global Scenery/Global Airports/Earth nav data/apt.dat"

for f in earth_nav.dat earth_fix.dat earth_aptmeta.dat earth_awy.dat \
         earth_hold.dat earth_mora.dat earth_msa.dat; do
  if [[ ! -f "$DEFAULT_DATA/$f" ]]; then
    echo "Missing $DEFAULT_DATA/$f" >&2
    exit 1
  fi
done
if [[ ! -f "$APT_SRC" ]]; then
  echo "Missing $APT_SRC" >&2
  exit 1
fi

echo "Packaging nav data from: $XP_ROOT"
rm -rf "$DEST"
mkdir -p "$DEST/Resources/default data/airspaces"
mkdir -p "$DEST/Global Scenery/Global Airports/Earth nav data"

for f in earth_nav.dat earth_fix.dat earth_aptmeta.dat earth_awy.dat \
         earth_hold.dat earth_mora.dat earth_msa.dat; do
  cp "$DEFAULT_DATA/$f" "$DEST/Resources/default data/$f"
done

if [[ -f "$DEFAULT_DATA/airspaces/airspace.txt" ]]; then
  cp "$DEFAULT_DATA/airspaces/airspace.txt" \
    "$DEST/Resources/default data/airspaces/airspace.txt"
fi

tar -czf "$DEST/Resources/default data/CIFP.tar.gz" -C "$DEFAULT_DATA" CIFP
gzip -c "$APT_SRC" > "$DEST/Global Scenery/Global Airports/Earth nav data/apt.dat.gz"

cat > "$DEST/README.txt" <<EOF
X-Plane navigation databases copied from a local install for avionics-standalone
and Cursor cloud agents. Proprietary X-Plane data — do not redistribute.

Point the standalone shell at this tree:
  --nav-data-dir "$DEST"

Or run tools/install_test_navdata.sh to decompress apt.dat and CIFP before use.
EOF

echo "Done. Fixture root: $DEST"
du -sh "$DEST"
