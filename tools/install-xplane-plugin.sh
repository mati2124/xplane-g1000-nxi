#!/usr/bin/env bash
# Build the G1000 NXi X-Plane plugin (and the Reload Plugins dev helper), copy
# them into X-Plane's plugins folder, and print what to do next.
#
# Usage:
#   tools/install-xplane-plugin.sh
#   XPLANE_DIR="/path/to/X-Plane 12" tools/install-xplane-plugin.sh
#
# After the first install, restart X-Plane once so both plugins are discovered.
# On later runs, rebuild + copy + Plugins → Reload Plugins is enough.

set -euo pipefail

ROOT="$(cd "$(dirname "$0")/.." && pwd)"
BUILD_DIR="${BUILD_DIR:-$ROOT/build}"
SDK_DIR="${XPLANE_SDK_DIR:-$ROOT/X-Plane-SDK}"

if [[ -n "${XPLANE_DIR:-}" ]]; then
  :
elif [[ -d "$HOME/X-Plane 12" ]]; then
  XPLANE_DIR="$HOME/X-Plane 12"
else
  echo "Set XPLANE_DIR to your X-Plane install (e.g. export XPLANE_DIR=\"\$HOME/X-Plane 12\")" >&2
  exit 1
fi

PLUGINS_DIR="$XPLANE_DIR/Resources/plugins"
if [[ ! -d "$PLUGINS_DIR" ]]; then
  echo "Plugins folder not found: $PLUGINS_DIR" >&2
  exit 1
fi

if [[ ! -d "$SDK_DIR/CHeaders/XPLM" ]]; then
  echo "X-Plane SDK not found at $SDK_DIR (set XPLANE_SDK_DIR)" >&2
  exit 1
fi

CMAKE="$(command -v cmake 2>/dev/null || true)"
if [[ -z "$CMAKE" && -x "$HOME/Library/Python/3.9/bin/cmake" ]]; then
  CMAKE="$HOME/Library/Python/3.9/bin/cmake"
fi
if [[ -z "$CMAKE" ]]; then
  echo "cmake not found on PATH" >&2
  exit 1
fi

case "$(uname -m)" in
  x86_64) ABI="mac_x64" ;;
  arm64)  ABI="mac_arm64" ;;
  *)
    echo "Unsupported macOS arch: $(uname -m)" >&2
    exit 1
    ;;
esac

echo "==> Configuring ($BUILD_DIR)"
"$CMAKE" -S "$ROOT" -B "$BUILD_DIR" \
  -DBUILD_XPLANE_SHELL=ON \
  -DXPLANE_SDK_DIR="$SDK_DIR" \
  >/dev/null

echo "==> Building xplane-avionics + reload-plugins"
"$CMAKE" --build "$BUILD_DIR" --target xplane-avionics reload-plugins -j"$(sysctl -n hw.ncpu 2>/dev/null || echo 4)"

RELOAD_SRC="$BUILD_DIR/tools/reload-plugins/reload-plugins.xpl"
RELOAD_DST="$PLUGINS_DIR/reload-plugins/$ABI/reload-plugins.xpl"
AVIONICS_DST="$PLUGINS_DIR/xplane-avionics/$ABI/xplane-avionics.xpl"
ASSETS_DST="$PLUGINS_DIR/xplane-avionics/assets"

echo "==> Installing xplane-avionics plugin + bundled assets"
"$CMAKE" --install "$BUILD_DIR" --component plugin --prefix "$PLUGINS_DIR"

mkdir -p "$(dirname "$RELOAD_DST")"
cp "$RELOAD_SRC" "$RELOAD_DST"

echo ""
echo "Installed:"
echo "  $AVIONICS_DST"
echo "  $ASSETS_DST/ (fonts, eis, checklists)"
echo "  $RELOAD_DST"
echo ""
echo "Per-aircraft overrides (no rebuild): drop files into the assets folder and"
echo "reload plugins — user-added files are left in place on later deploys."
echo "  $ASSETS_DST/eis/<icao>.eis"
echo "  $ASSETS_DST/checklists/<icao>.checklist"
echo "  (<icao> = aircraft acf_ICAO lowercased, e.g. tbm9, b738)"
echo ""
echo "In X-Plane:"
echo "  • First time only: fully quit and restart X-Plane (plugins are scanned at startup)."
echo "  • After code changes: Plugins → Reload Plugins (no full restart needed)."
echo "  • Toggle G1000 NXi on/off: Plugins → Plugin Admin → Enable/Disable Plugins."
