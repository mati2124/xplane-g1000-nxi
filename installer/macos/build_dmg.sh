#!/usr/bin/env bash
# Build a macOS drag-and-drop DMG with the plugin payload, a .app bundle for the
# standalone shell, and a small installer script. Run from the repo root after
# `cmake --install` has populated stage/plugin and stage/standalone.
set -euo pipefail

VERSION="${1:-0.0.0}"
REPO_ROOT="$(cd "$(dirname "$0")/../.." && pwd)"
STAGE_PLUGIN="${REPO_ROOT}/stage/plugin"
STAGE_STANDALONE="${REPO_ROOT}/stage/standalone"
WORK="${REPO_ROOT}/installer/macos/.work"
DIST="${REPO_ROOT}/dist"
APP_NAME="G1000 NXi.app"
DMG_NAME="g1000nxi-installer-macos-${VERSION}.dmg"

if [[ ! -d "${STAGE_PLUGIN}/xplane-avionics/mac_x64" ]]; then
  echo "Missing staged plugin at ${STAGE_PLUGIN}/xplane-avionics/mac_x64" >&2
  exit 1
fi
if [[ ! -x "${STAGE_STANDALONE}/avionics-standalone" ]]; then
  echo "Missing staged standalone binary" >&2
  exit 1
fi

rm -rf "${WORK}"
mkdir -p "${WORK}/payload" "${DIST}"

# Plugin folder users copy into X-Plane/Resources/plugins/
cp -R "${STAGE_PLUGIN}/xplane-avionics" "${WORK}/payload/xplane-avionics"

# Standalone .app bundle (assets live in Contents/Resources/assets).
APP_DIR="${WORK}/payload/${APP_NAME}"
mkdir -p "${APP_DIR}/Contents/MacOS" "${APP_DIR}/Contents/Resources"
sed "s/VERSION_PLACEHOLDER/${VERSION}/g" \
  "${REPO_ROOT}/installer/macos/Info.plist" > "${APP_DIR}/Contents/Info.plist"
cp "${STAGE_STANDALONE}/avionics-standalone" "${APP_DIR}/Contents/MacOS/"
chmod +x "${APP_DIR}/Contents/MacOS/avionics-standalone"
cp -R "${STAGE_STANDALONE}/assets" "${APP_DIR}/Contents/Resources/"

# Double-clickable installer for the plugin.
cat > "${WORK}/payload/Install Plugin.command" <<'EOF'
#!/usr/bin/env bash
set -euo pipefail
DIR="$(cd "$(dirname "$0")" && pwd)"
read -r -p "Path to your X-Plane 12 folder: " XPLANE
XPLANE="${XPLANE/#\~/$HOME}"
if [[ ! -d "${XPLANE}/Resources/plugins" ]]; then
  echo "That folder does not contain Resources/plugins." >&2
  read -r -p "Press Enter to close."
  exit 1
fi
DEST="${XPLANE}/Resources/plugins/xplane-avionics"
mkdir -p "${DEST}"
ditto "${DIR}/xplane-avionics/" "${DEST}/"
echo "Installed plugin to ${DEST}"
read -r -p "Press Enter to close."
EOF
chmod +x "${WORK}/payload/Install Plugin.command"

# Optional desktop alias for the standalone app.
ln -sf "${APP_NAME}" "${WORK}/payload/Drag G1000 NXi to Applications"

cat > "${WORK}/payload/README.txt" <<EOF
G1000 NXi ${VERSION}

1. Plugin (in X-Plane):
   Double-click "Install Plugin.command" and choose your X-Plane 12 folder.
   Restart X-Plane (or use Plugins → Reload Plugins).

2. Standalone app:
   Drag "${APP_NAME}" to /Applications (or run it from this disk image).

Unsigned build: if macOS blocks the app, right-click → Open once.
EOF

hdiutil create -volname "G1000 NXi ${VERSION}" -srcfolder "${WORK}/payload" \
  -ov -format UDZO "${DIST}/${DMG_NAME}"
echo "Wrote ${DIST}/${DMG_NAME}"
