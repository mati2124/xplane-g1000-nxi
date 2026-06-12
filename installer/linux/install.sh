#!/usr/bin/env bash
# Interactive installer for Linux. Expects plugin/ and standalone/ directories
# beside this script (as laid out by build_installer.sh).
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
PLUGIN_SRC="${SCRIPT_DIR}/plugin/xplane-avionics"
STANDALONE_SRC="${SCRIPT_DIR}/standalone"

usage() {
  echo "Usage: $0 [--xplane DIR] [--standalone] [--desktop-icon] [--startup]"
  exit 1
}

XPLANE_DIR=""
INSTALL_STANDALONE=0
DESKTOP_ICON=0
START_AT_LOGIN=0

while [[ $# -gt 0 ]]; do
  case "$1" in
    --xplane) XPLANE_DIR="$2"; shift 2 ;;
    --standalone) INSTALL_STANDALONE=1; shift ;;
    --desktop-icon) DESKTOP_ICON=1; shift ;;
    --startup) START_AT_LOGIN=1; shift ;;
    -h|--help) usage ;;
    *) echo "Unknown option: $1" >&2; usage ;;
  esac
done

if [[ -z "${XPLANE_DIR}" ]]; then
  read -r -p "X-Plane 12 install folder: " XPLANE_DIR
fi
XPLANE_DIR="${XPLANE_DIR/#\~/$HOME}"

if [[ ! -d "${XPLANE_DIR}/Resources/plugins" ]]; then
  echo "Error: ${XPLANE_DIR} does not look like an X-Plane install." >&2
  exit 1
fi

if [[ ! -d "${PLUGIN_SRC}" ]]; then
  echo "Error: plugin payload missing at ${PLUGIN_SRC}" >&2
  exit 1
fi

PLUGIN_DEST="${XPLANE_DIR}/Resources/plugins/xplane-avionics"
mkdir -p "${PLUGIN_DEST}"
cp -a "${PLUGIN_SRC}/." "${PLUGIN_DEST}/"
echo "Installed plugin to ${PLUGIN_DEST}"

if [[ "${INSTALL_STANDALONE}" -eq 1 ]]; then
  if [[ ! -x "${STANDALONE_SRC}/avionics-standalone" ]]; then
    echo "Error: standalone payload missing" >&2
    exit 1
  fi
  STANDALONE_DEST="${HOME}/.local/share/g1000-nxi"
  mkdir -p "${STANDALONE_DEST}"
  cp -a "${STANDALONE_SRC}/." "${STANDALONE_DEST}/"
  chmod +x "${STANDALONE_DEST}/avionics-standalone"
  BIN_DIR="${HOME}/.local/bin"
  mkdir -p "${BIN_DIR}"
  ln -sf "${STANDALONE_DEST}/avionics-standalone" "${BIN_DIR}/avionics-standalone"
  echo "Installed standalone to ${STANDALONE_DEST}"

  # Install the app icon into the user's icon theme so the .desktop entry
  # (Icon=g1000-nxi) resolves in the menu / launcher / taskbar.
  if [[ -f "${SCRIPT_DIR}/g1000-nxi.png" ]]; then
    ICON_DIR="${HOME}/.local/share/icons/hicolor/256x256/apps"
    mkdir -p "${ICON_DIR}"
    cp "${SCRIPT_DIR}/g1000-nxi.png" "${ICON_DIR}/g1000-nxi.png"
    if command -v gtk-update-icon-cache >/dev/null 2>&1; then
      gtk-update-icon-cache -q -t "${HOME}/.local/share/icons/hicolor" || true
    fi
    echo "Installed app icon"
  fi

  if [[ "${DESKTOP_ICON}" -eq 1 ]]; then
    DESKTOP_DIR="${HOME}/.local/share/applications"
    DESKTOP_FILE="${DESKTOP_DIR}/g1000-nxi.desktop"
    mkdir -p "${DESKTOP_DIR}"
    sed "s|Exec=avionics-standalone|Exec=${STANDALONE_DEST}/avionics-standalone|" \
      "${SCRIPT_DIR}/g1000-nxi.desktop" > "${DESKTOP_FILE}"
    chmod +x "${DESKTOP_FILE}"
    if command -v update-desktop-database >/dev/null 2>&1; then
      update-desktop-database "${DESKTOP_DIR}" || true
    fi
    echo "Wrote ${DESKTOP_FILE}"
  fi

  if [[ "${START_AT_LOGIN}" -eq 0 ]]; then
    read -r -p "Start the standalone app automatically at login? [y/N] " REPLY
    [[ "${REPLY}" =~ ^[Yy]$ ]] && START_AT_LOGIN=1
  fi
  if [[ "${START_AT_LOGIN}" -eq 1 ]]; then
    # XDG autostart: a .desktop file in ~/.config/autostart launches at login.
    AUTOSTART_DIR="${HOME}/.config/autostart"
    AUTOSTART_FILE="${AUTOSTART_DIR}/g1000-nxi.desktop"
    mkdir -p "${AUTOSTART_DIR}"
    sed "s|Exec=avionics-standalone|Exec=${STANDALONE_DEST}/avionics-standalone|" \
      "${SCRIPT_DIR}/g1000-nxi.desktop" > "${AUTOSTART_FILE}"
    chmod +x "${AUTOSTART_FILE}"
    echo "Enabled start at login (${AUTOSTART_FILE})"
  fi
else
  read -r -p "Install standalone app too? [y/N] " REPLY
  if [[ "${REPLY}" =~ ^[Yy]$ ]]; then
    exec "$0" --xplane "${XPLANE_DIR}" --standalone \
      $([[ "${DESKTOP_ICON}" -eq 1 ]] && echo --desktop-icon) \
      $([[ "${START_AT_LOGIN}" -eq 1 ]] && echo --startup)
  fi
fi

echo "Done. Restart X-Plane or use Plugins → Reload Plugins."
