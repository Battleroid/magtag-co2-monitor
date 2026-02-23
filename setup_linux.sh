#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
VENV_DIR="$ROOT_DIR/.venv"

log() {
  echo "[setup] $*"
}

have_cmd() {
  command -v "$1" >/dev/null 2>&1
}

install_with_pkg_manager() {
  local pkgs=("$@")

  if have_cmd apt-get; then
    sudo apt-get update
    sudo apt-get install -y "${pkgs[@]}"
    return 0
  fi

  if have_cmd dnf; then
    sudo dnf install -y "${pkgs[@]}"
    return 0
  fi

  if have_cmd pacman; then
    sudo pacman -Sy --noconfirm "${pkgs[@]}"
    return 0
  fi

  if have_cmd zypper; then
    sudo zypper --non-interactive install "${pkgs[@]}"
    return 0
  fi

  return 1
}

if [[ "${OSTYPE:-}" != linux* ]]; then
  echo "This setup script currently supports Linux only."
  exit 1
fi

if ! have_cmd python3; then
  echo "python3 is required but not found."
  if have_cmd sudo; then
    log "Attempting to install python3 via system package manager..."
    install_with_pkg_manager python3 python3-venv python3-pip || {
      echo "Could not auto-install python3. Please install python3, python3-venv, and pip manually."
      exit 1
    }
  else
    echo "Install python3 + venv manually, then re-run this script."
    exit 1
  fi
fi

if [[ ! -d "$VENV_DIR" ]]; then
  log "Creating virtual environment in .venv"
  python3 -m venv "$VENV_DIR"
fi

log "Activating virtual environment"
# shellcheck disable=SC1090
source "$VENV_DIR/bin/activate"

log "Upgrading pip/setuptools/wheel"
python -m pip install --upgrade pip setuptools wheel

log "Installing Python tooling (PlatformIO + Pillow)"
python -m pip install --upgrade platformio pillow

log "Verifying PlatformIO install"
pio --version

log "Pre-fetching PlatformIO platform/libraries"
pio pkg install -e magtag

if id -nG "$USER" | grep -qw dialout; then
  log "User '$USER' is in dialout group."
else
  cat <<'EOF'
[setup] NOTE: your user is not in the 'dialout' group.
[setup] Upload/serial may fail with permission denied until you run:
[setup]   sudo usermod -aG dialout "$USER"
[setup] Then log out and log back in.
EOF
fi

cat <<EOF

Setup complete.

Next steps:
  1) Activate env:  source .venv/bin/activate
  2) Build:         make build
  3) Upload:        make upload
  4) Monitor:       make monitor

If upload fails to find a serial port, check /dev/ttyACM* or /dev/ttyUSB*.
EOF
