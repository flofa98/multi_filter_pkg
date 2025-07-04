#!/bin/bash

# Verzeichnis dieses Skripts ermitteln (egal von wo gestartet)
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"

# Install-Ordner relativ dazu
SETUP_PATH="${SCRIPT_DIR}/../../install/setup.bash"

echo ""
echo "⚠️  Bitte setzen Sie die Initialpose in RViz."
read -p "➡️  Weiter mit [Y/n]: " confirm

if [[ "$confirm" == "n" || "$confirm" == "N" ]]; then
  echo "❌ Abgebrochen."
  exit 0
fi

echo "✅ Initialpose bestätigt – starte waypoint_follower..."

gnome-terminal -- bash -c "source ${SETUP_PATH} && ros2 run multi_filter_pkg waypoint_follower; exec bash"

