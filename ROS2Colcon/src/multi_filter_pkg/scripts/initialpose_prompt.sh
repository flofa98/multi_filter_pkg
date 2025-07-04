#!/bin/bash

# Absoluter Pfad zu diesem Skript (auch bei ros2 launch korrekt)
SCRIPT_PATH="$(realpath "${BASH_SOURCE[0]}")"
SCRIPT_DIR="$(dirname "$SCRIPT_PATH")"

# Projekt-Wurzel ermitteln (zwei Ebenen hoch von scripts/)
PROJECT_ROOT="$(realpath "${SCRIPT_DIR}/../../..")"

# Setup-Skript
SETUP_PATH="${PROJECT_ROOT}/install/setup.bash"

# Debug-Ausgabe (optional)
echo "SCRIPT_DIR     = $SCRIPT_DIR"
echo "PROJECT_ROOT   = $PROJECT_ROOT"
echo "Sourcing setup = $SETUP_PATH"

# Benutzerabfrage
echo ""
echo "⚠️  Bitte setzen Sie die Initialpose in RViz."
read -p "➡️  Weiter mit [Y/n]: " confirm

if [[ "$confirm" == "n" || "$confirm" == "N" ]]; then
  echo "❌ Abgebrochen."
  exit 0
fi

# Node im neuen Terminal starten
echo "✅ Initialpose bestätigt – starte waypoint_follower..."

gnome-terminal -- bash -c "source '${SETUP_PATH}' && ros2 run multi_filter_pkg waypoint_nav_node; exec bash"


