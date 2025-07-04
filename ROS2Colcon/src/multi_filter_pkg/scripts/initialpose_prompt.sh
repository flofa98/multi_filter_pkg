#!/bin/bash

echo ""
echo "⚠️  Bitte setzen Sie die Initialpose in RViz."
read -p "➡️  Weiter mit [Y/n]: " confirm

if [[ "$confirm" == "n" || "$confirm" == "N" ]]; then
  echo "❌ Abgebrochen."
  exit 0
fi

echo "✅ Initialpose bestätigt – starte waypoint_follower..."
gnome-terminal -- bash -c "source ~/multi_filter_pkg/ROS2Colcon/install/setup.bash && ros2 run multi_filter_pkg waypoint_follower; exec bash"
