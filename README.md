## 🧭 ROS 2 Waypoint Navigation mit Nav2 und TurtleBot3


### Voraussetzungen

* ROS 2 installiert (z. B. ROS 2 Jazzy)
* TurtleBot3-Pakete installiert
* Nav2 installiert
* Das Projekt mit der Datei `waypoint_nav_node.cpp` ist als Paket `multi_filter_pkg` eingebunden
* Gazebo ist korrekt konfiguriert

---

### 🔧 1. Simulation und Nav2 starten

Starte die TurtleBot3-Simulation mit aktivem Nav2-Stack und RViz:

```bash
ros2 launch nav2_bringup tb3_simulation_launch.py headless:=False
```

* Dadurch wird eine Gazebo-Welt geladen
* Nav2 (inkl. AMCL, BT Navigator etc.) wird gestartet
* RViz öffnet sich automatisch zur Visualisierung

---

### 📍 2. Initialpose setzen

In RViz:

1. Wähle das Tool **2D Pose Estimate** (Pfeilsymbol mit "Initial Pose") aus der oberen Leiste
2. Klicke in der Karte auf die ungefähre Startposition des Roboters und ziehe den Pfeil in Fahrtrichtung

Dies setzt die Anfangsposition des Roboters für den AMCL-Algorithmus zur Lokalisierung. Ohne diese Angabe kann Nav2 keine Navigation starten.

---

### 🚗 3. Waypoint-Navigation starten

Starte nun die Node für die automatische Navigation zu Wegpunkten:

```bash
ros2 run multi_filter_pkg waypoint_nav_node
```

Diese Node:

* Liest die Wegpunkte aus einer YAML-Datei
* Sendet Navigationsziele über die `NavigateToPose`-Action an Nav2
* Beobachtet den Status der Ziele und wechselt zum nächsten Wegpunkt, sobald ein Ziel erreicht ist

---

### ✅ Hinweise

* Achte darauf, dass `waypoint_nav_node` erst **nach dem Setzen der Initialpose** gestartet wird.
* Die Wegpunkt-YAML-Datei sollte sich im Paket befinden und korrekt referenziert sein.
* Bei Problemen mit der Navigation überprüfe die TF-Frames (`tf2_tools`, `ros2 run tf2_tools view_frames`) und das Logging in der Konsole.

---
