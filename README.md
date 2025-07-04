## 🛠️ Anleitung: Start des ROS 2 Projekts mit Navigation und Initialpose

### 1. Repository clonen oder bereits im Projektverzeichnis sein

Beispiel:

```bash
cd ~/multi_filter_pkg/ROS2Colcon
```

---

### 2. Build des Projekts mit `colcon`

```bash
colcon build --symlink-install
```

---

### 3. `setup.bash` sourcen (einmalig im Terminal):

```bash
source install/setup.bash
```

---

### 4. `launch.sh` ausführbar machen (nur einmal nötig, z. B. nach dem Clonen):

```bash
chmod +x src/multi_filter_pkg/scripts/launch.sh
```

---

### 5. Launch-Skript starten:

```bash
./src/multi_filter_pkg/scripts/launch.sh
```

Dieses Skript erledigt Folgendes:

1. Startet **Nav2 und RViz** im Hintergrund.
2. **Hinweis:** Es dauert **einige Sekunden**, bis RViz vollständig geladen ist und der Roboter erscheint.
3. Sobald du die Initialpose in RViz gesetzt hast (via „2D Pose Estimate“-Tool), wirst du zur Bestätigung aufgefordert:

   ```bash
   Initialpose gesetzt? (y/n):
   ```
4. Erst **nach Bestätigung mit `y`** wird die Wegpunktnavigation (`waypoint_nav_node`) gestartet.

---

### ℹ️ Hinweise zur Initialpose

* Verwende in RViz das Tool **„2D Pose Estimate“**, um die Initialpose zu setzen.
* Warte ein paar Sekunden, bis RViz komplett geladen ist und der Roboter angezeigt wird.
* Ohne Initialpose kann die Navigation nicht korrekt starten.
* Die Node braucht sehr lange bis sie sich initialisiert.

---

Fertig!

