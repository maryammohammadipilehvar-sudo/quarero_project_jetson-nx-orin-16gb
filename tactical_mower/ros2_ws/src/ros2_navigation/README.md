# ROS2 Navigation Package

ROS2-basiertes Navigation-Package mit nav2-Integration für Pfadplanung basierend auf OccupancyGrid vom Lidar.

## Übersicht

Das `ros2_navigation` Package bietet:
- **Service-Server** für Pfadplanung (`compute_path_to_pose`)
- **Replanning-Service** für automatische Pfad-Neuplanung bei Kollisionen (`replan_path`)
- **Modulare Architektur** für einfache Erweiterung und Wartung
- **Grid-Erweiterung** für Ziele außerhalb des ursprünglichen OccupancyGrids
- **Obstacle Inflation** für sichere Pfadplanung

## Installation

### Als Submodule in ROS2 Workspace

```bash
# In deinem ROS2 Workspace:
cd /path/to/your/ros2_ws/src
git submodule add <url-to-ros2_navigation-repo> ros2_navigation

# Workspace bauen:
cd /path/to/your/ros2_ws
source /opt/ros/jazzy/setup.bash  # oder deine ROS2-Distribution
colcon build --packages-select ros2_navigation
source install/setup.bash
```

### Direktes Klonen

```bash
# Repository klonen:
git clone <url-to-ros2_navigation-repo> ros2_navigation

# In Workspace einbinden (symlink oder kopieren):
cd /path/to/your/ros2_ws/src
ln -s /path/to/ros2_navigation ros2_navigation

# Workspace bauen:
cd /path/to/your/ros2_ws
colcon build --packages-select ros2_navigation
source install/setup.bash
```

## Schnittstellen

### Services

#### `compute_path_to_pose`
- **Service Type**: `ros2_navigation/srv/ComputePathToPose`
- **Beschreibung**: Plant einen Pfad von der aktuellen Roboterpose (oder angegebener Startpose) zu einem Ziel.
- **Request**:
  - `goal_pose` (geometry_msgs/PoseStamped): Zielpose
  - `start_pose` (geometry_msgs/PoseStamped, optional): Startpose (leer = aktuelle Roboterpose)
- **Response**:
  - `path` (nav_msgs/Path): Geplanter Pfad
  - `path_found` (bool): Erfolgsstatus
  - `planning_time` (float32): Planungszeit in Sekunden
  - `message` (string): Statusnachricht

#### `replan_path`
- **Service Type**: `ros2_navigation/srv/ReplanPath`
- **Beschreibung**: Prüft einen Pfad auf Kollisionen und replant bei Bedarf automatisch.
- **Request**:
  - `path` (nav_msgs/Path): Zu prüfender Pfad
  - `goal_pose` (geometry_msgs/PoseStamped): Zielpose (für Replanning)
  - `start_waypoint_index` (int32): Ab welchem Waypoint prüfen (0 = von Anfang)
- **Response**:
  - `path_found` (bool): True wenn neuer Pfad gefunden
  - `path` (nav_msgs/Path): Neuer Pfad (leer wenn keine Kollision)
  - `has_collision` (bool): True wenn Kollision erkannt
  - `message` (string): Statusnachricht

### Topics

#### Subscribed
- `lidar/occupancy_grid` (nav_msgs/OccupancyGrid, konfigurierbar): OccupancyGrid vom Lidar/Mapping-System
- `/tf` und `/tf_static` (automatisch): TF2 Transformationen für Roboterpose

#### Published
- `planned_path` (nav_msgs/Path, konfigurierbar): Geplante Pfade für Visualisierung
- `inflated_occupancy_grid` (nav_msgs/OccupancyGrid, optional): Inflated Grid für RViz (wenn `inflation_radius > 0`)
- `extended_occupancy_grid` (nav_msgs/OccupancyGrid, optional): Erweiterte Grids für RViz (bei Bedarf)

### TF2 Frames

**Erforderlich:**
- `robot_base_frame` (Standard: `base_footprint`): Frame des Roboters für Pose-Abfragen
- `global_frame` (Standard: `odom`): Globales Koordinatensystem für Ziele und Pfade

**Typische Frame-Hierarchie:**
```
odom (global_frame)
  └── base_footprint (robot_base_frame)
       └── base_link
```

## Verwendung

### Node starten

```bash
# Mit Launch-Datei (empfohlen):
ros2 launch ros2_navigation nav2_navigation.launch.py

# Oder direkt:
ros2 run ros2_navigation nav2_navigation_node
```

### Service aufrufen

```bash
# Pfad planen:
ros2 service call /compute_path_to_pose ros2_navigation/srv/ComputePathToPose \
  "{goal_pose: {header: {frame_id: 'odom'}, pose: {position: {x: 5.0, y: 3.0}}}}"

# Pfad auf Kollisionen prüfen und replanen:
ros2 service call /replan_path ros2_navigation/srv/ReplanPath \
  "{path: {...}, goal_pose: {...}, start_waypoint_index: 0}"
```

## Konfiguration

Parameter können in `config/nav2_navigation_params.yaml` angepasst werden:

### Wichtige Parameter

- `use_sim_time`: **Wichtig**: `true` für Simulation (Gazebo), `false` für echten Roboter
- `occupancy_grid_topic`: Topic für OccupancyGrid (Standard: `lidar/occupancy_grid`)
- `planned_path_topic`: Topic für geplante Pfade (Standard: `planned_path`)
- `robot_base_frame`: Frame-ID des Roboters (Standard: `base_footprint`)
- `global_frame`: Globales Frame (Standard: `odom`)
- `algorithm_type`: Planungsalgorithmus (`astar` oder `theta_star`)
- `inflation_radius`: Distanz zu Hindernissen in Metern (Standard: `0.3`, `0.0` deaktiviert)
- `replanning_service_name`: Name des Replanning-Services (Standard: `replan_path`)

### Vollständige Parameterliste

Siehe `config/nav2_navigation_params.yaml` für alle verfügbaren Parameter.

## Entwicklung

### Package-Struktur

```
ros2_navigation/            # Package-Root (als Submodule in ros2_ws/src/)
├── CMakeLists.txt          # Build-Konfiguration
├── package.xml             # Package-Metadaten
├── config/                 # Konfigurationsdateien
│   └── nav2_navigation_params.yaml
├── launch/                 # Launch-Dateien
│   └── nav2_navigation.launch.py
├── srv/                    # Service-Definitionen
│   ├── ComputePathToPose.srv
│   └── ReplanPath.srv
├── ros2_navigation/        # Python-Module
│   ├── nav2_navigation_node.py
│   ├── path_planning_service.py
│   ├── replanning_service.py
│   ├── planner_manager.py
│   ├── tf_manager.py
│   ├── grid_utils.py
│   └── ...
└── test/                   # Tests
    └── test_*.py
```

### Erweitern des Packages

#### Neuen Service hinzufügen

1. Service-Definition in `srv/` erstellen
2. In `CMakeLists.txt` registrieren:
   ```cmake
   rosidl_generate_interfaces(${PROJECT_NAME}
     "srv/ComputePathToPose.srv"
     "srv/ReplanPath.srv"
     "srv/YourNewService.srv"  # Hinzufügen
     ...
   )
   ```
3. Service-Handler in `ros2_navigation/` implementieren
4. In `nav2_navigation_node.py` integrieren

#### Neuen Planungsalgorithmus hinzufügen

1. Algorithmus in `planner_manager.py` implementieren
2. In `_plan_path_*` Methoden integrieren
3. Parameter `algorithm_type` erweitern

### Tests ausführen

```bash
cd /path/to/your/ros2_ws
colcon test --packages-select ros2_navigation
colcon test-result --verbose
```

## Systemanforderungen

### Erforderlich

- **ROS2** (getestet mit Jazzy, sollte mit anderen Distributionen funktionieren)
- **Python 3.8+**
- **CMake 3.8+**
- **nav2** Packages (für Planner-Plugins, optional)

### Externe Abhängigkeiten

Das Package benötigt:
- **Lidar/Mapping-System**: Muss `nav_msgs/OccupancyGrid` publizieren
- **TF2 System**: Muss Transformationen zwischen `robot_base_frame` und `global_frame` bereitstellen
- **Standard ROS2 Messages**: `geometry_msgs`, `nav_msgs`, `std_msgs`

## Weitere Dokumentation

- **Externe Schnittstellen**: Siehe `EXTERNE_SCHNITTSTELLEN.md` für detaillierte Schnittstellen-Dokumentation
- **Docker-Setup**: Für isolierte Entwicklungsumgebung (optional)

## Lizenz

MIT License
