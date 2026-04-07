# Externe Schnittstellen von ros2_navigation

Dieses Dokument listet alle externen Schnittstellen auf, die das `ros2_navigation` Package von außen nutzt oder bereitstellt. Diese Liste definiert die Anforderungen an ein System, in das dieses Package eingebunden werden soll.

## Übersicht

Das `ros2_navigation` Package benötigt:
- **1 Subscribed Topic** (OccupancyGrid vom Lidar)
- **2 TF2 Topics** (automatisch von TF2 Buffer/Listener abonniert)
- **3 Published Topics** (für Visualisierung)
- **1 Service Server** (für Pfadplanungsanfragen)

---

## Subscribed Topics (Eingänge)

### 1. OccupancyGrid vom Lidar
- **Topic Name**: `lidar/occupancy_grid` (konfigurierbar via Parameter `occupancy_grid_topic`)
- **Message Type**: `nav_msgs/OccupancyGrid`
- **Beschreibung**: Das Package abonniert dieses Topic, um die aktuelle Karte der Umgebung zu erhalten. Dieses Topic muss von einem Lidar-System oder einem anderen Mapping-System bereitgestellt werden.
- **Anforderungen**:
  - Das Grid muss gültige Metadaten enthalten (width, height, resolution, origin)
  - Das Grid sollte regelmäßig aktualisiert werden
  - Frame ID sollte konsistent sein (typischerweise `odom` oder `map`)

### 2. TF Transformationen (automatisch)
- **Topic Name**: `/tf`
- **Message Type**: `tf2_msgs/TFMessage`
- **Beschreibung**: Wird automatisch vom TF2 Buffer/Listener abonniert. Wird benötigt, um Roboterpose und Koordinatentransformationen zu erhalten.
- **Anforderungen**:
  - TF Tree muss die Transformation zwischen `robot_base_frame` (Standard: `base_footprint`) und `global_frame` (Standard: `odom`) bereitstellen
  - Transformationen müssen regelmäßig publiziert werden

### 3. Statische TF Transformationen (automatisch)
- **Topic Name**: `/tf_static`
- **Message Type**: `tf2_msgs/TFMessage`
- **Beschreibung**: Wird automatisch vom TF2 Buffer/Listener abonniert. Für statische Transformationen (z.B. zwischen `base_link` und `base_footprint`).
- **Anforderungen**:
  - Statische Transformationen sollten beim Start publiziert werden

---

## Published Topics (Ausgänge)

### 1. Geplante Pfade
- **Topic Name**: `planned_path` (konfigurierbar via Parameter `planned_path_topic`)
- **Message Type**: `nav_msgs/Path`
- **Beschreibung**: Publiziert geplante Pfade nach erfolgreicher Pfadplanung. Wird für Visualisierung in RViz verwendet.
- **Verwendung**: Externe Systeme können dieses Topic abonnieren, um geplante Pfade zu visualisieren oder zu verwenden.

### 2. Inflated Occupancy Grid
- **Topic Name**: `inflated_occupancy_grid` (konfigurierbar via Parameter `inflated_grid_topic`)
- **Message Type**: `nav_msgs/OccupancyGrid`
- **Beschreibung**: Publiziert das "aufgeblähte" OccupancyGrid, wenn `inflation_radius > 0.0` konfiguriert ist. Zeigt die erweiterten Hindernisse für die Pfadplanung.
- **Verwendung**: Nur für Visualisierung in RViz. Wird nur publiziert, wenn Inflation aktiviert ist.

### 3. Extended Occupancy Grid
- **Topic Name**: `extended_occupancy_grid` (konfigurierbar via Parameter `extended_grid_topic`)
- **Message Type**: `nav_msgs/OccupancyGrid`
- **Beschreibung**: Publiziert das erweiterte OccupancyGrid, wenn das Ziel außerhalb des ursprünglichen Grids liegt und das Grid erweitert wurde.
- **Verwendung**: Nur für Visualisierung in RViz. Wird nur bei Bedarf publiziert.

---

## Services

### 1. Pfadplanung Service (Server)
- **Service Name**: `compute_path_to_pose`
- **Service Type**: `ros2_navigation/srv/ComputePathToPose`
- **Beschreibung**: Service Server, der Pfadplanungsanfragen entgegennimmt und geplante Pfade zurückgibt.
- **Request**:
  - `goal_pose` (geometry_msgs/PoseStamped): Zielpose für die Pfadplanung
  - `start_pose` (geometry_msgs/PoseStamped, optional): Startpose. Wenn leer oder ungültig, wird die aktuelle Roboterpose über TF2 gelesen.
- **Response**:
  - `path` (nav_msgs/Path): Geplanter Pfad (leer wenn kein Pfad gefunden)
  - `path_found` (bool): Erfolgsstatus
  - `planning_time` (float32): Benötigte Zeit für die Planung in Sekunden
  - `message` (string): Statusnachricht
- **Verwendung**: Externe Systeme (z.B. Navigations-Controller) können diesen Service aufrufen, um Pfade zu planen.

---

## TF2 Frames

Das Package benötigt folgende Frames im TF Tree:

### Erforderliche Frames:
- **`robot_base_frame`** (Standard: `base_footprint`): Frame des Roboters für Pose-Abfragen. Sollte auf dem Boden sein (z=0) für 2D-Navigation.
- **`global_frame`** (Standard: `odom`): Globales Koordinatensystem für Ziele und Pfade.

### Typische Frame-Hierarchie:
```
odom (global_frame)
  └── base_footprint (robot_base_frame)
       └── base_link
```

---

## Konfigurierbare Parameter

Alle Topic-Namen und Frame-Namen sind über ROS2-Parameter konfigurierbar:

- `occupancy_grid_topic`: Topic für OccupancyGrid (Standard: `lidar/occupancy_grid`)
- `planned_path_topic`: Topic für geplante Pfade (Standard: `planned_path`)
- `inflated_grid_topic`: Topic für inflated grid (Standard: `inflated_occupancy_grid`)
- `extended_grid_topic`: Topic für extended grid (Standard: `extended_occupancy_grid`)
- `robot_base_frame`: Roboter-Base-Frame (Standard: `base_footprint`)
- `global_frame`: Globales Koordinatensystem (Standard: `odom`)

---

## Zusammenfassung der Systemanforderungen

Ein System, das `ros2_navigation` integrieren möchte, muss bereitstellen:

1. **Lidar/Mapping-System**: Muss `nav_msgs/OccupancyGrid` auf dem konfigurierten Topic publizieren
2. **TF2 System**: Muss einen funktionierenden TF Tree mit Transformationen zwischen `robot_base_frame` und `global_frame` bereitstellen
3. **Optional**: System kann den `compute_path_to_pose` Service aufrufen, um Pfade zu planen
4. **Optional**: System kann die Published Topics abonnieren für Visualisierung oder weitere Verarbeitung

Das Package stellt bereit:
- `compute_path_to_pose` Service für Pfadplanung
- `planned_path` Topic für geplante Pfade
- `inflated_occupancy_grid` Topic für Visualisierung (wenn aktiviert)
- `extended_occupancy_grid` Topic für Visualisierung (bei Bedarf)

