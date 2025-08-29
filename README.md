#  StudentMobilityCompetition-Planning-Control

A collection of ROS2 packages dedicated to **Path/Speed Planning and Tracking** for a custom-built EV Autonomous Vehicle.  
This repository manages my personal work for the **Planning / (High-Level)Control Team** in the Student Mobility Competition (Konkuk University - Team K.A.I.).

```markdown

## Project Structure


Planning/
├── cone_labeling/ # Classifies Cones into Left and Rigth Sides(Using DBSCAN)
│   ├── launch/
│   │   └── cone_labeling.launch.py
│   └── src/
│       ├── cone_classifier.cpp
│       ├── cone_delaunay_connector.cpp
│       └── midpoint_bspline_interpolator.cpp
│
├── cones_no_color/ # Local path generation based on traffic cones
│   ├── msg/ # ModifiedFloat32MultiArray.msg
│   ├── scripts/
│   │   ├── cone_safe_zone.py
│   │   ├── reference_path_planning.py
│   │   └── visualize_cones.py
│   └── launch/ …
│
├── gps_global_planner/ # Global path generation using GPS·RTK
│   ├── data/ # CSVs containing Longitude/Latitude, UTM Coordinates, Covariances
│   ├── launch/
│   │   ├── gps_global_planner_launch.py
│   ├── scripts/
│   │   ├── auto_place_cones.py
│   │   ├── cone_roi_publisher.py
│   │   ├── course_csv_creator.py
│   │   ├── global_yaw_estimator_node.py
│   │   └── publish_global_cones.py
│   └── src/
│       ├── gps_to_local_cartesian.cpp
│       ├── local_cartesian_path_publisher.cpp
│       ├── status_colored_path_publisher.cpp
│       └── vehicle_tf_broadcaster.cpp
│
├── reference_path_classifier/ # Classifies Cones into Left and Rigth Sides(Using Global Path)
│   ├── launch/
│   │   └── classify_cones_by_side.launch.py
│   ├── scripts/
│   │   └── classify_cones_by_side.py
│   └── src/
│       └── classify_cones_by_side.cpp
│
├── speed_planner/ # Calculates Desired Speed for Each Waypoints based on Curvature of the Path
│   ├── launch/
│   │   └── simple_speed_planner.launch.py
│   ├── config/
│   │   └── simple_speed_planner.yaml
│   └── src/
│       └── simple_speed_planner.cpp
│
├ How To Play.txt # Contains How to Run My Packages
│
└── Understanding TF Relationships in the Planning Pipeline.pdf
```

---

## Key Features

| Package | Main Functions | Language |
|--------|-----------|------|
| **cones_no_color** | • Generates an optimal reference path from cone locations<br>• Provides RViz visualization nodes | Python |
| **gps_global_planner** | • Converts RTK-GPS logs to CSV and nav_msgs/Path<br>• ENU ↔ Local Cartesian conversion<br>• Colors the global path by state and speed | C++, Python |
| **reference_path_classifier** | • Classifies cones on the left and right sides relative to the vehicle | Python |

---

## Usage
- Refer to How To Play.txt

### Build (ROS2 Humble)

```bash
cd [workspace path]
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```
- Use `--symlink-install` for immediate reflection of source code changes in the install directory.
- Use `-DCMAKE_BUILD_TYPE=Release` for optimized execution performance.

### Example Execution

```bash
# 1) Generate and publish the GPS-based global path
ros2 launch gps_global_planner gps_global_planner_launch.py

# 2) Classify left-right cones using Global Planned Path
ros2 launch reference_path_classifier classify_cones_by_side.launch.py

# 3) Generate a local path based on detected cones
ros2 run cones_no_color reference_path_planning.py
```

### Demo Video

<table>
  <tr>
    <td align="center">
      <img src="docs/images/1global_pathplanning.gif" width="45%"/><br>
      <b>1. Global Path Planning</b>
    </td>
    <td align="center">
      <img src="docs/images/2globalcones_and_roi.gif" width="45%"/><br>
      <b>2. Global Cones and ROI(Sim)</b>
    </td>
  </tr>
  <tr>
    <td align="center">
      <img src="docs/images/3rightleft_classification_using_frenet.gif" width="45%"/><br>
      <b>3. Right/Left Classification using Frenet</b>
    </td>
    <td align="center">
      <img src="docs/images/4delaunay_triangulation.gif" width="45%"/><br>
      <b>4. Delaunay Triangulation</b>
    </td>
  </tr>
  <tr>
    <td align="center">
      <img src="docs/images/5interpolation_sampling.gif" width="45%"/><br>
      <b>5. Interpolation and Sampling</b>
    </td>
    <td align="center">
      <img src="docs/images/6cone_radius_safe_zone.gif" width="45%"/><br>
      <b>6. Obstacle Radius and Visualize Safe Zone</b>
    </td>
  </tr>
  <tr>
    <td align="center">
      <img src="docs/images/7local_speed_planning.gif" width="45%"/><br>
      <b>7. Local Speed Planning</b>
    </td>
    <td align="center">
      <img src="docs/images/8pure_pursuit.gif" width="45%"/><br>
      <b>8. Path Tracking(Pure Pursuit)</b>
    </td>
  </tr>
</table>

---

### Several Attempts for Better Planning and Tracking

<p align="center">
  <img src="docs/images/9adaptive pure pursuit.gif" width="60%"/><br>
  <b>Adaptive Pure Pursuit</b>
</p>

<p>
  This GIF visualizes the <b>Adaptive Pure Pursuit algorithm</b>, based on the paper 
  <i>“Accurate Path Tracking by Adjusting Look-Ahead Point”</i>. The <b>look-ahead distance</b> dynamically 
  adjusts depending on vehicle speed and path curvature, enabling more accurate and stable path tracking 
  compared to standard Pure Pursuit.
</p>

<ul>
  <li><b>Mint Marker:</b> Look-ahead point of Standard Pure Pursuit</li>
  <li><b>Pink Marker:</b> Look-ahead point of Adaptive Pure Pursuit</li>
</ul>

<p>
  In curves, the look-ahead distance is reduced to minimize path deviation, while on straights it is increased 
  for smoother tracking, improving trajectory stability under diverse driving conditions.
</p>

---

<div align="center">
  <img src="docs/images/teammates/potential field1.gif" width="45%" />
  <img src="docs/images/teammates/potential field2.gif" width="45%" />
  <br>
  <b>Potential Field-Based Local Path Planning</b>
</div>

<p>

  This GIF demonstrates <b>local path planning using a Potential Field method</b>. The algorithm generates a field 
  around all detected cones and identifies the lowest "trough" line as the optimal drivable path, 
  marking these as <b>Through Points</b> (mandatory waypoints the path must pass through). 
  The selected points are then refined with <b>Centripetal Catmull–Rom Spline Interpolation</b> to generate a smooth and continuous 
  local path relative to the vehicle's <code>base_link</code> frame.
</p>


<ul>
  <li><b>Colored Grid:</b> Potential field intensity map</li>
  <li><b>Yellow Dots:</b> Trough Points before smoothing</li>
  <li><b>Pink Line:</b> Final smoothed local path</li>
  <li><b>Blue / Yellow Spheres:</b> Left and Right cones respectively</li>
</ul>


---

## Development Environment

| Item            | Version / Tool               |
|-----------------|------------------------|
| OS              | Ubuntu 22.04           |
| ROS             | ROS2 Humble            |
| GPS RTK Module            | ZED-F9P-04B-01            |
| IMU Module            | myAHRS+            |
| Languages            | Python 3.10 / C++17    |

---

## References

- [ROS2 Official Documentation](https://docs.ros.org/en/humble/)
- [MathWorks Blog: Path Planning for Formula Student Driverless Cars](https://blogs.mathworks.com/student-lounge/2022/10/03/path-planning-for-formula-student-driverless-cars-using-delaunay-triangulation/?from=kr)
- [u-blox GitHub - RTKLIB, GNSS tools](https://github.com/u-blox)
- [u-blox F9P Interface Description (Documentation)](https://content.u-blox.com/sites/default/files/documents/u-blox-F9-HPG-1.32_InterfaceDescription_UBX-22008968.pdf)
---

## Contributions & Contact

- imhyeonwoo21@gmail.com
- imhyeonwoo21@konkuk.ac.kr
