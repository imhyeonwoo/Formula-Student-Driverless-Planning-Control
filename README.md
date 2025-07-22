#  StudentMobilityCompetition-Planning-Control

A collection of ROS2 packages dedicated to **Path/Speed Planning and Tracking** for a custom-built EV Autonomous Vehicle.  
This repository manages my personal work for the **Planning / (High-Level)Control Team** in the Student Mobility Competition (Konkuk University - Team K.A.I.).

```markdown

## Project Structure


Planning/
├── cones_no_color/ # Local path generation based on traffic cones
│ ├── msg/ # ModifiedFloat32MultiArray.msg
│ ├── scripts/
│ │ ├── reference_path_planning.py
│ │ └── visualize_cones.py
│ └── launch/ …
├── gps_global_planner/ # Global path generation using GPS·RTK
│ ├── data/ # RTK logs · sample CSVs
│ ├── scripts/
│ │ ├── auto_place_cones.py
│ │ ├── cone_roi_publisher.py
│ │ ├── course_csv_creator.py
│ │ ├── global_yaw_estimator_node.py
│ │ └── publish_global_cones.py
│ └── src/
│ ├── gps_to_local_cartesian.cpp
│ ├── local_cartesian_path_publisher.cpp
│ ├── status_colored_path_publisher.cpp
│ └── vehicle_tf_broadcaster.cpp
└── reference_path_classifier/ # Classifies cones into left and right sides
└── scripts/
└── classify_cones_by_side.py
```

---

## Key Features

| Package | Main Functions | Language |
|--------|-----------|------|
| **cones_no_color** | • Generates an optimal reference path from cone locations<br>• Provides RViz visualization nodes | Python |
| **gps_global_planner** | • Converts RTK-GPS logs to CSV and nav_msgs/Path<br>• ENU ↔️ Local Cartesian conversion<br>• Colors the global path by state and speed | C++, Python |
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
- Use --symlink-install for immediate reflection of source code changes in the install directory.
- Use -DCMAKE_BUILD_TYPE=Release for optimized execution performance.

### Example Execution

```bash
# 1) Generate and publish the GPS-based global path
ros2 launch gps_global_planner gps_global_planner_launch.py

# 2) Generate a local path based on detected cones
ros2 run cones_no_color reference_path_planning.py

# 3) Check the result of cone left-right classification
ros2 run reference_path_classifier classify_cones_by_side.py
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
      <b>2. Global Cones and ROI</b>
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
</table>


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
