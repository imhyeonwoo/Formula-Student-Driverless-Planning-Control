# Time Sampling Path 사용 버전
 **목표:** Local Cartesian 경로와 Global Path 기준으로 ROI 내 콘들의 절대좌표를 구해 좌우 구분을 한 후 Local Path Planning을 진행한 후 등시간격 샘플링된 경로(`/time_param_path`)를 Publish

---

 ## 1. Global Path Planning + TF
 ```bash
 ros2 launch gps_global_planner gps_global_planner_launch.py
 ```
- 파일: `gps_global_planner/launch/gps_global_planner_launch.py`
- 노드: 
  - `gps_to_local_cartesian` : UTM -> Local Cartesian : gps-imu fusion 잘되면 여기서 받아오기
  - `status_colored_path_publisher` : Covariance에 따른 글로벌 경로 색상 구분
  - `vehicle_tf_broadcaster` : TF 정의 -> global yaw 값은 Localization의 odometry 정보로부터 받아옴
  - `hud_overlay` 노드 실행하기 위한 **requirement** : `rviz_2d_overlay_plugins` 패키지
    ```bash
    sudo apt update
    sudo apt install ros-humble-rviz-2d-overlay-msgs ros-humble-rviz-2d-overlay-plugins
    ```

---

 ## 2. Yaw값 구하기
 ```bash
 ros2 launch imu_preprocess imu_preprocess.launch.py
 ros2 launch gps_imu_fusion ekf_fusion.launch.py
 ```
 ```bash
 # just in case...
 ros2 run gps_imu_fusion_ihw global_yaw_complementary_node
 # 100Hz
 ```
 ```bash
 # for dubuging[웬만하면 가장 정확하지만 8 Hz 밖에 안되므로 사용하기에는 부적합 (단순 디버깅 용)]
 ros2 run gps_global_planner global_yaw_estimator_node.py
 # 8Hz
 ```

---

 ## 3. gps reference 경로를 기반으로 콘 절대좌표 변환 및 좌우 구분(ihw)
 ```bash
 ros2 launch reference_path_classifier classify_cones_by_side.launch.py
 ```
- 파일: `reference_path_classifier/src/classify_cones_by_side.cpp`
- 노드: 
  - `classify_cones_by_side` : K-D Tree로 콘 탐색(Frenet Frame 느낌)'
- `reference_path_classifier/modification.md` 참고

---

 ## 4-0. Local Path Planning by IHW
 ```bash
 ros2 run cones_no_color reference_path_planning.py
 ```
 - 파일: `cones_no_color/scripts/reference_path_planning.py`
 - Delanuay Triangulation -> B-Spline -> arc length로 등간격 샘플링

 ## 4-1. Local Path Planning by KTY
 ```bash
  ros2 run cone_labeling_k potential_field
 ```
 - 파일: `cone_labeling_k/cone_labeling_k/potential_field.py`
 - Potential Filed -> Field 내 Trough(골) Point 탐색 -> 거리 기반 필터링 -> Spline -> (Labeling) -> Path Planning 
 

 ## 4-2. Local Path Planning by HSM
 ```bash
  ros2 run cone_labeling_hsm path_planning
 ```

---

 ## 5-0. 임시 Current Speed(m/s) Publish 노드 실행
 ```bash
 ros2 run gps_imu_fusion_ihw current_speed_node
 # 100Hz
 ```
 -> publishes `/current_speed` -> change if wanted
 - 이것 또한 마찬가지로 Localization에서 잘하거나 제어 파트에서 속도 잘 주면 사용

 ## 5-1. 곡률 기반 속도 플래닝 노드 실행
 ```bash
 ros2 launch speed_planner simple_speed_planner.launch.py
 ```
 - `speed planning explanation.md` 참고

---

 ## 6. 등시간격으로 재샘플링된 경로 생성
 ```bash
 ros2 run trajectory_processing traj_time_parameterizer.py
 ```
 - `trajectory_processing/time_parameterize explain_ko.md` 참고
 - 사용방법은 논의 필요
---

 ## 7. Pure Pursuit
 정적 $L_d$ 사용 시
 ```bash
 ros2 launch pure_pursuit pure_pursuit_static.launch.py
 ```

 동적 $L_d$ 사용 시
 ```bash
 ros2 launch pure_pursuit pure_pursuit_dynamic.launch.py
 ```
 - 동적 Look Ahead Distance 방식은 `pure_pursuit/dynamic ld explanation.md` 참고

 적응형 $L_d$ 사용 시
 ```bash
 ros2 launch pure_pursuit adaptive_pure_pursuit.launch.py
 ```
---

## 8. AEB Detection
 ```bash
 ros2 launch aeb_k aeb_k_with_visualizer.launch.py
 ```