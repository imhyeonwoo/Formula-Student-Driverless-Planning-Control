```markdown
# 🛣️ StudentMobilityCompetition-Planning-Control

ROS2 기반 **지상 차량(UGV) 경로 생성·추종** 전용 패키지 모음입니다.  
창작모빌리티 경진대회(자작자율차 부문) **Planning / Control 파트** 개인 작업을 버전 관리하기 위해 만든 레포지토리입니다.
---

## 📁 프로젝트 구조


Planning/
├── cones_no_color/ # 라바콘 기반 로컬 경로 생성
│ ├── msg/ # ModifiedFloat32MultiArray.msg
│ ├── scripts/
│ │ ├── reference_path_planning.py
│ │ └── visualize_cones.py
│ └── launch/ …
├── gps_global_planner/ # GPS·RTK 기반 글로벌 경로 생성
│ ├── data/ # RTK 로그 · 샘플 CSV
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
└── reference_path_classifier/ # 라바콘 좌·우측 분류기
└── scripts/
└── classify_cones_by_side.py
```

---

## 주요 기능

| 패키지 | 핵심 기능 | 언어 |
|--------|-----------|------|
| **cones_no_color** | • 라바콘 위치를 받아 최적 경로(reference path) 획득<br>• RViz 시각화 노드 제공 | Python |
| **gps_global_planner** | • RTK-GPS 로그를 CSV → nav_msgs/Path 변환<br>• ENU ↔️ Local Cartesian 변환<br>• 글로벌 경로 컬러링(상태·속도별) | C++, Python |
| **reference_path_classifier** | • 양쪽 라바콘을 분류해 차량 기준 좌·우 벡터 생성 | Python |

---

## ⚙️ 사용 방법
- 전체 코스 자율비행하기 전 Trigger 명령으로 웨이포인트마다 호버링 상태를 관찰하고 싶다면 How To Play.txt 참고
- 최종 결과물을 테스트하고 싶다면 How To Play_FINAL.txt 참고

### 🔧 빌드 (ROS2 기준)

```bash
cd [워크스페이스 경로]
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```
- 소스 수정 시 설치 디렉토리에 즉시 반영되도록 심볼릭 링크로 설치하고, 최적화 빌드를 통해 실행 성능을 높이기 위해 colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release를 사용

### 🚀 실행 예시

```bash
# 1) GPS 글로벌 경로 생성 & 퍼블리시
ros2 launch gps_global_planner gps_global_planner_launch.py

# 2) 라바콘 기반 로컬 경로 생성
ros2 run cones_no_color reference_path_planning.py

# 3) 라바콘 좌·우측 분류 결과 확인
ros2 run reference_path_classifier classify_cones_by_side.py
```

### 🚗 주요 기능 시연 영상

👉 [유튜브에서 보기]([https://www.youtube.com/watch?v=iVzSpW8ZjFI](https://youtu.be/qMMXWr9FITQ))

## 영상 설명

---

## 🛠️ 개발 환경

| 항목            | 버전/도구               |
|-----------------|------------------------|
| OS              | Ubuntu 22.04           |
| ROS             | ROS2 Humble            |
| GPS RTK             | u-blox F9P            |
| 언어            | Python 3.10 / C++17    |

---

## 🔗 참고 자료

- [ROS2 공식 문서](https://docs.ros.org/en/humble/)
- [MathWorks 공식 사이트](https://blogs.mathworks.com/student-lounge/2022/10/03/path-planning-for-formula-student-driverless-cars-using-delaunay-triangulation/?from=kr)

---

## 🤝 기여 및 문의

- imhyeonwoo21@gmail.com
- imhyeonwoo21@konkuk.ac.kr
```
