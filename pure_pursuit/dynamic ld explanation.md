# Pure Pursuit — Dynamic Look-Ahead Distance (Ld) 노드 설명서

> 마지막 업데이트: 2025-08-26  
> 노드 이름: **`pure_pursuit_dynamic`**  
> 목적: 속도(선택적으로 곡률) 기반 **동적 Look-Ahead Distance**로 조향 명령(`/cmd/steer`, 단위 **deg**, 좌회전 **+**)을 퍼블리시

---

## 개요

이 노드는 로컬 경로(`/local_planned_path`)와 차량 현재 속도(`/current_speed`)를 받아, **Pure Pursuit** 기법으로 차량 조향각을 계산합니다.  
Look-Ahead Distance \(L_d\) 를 **속도에 따라 동적으로** 조정하며, 옵션으로 **경로 곡률**을 반영할 수 있습니다.

- **입력**
  - `nav_msgs/Path` : `/local_planned_path` (기준 프레임: `base_link` 권장)
  - `std_msgs/Float32` : `/current_speed` (단위: **m/s**)
- **출력**
  - `std_msgs/Float32` : `/cmd/steer` (단위: **deg**, 좌회전 **+**)
  - `visualization_msgs/Marker` : `/lookahead_point_marker` (민트색, SPHERE)

---

## 핵심 아이디어

### 1) 동적 Ld 공식 (기본)

$$L_{d,\mathrm{raw}} = L_0 + k_v \cdot v$$

- $v$: 현재 속도(m/s), `ema_tau_speed`로 EMA 필터 후 사용
- $L_0$ (m): 저속에서의 기본 Ld
- $k_v$ (s): 속도와 곱해 Ld [m]로 환산되는 계수

### 2) (선택) 곡률 보정 항

$$L_{d,\mathrm{raw}} \leftarrow L_{d,\mathrm{raw}} + \frac{k_k}{|\kappa| + \varepsilon}$$

- $\kappa$: 경로 시작부에서 `curv_window_m` 지점 근방의 **3점 곡률** 추정치
- $k_k$: 급커브에서 Ld를 자동으로 축소/확대하는 보정 계수 (양수면 급커브에서 **Ld 증가**가 아니라, 통상은 반대로 “감소”를 원하면 **음수**를 사용하거나 설계를 반대로 두면 됩니다. 기본은 비활성화)

### 3) 클램프 & 평활

$$L_d = \mathrm{clip}\big(L_{d,\min}, L_{d,\mathrm{raw}}, L_{d,\max}\big)$$


조향 명령은 최종적으로 `steer_limit_deg`로 **클램프**되고, `ema_tau_cmd`로 **EMA 평활**되어 출력됩니다.

### 4) 조향각 계산 (Pure Pursuit)

경로가 **`base_link`** 기준일 때, 룩어헤드 타깃 포인트 \(p=(x, y)\)에서

$$\delta_\mathrm{rad} = \mathrm{atan2}\!\left(\frac{2 L \, y}{x^2+y^2}\right),\quad \delta_\mathrm{deg} = \delta_\mathrm{rad}\cdot\frac{180}{\pi}$$

- $L$: 휠베이스(본 프로젝트: **1.295 m**)
- 좌회전이 **양수**가 되도록 y>0이면 +가 나오며, REP-103 관례에 부합

---

## 토픽 & 프레임

| 역할 | 토픽 | 타입 | 단위/설명 |
|---|---|---|---|
| 경로 입력 | `/local_planned_path` | `nav_msgs/Path` | **`frame_id=base_link` 권장** |
| 속도 입력 | `/current_speed` | `std_msgs/Float32` | **m/s**, 약 100 Hz |
| 조향 출력 | `/cmd/steer` | `std_msgs/Float32` | **deg**, 좌회전 **+** |
| 시각화 | `/lookahead_point_marker` | `visualization_msgs/Marker` | 민트색 SPHERE |

> 경로가 `map` 프레임이라면 Pure Pursuit 노드 내부에서 `map→base_link` TF 변환이 필요합니다. 현재 구현은 **`base_link` 기준 경로**를 가정합니다.

---

## 파라미터 (기본값)

아래는 `config/pure_pursuit_dynamic.yaml`의 기본값 예시입니다.

```yaml
pure_pursuit_dynamic:
  ros__parameters:
    # Topics
    path_topic: "/local_planned_path"
    speed_topic: "/current_speed"
    steer_topic: "/cmd/steer"
    lookahead_marker_topic: "/lookahead_point_marker"

    # Vehicle
    wheelbase_m: 1.295

    # Dynamic Ld
    L0: 1.5           # [m]
    k_v: 0.6          # [s]
    Ld_min: 1.0       # [m]
    Ld_max: 5.0       # [m]

    # Curvature term (optional)
    use_curvature_term: false
    k_k: 0.0
    epsilon_kappa: 1.0e-6
    curv_window_m: 2.0

    # Behavior
    publish_rate_hz: 50.0
    steer_limit_deg: 30.0      # [deg]
    use_x_forward_only: true

    # Smoothing
    ema_tau_speed: 0.2         # [s]
    ema_tau_cmd: 0.1           # [s]

    # Marker (mint)
    marker_scale: 0.30
    marker_alpha: 1.0
    marker_r: 0.00
    marker_g: 1.00
    marker_b: 0.80
```

---

## 알고리즘 플로우

1. **속도 수신**: `/current_speed` → EMA($\tau = \text{ema\_tau\_speed}$) → $v_\mathrm{filt}$
  
2. **Ld 계산**: $L_d = \mathrm{clip}\big(L_{d,\min},\, L_0 + k_v v_\mathrm{filt} [+ \frac{k_k}{|\kappa|+\varepsilon}],\, L_{d,\max}\big)$
3. **룩어헤드 포인트 선택**: $\|p\| \ge L_d$인 **첫 점** (옵션: `x>0` 전방만)  
4. **조향 계산**: $\delta$ (rad) → deg 변환 → `steer_limit_deg` **클램프**  
5. **평활 & 퍼블리시**: EMA($\tau = \text{ema\_tau\_cmd}$) → `/cmd/steer` (deg) 퍼블리시  
6. **시각화**: `/lookahead_point_marker`에 **민트색 SPHERE** 표시


---

## 빌드 & 실행

### CMakeLists.txt (발췌)
```cmake
add_executable(pure_pursuit_dynamic src/pure_pursuit_dynamic.cpp)
ament_target_dependencies(pure_pursuit_dynamic
  rclcpp std_msgs nav_msgs visualization_msgs
)
install(TARGETS pure_pursuit_dynamic
  DESTINATION lib/${PROJECT_NAME})
```

### Launch
`launch/pure_pursuit_dynamic.launch.py` 사용:
```bash
# 빌드
colcon build --packages-select pure_pursuit
source install/setup.bash

# 기본 파라미터로 실행
ros2 launch pure_pursuit pure_pursuit_dynamic.launch.py

# 작업 트리의 yaml을 직접 지정해 실행
ros2 launch pure_pursuit pure_pursuit_dynamic.launch.py \
  params_file:=$COLCON_WS/src/Planning/pure_pursuit/config/pure_pursuit_dynamic.yaml
```

---

## 튜닝 가이드

- **반응성/안정성 균형**
  - `k_v` ↑ → 속도에 민감하게 Ld가 커짐(고속 안정 ↑, 응답 ↓)
  - `L0` ↓ / `Ld_min` ↓ → 저속에서 더 예리한 회전(응답 ↑, 진동 위험 ↑)
  - `Ld_max` ↑ → 고속 직진 안정 ↑ (과도하면 둔감)
- **명령 부드러움**
  - `ema_tau_cmd` ↑ → 출력을 더 부드럽게(응답 ↓)
- **급커브 대응**
  - `use_curvature_term=true` + `k_k` 조정: 급커브에서 Ld 자동 보정  
    - 일반적으로 급커브에서 **Ld를 줄이고자** 하면 \(k_k < 0\)로 두는 방식을 권장  
- **전방점만 사용**
  - `use_x_forward_only=true` 권장 (경로 잡음/후방점 선택 방지)

---

## 확인/디버깅 체크리스트

- 단위 확실화: `/current_speed`는 **m/s**, `/cmd/steer`는 **deg**(좌회전 +)
- 프레임: `/local_planned_path`는 **`base_link` 기준**인지 확인
- 룩어헤드 시각화: `/lookahead_point_marker`가 경로 위 전방에 찍히는지
- 제한/평활: `steer_limit_deg`, `ema_tau_cmd` 조절로 오버슈트/진동 조정
- 경로 샘플 간격이 너무 성기면: Path 생성기의 `arc_step` 재검토(현재 0.5 m)

---

## 예시 값 (v_max ≈ 4 m/s 환경)

- `L0 = 1.5 m`, `k_v = 0.6 s`, `Ld_min = 1.0 m`, `Ld_max = 5.0 m`
  - v=0 → Ld=1.5 m  
  - v=4 → Ld=3.9 m
- `steer_limit_deg = 30°`, `ema_tau_cmd = 0.1 s`

---

## 변경 이력(요점)

- 출력 토픽을 **도(°)**로 통일: `/cmd/steer` (좌회전 **양수**)
- 라디안 토픽(`/cmd/steer_rad`) 제거
- 속도 기반 Ld + (옵션) 곡률 보정
- 민트색 룩어헤드 마커 시각화

---

## 참고: 정적 Ld 노드 차이점

| 항목 | 정적 Ld | 동적 Ld |
|---|---|---|
| **Look-Ahead** | 고정값 | $L_0 + k_v * v$ |
| **입력 속도 필요** | X | O (`/current_speed`) |
| **튜닝 난이도** | 낮음 | 중간(더 유연) |
| **고속 안정성** | 파라미터 의존 | 고속에서 자연히 증가 |

---
