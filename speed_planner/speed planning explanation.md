
# Speed Planner Node (`speed_planner.cpp`) – Algorithm and Code Walk‑through

본 문서는 `src/Planning/speed_planning/src/speed_planner.cpp`에 구현된 **SpeedPlanner** ROS 2 노드의 구조와 알고리즘을 한눈에 정리한 설명서입니다.  
코드는 **곡률(κ)** 과 **차량의 동역학 제약**을 이용하여 경로상의 각 웨이포인트에 대한 **목표 속도 프로파일**을 계산하고 퍼블리시합니다.

---

## 1. 노드 개요

| 항목 | 내용 |
|------|------|
| 패키지 | `speed_planning` |
| 노드 이름 | `speed_planner` |
| 입력 토픽 | `/local_planned_path` (`nav_msgs/Path`) – 로컬 경로 <br>`/current_speed` (`std_msgs/Float32`) – 차량 현재 속도 |
| 출력 토픽 | `/desired_speed_profile` (`std_msgs/Float32MultiArray`) – 웨이포인트별 목표 속도 배열 |
| 주요 파라미터 | `max_speed`, `max_accel`, `max_decel`, `max_lat_accel` |

---

## 2. 핵심 아이디어

1. **곡률 제한 속도**  
   곡률 κ가 큰 구간(급커브)에서는 횡가속 한계 a<sub>lat,max</sub>를 넘지 않도록  
   $$
   v_{curve}=\sqrt{\dfrac{a_{lat,max}}{|\kappa|}}
   $$
2. **가속·감속 한계**  
   - 전·후방 패스를 통해 **종방향 가속/감속 한계**(a<sub>long,max</sub>)를 적용  
   - 물리적으로 불가능한 속도 변화를 제거
3. **Feed‑Forward** (미래 곡률 고려)  
   ![Feed‑Forward](docs/images/speed_planning/2feed_forward.png)  
   향후 일정 거리의 평균 곡률 \(\bar{\kappa}\)를 사용하면 MPC 등 상위 제어기에 유용한 **곡률 분포** 정보를 얻을 수 있다. (현 구현에는 포함되지 않았으나 확장 가능)

---

## 3. 곡률 계산

![Δs 구간 곡률](docs/images/speed_planning/1curvature.png)

삼각형 법을 이용해 **연속한 세 점** \(P_{i-1},P_i,P_{i+1}\) 로 이루는 원의 반지름 R를 구하고  
$$
\kappa = \dfrac{1}{R} = \dfrac{4A}{abc} \quad (a, b, c는 삼각형 변 길이, A는 면적)
$$


코드 구현 `curvature2D()` 요약:

```cpp
a = hypot(x1-x0, y1-y0);
b = hypot(x2-x1, y2-y1);
c = hypot(x2-x0, y2-y0);
area2 = std::abs((x1-x0)*(y2-y0) - (x2-x0)*(y1-y0)); // = 2A
kappa  = 2*area2 / (a*b*c);  // det = 2A ⇒ κ = 4A/(abc)
```

> 면적이 0(세 점이 일직선) → κ=0 ⇢ 속도는 `max_speed`로 유지.

---

## 4. 속도 프로파일 생성 절차

![가·감속 제약 시뮬레이션](docs/images/speed_planning/3accel_limit.png)

| 단계 | 설명 | 수식 / 의사코드 |
|------|------|----------------|
|① **초기화**|`speed[i] = max_speed` (전 구간)|—|
| ② **곡률 제한** | `speed[i] = min(speed[i], v_curve)` | v_curve = sqrt(a_lat,max / (|$\kappa$| + $\epsilon$)) |
|③ **시작점 보정**|`speed[0] = min(speed[0], current_speed)`|—|
|④ **Backward pass**<br>(감속 제약)|`for j=N-2→0:`<br>`dist = ‖Pj+1-Pj‖`<br>`v_allow = √(v_next² + 2·a_decel·dist)`<br>`speed[j] = min(speed[j], v_allow)`|방정식: $v_{allow}=\sqrt{v_{next}^2 + 2a_{decel}Δs}$|
|⑤ **Forward pass**<br>(가속 제약)|`for j=0→N-2:`<br>`dist = ‖Pj+1-Pj‖`<br>`v_allow = √(v_curr² + 2·a_accel·dist)`<br>`speed[j+1] = min(speed[j+1], v_allow)`|방정식: $v_{allow}=\sqrt{v_{curr}^2 + 2a_{accel}Δs}$|
|⑥ **퍼블리시**|`Float32MultiArray` 로 퍼블리시|—|



### 중요 파라미터

| 이름 | 의미 | 기본값 | 코드 위치 |
|------|------|--------|-----------|
|`max_speed`|최대 직선 속도 제한|20 m/s|`declare_parameter`|
|`max_lat_accel`|허용 횡가속|4 m/s²|—|
|`max_accel`|최대 종가속 (가속)|2 m/s²|—|
|`max_decel`|최대 종가속 (감속)|2 m/s²|—|

---

## 5. 클래스 및 ROS 인터페이스

```txt
SpeedPlanner
 ├─ Publishers
 │   └─ /desired_speed_profile   (std_msgs/Float32MultiArray)
 │   ├─ /cmd/speed               (std_msgs/Float32)
 │   └─ /cmd/rpm                 (std_msgs/Float32)
 ├─ Subscriptions
 │   ├─ /local_planned_path      (nav_msgs/Path)
 │   └─ /current_speed           (std_msgs/Float32)
 └─ Parameters
     ├─ max_speed
     ├─ max_accel
     ├─ max_decel
     └─ max_lat_accel
```

- **`pathCallback()`** : 경로 수신 시 전체 속도 프로파일 재계산  
- **`speedCallback()`** : 현재 속도 업데이트 → 다음 계획 시 초기값에 반영

---

## 6. 확장 포인트

1. **미래 곡률 기반 Feed‑Forward 가속 설정**  
   이미지를 참고하여 \(\bar{\kappa}\) 를 이용하면 MPC에서 예측 제어 성능 향상 가능.
2. **Z 축 고도 고려 3D 곡률**  
   현재는 2D 평면 가정 → 필요 시 3D 위치 벡터로 일반화 가능.
3. **동적 파라미터 재구성** (*rclcpp::ParameterEventHandler*)  
   실시간으로 파라미터를 조정하여 주행 중 튜닝 가능.

---

## 7. 결론

`SpeedPlanner` 노드는 **곡률‑기반 속도 제한 + 종가속 제한**의 두 단계를 결합하여
안전하면서도 부드러운 속도 프로파일을 생성합니다.  
MPC, Pure Pursuit 등 상위 제어기에 바로 입력할 수 있으며, 파라미터를 조정해
차량 제원이나 트랙 특성에 맞게 쉽게 재사용할 수 있습니다.

---
