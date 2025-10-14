# Pure Pursuit — Static Look-Ahead with Damping 노드 설명서

> 노드 이름: **`pure_pursuit_static_damped`**  
> 목적: 고정 Look-Ahead Distance(정적 Ld)에 **헤딩오차 변화율 D항**을 더해, 진동 억제/응답 개선된 조향 명령(`/cmd/steer`, 단위 **deg**) 퍼블리시

---

## 개요

이 노드는 로컬 경로(`/local_planned_path`)를 입력으로 받아 **Pure Pursuit** 방식으로 조향각을 계산합니다.  
기본은 정적 Ld(고정 거리)이며, 여기에 **헤딩오차의 변화율** 항을 더해(일종의 D 제어), 코너 진입/탈출 시의 진동을 줄이고 응답을 다듬습니다.

- 입력
  - `nav_msgs/Path` : `/local_planned_path` (권장 프레임: `base_link`)
- 출력
  - `std_msgs/Float32` : `steer_topic` (기본 `/cmd/steer`, 단위: **deg**)
  - `visualization_msgs/Marker` : `lookahead_marker_topic` (기본 `/lookahead_point_marker`)

---

## 핵심 아이디어

### 1) 정적 Ld 기반 Pure Pursuit

경로상의 룩어헤드 목표점 \(p=(x, y)\)에서 기본 조향각(rad):

$$\delta_{\mathrm{pp}} = \mathrm{atan2}\!\left(\frac{2 L \, y}{x^2+y^2}\right),\quad \delta_{\deg} = \delta_{\mathrm{pp}}\cdot\frac{180}{\pi}$$

여기서 $L$은 휠베이스, ($x$,$y$)는 `base_link` 기준 목표점 좌표입니다.

### 2) 헤딩오차 변화율 D항 추가

- 헤딩오차 $\alpha = \mathrm{atan2}(y, x)$
- 시간 미분을 1차 LPF(시간상수 `deriv_lpf_tau`)로 안정화한 후, $k_d$를 곱해 기본 조향에 더합니다.

$$\delta = \delta_{\mathrm{pp}} + k_d\,\dot{\alpha}_{\mathrm{LPF}}$$

이로써 코너 진입 시 과도한 조향을 누그러뜨리거나, 급격한 방향 변화에 대한 응답을 부드럽게 만들 수 있습니다.

### 3) 제한 및 부호 규약

- 최종 조향각(deg)은 `steer_limit_deg`로 클램프됩니다.
- `invert_steer_sign=true`이면 퍼블리시 직전에 부호를 반전합니다(우:+, 좌:- 체계를 쓰는 기존 시스템과의 호환 목적).

---

## 토픽 & 프레임

| 역할 | 파라미터/토픽 | 타입 | 설명 |
|---|---|---|---|
| 경로 입력 | `path_topic` (기본 `/local_planned_path`) | `nav_msgs/Path` | `frame_id=base_link` 권장 |
| 조향 출력 | `steer_topic` (기본 `/cmd/steer_sp`) | `std_msgs/Float32` | 단위: **deg** |
| 시각화 | `lookahead_marker_topic` (기본 `/lookahead_point_marker_damped`) | `visualization_msgs/Marker` | 룩어헤드 포인트 표시 |

> 현재 구현은 **`base_link` 기준 경로**를 가정합니다. 다른 프레임이면 사전 변환이 필요합니다.

---

## 파라미터 (기본값 예시)

`config/pure_pursuit_static_damped.yaml`

```yaml
pure_pursuit_static_damped:
  ros__parameters:
    # Topics
    path_topic: "/local_planned_path"
    steer_topic: "/cmd/steer_sp"
    lookahead_marker_topic: "/lookahead_point_marker_damped"

    # Vehicle
    wheelbase_m: 1.3

    # Static Look-Ahead Distance
    lookahead_m: 11.0

    # Limits / behavior
    publish_rate_hz: 50.0
    steer_limit_deg: 30.0
    use_x_forward_only: true

    # D-term (heading error rate)
    enable_d_term: true
    kd_heading_rate: 0.15
    deriv_lpf_tau: 0.20

    # Output sign convention
    invert_steer_sign: true

    # Marker (mint)
    marker_scale: 0.50
    marker_alpha: 1.0
    marker_r: 0.00
    marker_g: 1.00
    marker_b: 0.80
```

---

## 알고리즘 플로우

1. **룩어헤드 포인트 선택**: $\|p\| \ge L_d$ (고정 `lookahead_m`), 옵션으로 `x>0`만 후보
2. **기본 조향 계산**: $\delta_{\mathrm{pp}}$ (rad) → deg 변환
3. **D항 계산**: $\alpha=\mathrm{atan2}(y,x)$의 시간 변화율 → 1차 LPF → `kd_heading_rate` 곱해 가산
4. **제한/부호**: `steer_limit_deg` 클램프, 필요 시 `invert_steer_sign` 적용 후 퍼블리시
5. **시각화**: 룩어헤드 포인트를 마커로 퍼블리시

---

## 튜닝 가이드

- **D항 강도 (`kd_heading_rate`)**
  - 값을 키우면 헤딩 변화에 더 민감해져 코너 진입 과조향을 억제하거나 응답을 다듬을 수 있음
  - 과도하면 지터/노이즈 증폭 가능 → `deriv_lpf_tau`와 함께 조정
- **미분 노이즈 완화 (`deriv_lpf_tau`)**
  - 값을 키우면 더 부드럽지만 응답이 느려짐, 너무 작으면 노이즈 증폭
- **룩어헤드 거리 (`lookahead_m`)**
  - 크면 안정↑/민감도↓, 작으면 예리한 회전/진동 위험↑
- **부호 규약 (`invert_steer_sign`)**
  - 차량/저수준 제어기의 조향 부호 체계에 맞춰 설정

---

## 확인/디버깅 체크리스트

- `/local_planned_path`가 **`base_link` 기준**인지 확인
- 마커가 전방 룩어헤드 위치에 안정적으로 표시되는지
- 조향 출력이 `steer_limit_deg` 범위를 넘지 않는지(포화 빈도 확인)
- D항 활성화 시 (`enable_d_term=true`) 지나친 고주파가 보이면 `kd_heading_rate`↓ 또는 `deriv_lpf_tau`↑

---

## 예시 값 (v_max ≈ 4–6 m/s 환경)

- `lookahead_m = 11.0 m`
- `kd_heading_rate = 0.10 ~ 0.20 s`
- `deriv_lpf_tau = 0.15 ~ 0.30 s`
- `steer_limit_deg = 30°`

---

## 변경 요점

- 정적 Ld 기반 PP에 **헤딩오차 변화율 D항** 추가
- 출력 단위 **deg** 유지, 필요 시 부호 반전(`invert_steer_sign`) 지원
- 룩어헤드 마커를 별도 토픽(`..._damped`)으로 제공

