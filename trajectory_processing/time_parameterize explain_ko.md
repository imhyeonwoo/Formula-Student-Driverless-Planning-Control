# `traj_time_parameterizer.py`에서의 Equal-Time 샘플링
> **목표:** 호 길이(arc-length) 기준으로 샘플링된 로컬 경로(그리고 선택적 per-waypoint 속도 프로파일)를 **동일 시간 간격(equal-time)**의 궤적으로 변환하여 `/time_param_path`로 퍼블리시한다.

이 문서는 equal-time 샘플링의 **원리**를 설명하고, 각 단계를 구현한 **정확한 코드 위치**(`src/Planning/trajectory_processing/scripts/traj_time_parameterizer.py`)를 보여줍니다.

---

## 0) 파일 & 노드 컨텍스트

- 파일: `src/Planning/trajectory_processing/scripts/traj_time_parameterizer.py`  
- 노드 이름: `traj_time_parameterizer`  
- 입력 토픽:  
  - `/local_planned_path` (`nav_msgs/Path`) — 호 길이 기반 샘플링
  - `/desired_speed_profile` (`std_msgs/Float32MultiArray`) — 웨이포인트별 **원하는 속도**(선택)
- 출력 토픽:  
  - `/time_param_path` (`nav_msgs/Path`) — **동일 시간 간격**으로 리샘플링된 궤적

---

## 1) 개념 요약 — Equal-Time 샘플링이란?

우리는 **고정된 시간 간격** `Δt`(예: 0.1 s)에서의 웨이포인트를 원합니다:
$$
t_k = k\Delta t,\quad k = 0,1,2,\dots,\left\lfloor \frac{\texttt{preview\_time}}{\Delta t}\right\rfloor
$$

경로와 해당 속도가 주어지면, 각 **공간 구간**이 걸리는 시간을 추정하고, **누적 시간 테이블** \(T_k\)를 만든 뒤, 원하는 시간 \(t_k\)에서 **보간하여 위치**를 구합니다.

- **포인트 개수**는 **시간 지평**(`preview_time`)과 **시간 간격**(`dt`)에 의해 결정됩니다.  
- 출력 점들의 **공간 간격**은 **국소 속도** \(v\)에 따라 달라집니다: 빠를수록 간격이 멀어지고, 느릴수록 가까워집니다.

---

## 2) 파라미터 — 코드에서의 위치

**의미**
- `dt`: equal-time 샘플링 간격 [s]
- `v_nom`: 속도 프로파일이 없을 때 사용하는 기본(명목) 속도
- `preview_time`: 시간 지평(얼마나 미래까지 퍼블리시할지)

**코드**
```python
self.dt           = self.declare_parameter("dt", 0.1).value
self.v_nom        = self.declare_parameter("v_nom", 3.0).value
self.preview_time = self.declare_parameter("preview_time", 5.0).value
```

---

## 3) 구독 & 퍼블리시

**구독**
- `/local_planned_path` → 리샘플링 파이프라인 트리거:
```python
self.create_subscription(Path,
                         "/local_planned_path",
                         self.path_callback, 10)
```
- `/desired_speed_profile` → 최신 속도 배열 캐시:
```python
self.create_subscription(Float32MultiArray,
                         "/desired_speed_profile",
                         self.speed_callback, 10)
```

**퍼블리시**
- `/time_param_path` (equal-time 경로):
```python
self.pub_traj = self.create_publisher(Path, "/time_param_path", 10)
```

---

## 4) 속도 프로파일 입력(speed_planning/src/speed_planner.cpp)

**개념**: **웨이포인트별 원하는 속도**를 사용해 구간 이동 시간을 계산합니다(단일 명목 속도보다 더 정확).

**코드**
```python
def speed_callback(self, msg: Float32MultiArray):
    self.speed_profile = list(msg.data)
```

---

## 5) Path 콜백 — Equal-Time 파이프라인

### Step 5.1 — 좌표 추출
**개념**: 입력 `Path`에서 x, y 배열을 얻습니다.
```python
X = np.array([p.pose.position.x for p in path.poses])
Y = np.array([p.pose.position.y for p in path.poses])
```

### Step 5.2 — 구간 거리
**개념**: 인접한 점 \(i \rightarrow i+1\) 쌍마다 유클리드 거리 \(d_i\)를 계산합니다.
```python
d = np.hypot(np.diff(X), np.diff(Y))          # N-1
```

### Step 5.3 — 구간 대표 속도
**개념**
- 속도 프로파일이 있으면, 길이를 `N`(poses)와 정렬하고, 0 회피를 위해 clamp한 뒤, 구간마다 **양 끝 평균**을 사용합니다:
  $$
  v_{{\text{seg},i}} = \tfrac{1}{2}(v_i + v_{i+1})
  $$
- 없으면 상수 `v_nom`을 사용합니다.

**코드**
```python
if self.speed_profile:
    sp   = np.array(self.speed_profile, float)
    sp   = sp[:N] if len(sp) >= N else np.pad(sp, (0, N-len(sp)), 'edge')
    v_i  = np.maximum(sp, 1e-3)               # 0 방지
    v_seg = 0.5 * (v_i[:-1] + v_i[1:])        # N-1
else:
    v_seg = np.full(N-1, self.v_nom)          # 고정 속도
```

> **왜 평균인가요?** 호 길이 간격이 작을 때(예: 0.5 m) 구간 내 속도가 선형적으로 변한다고 보는 근사가 타당합니다. 이는 거리 대비 시간 적분에서 사다리꼴 적분에 해당합니다.

### Step 5.4 — 구간 이동 시간
**개념**: 
$$
\Delta t_i = \dfrac{d_i}{v_{{\text{seg},i}}}
$$
```python
dt_seg = d / v_seg
```

### Step 5.5 — 누적 시간 테이블
**개념**: 각 웨이포인트 도달 시간을 가지는 \(T_k\)를 만듭니다.
```python
T = np.insert(np.cumsum(dt_seg), 0, 0.0)
total_T = T[-1]
```

### Step 5.6 — Equal-Time 질의 그리드
**개념**: 우리가 출력 샘플을 원하는 시간들:
$t = 0, \Delta t, \dots$ up to $\min(\texttt{preview\_time},\, T_{\text{end}})$
```python
t_q = np.arange(0.0,
                min(total_T, self.preview_time) + 1e-9,
                self.dt)
```

### Step 5.7 — 시간 축에서 위치 보간
**개념**: 테이블 \((T_k, X_k)\), \((T_k, Y_k)\)을 사용해 **시간 기준**으로 보간합니다.  
이는 \(s(t)\)의 역함수를 통해 \(x(t), y(t)\)를 평가하는 이산적 등가물입니다.

**코드**
```python
x_q = np.interp(t_q, T, X)
y_q = np.interp(t_q, T, Y)
```

### Step 5.8 — Equal-Time `Path` 퍼블리시
**개념**: \((x_q, y_q)\)를 `nav_msgs/Path`로 다시 포장하여 퍼블리시합니다.
```python
traj = Path()
traj.header = path.header
traj.header.stamp = self.get_clock().now().to_msg()

for tq, x, y in zip(t_q, x_q, y_q):
    ps = PoseStamped()
    ps.header.frame_id = path.header.frame_id
    ps.pose.position.x = float(x)
    ps.pose.position.y = float(y)
    ps.pose.orientation.w = 1.0
    traj.poses.append(ps)

self.pub_traj.publish(traj)
```

---

## 6) 수학적 관점(얼마나 긴 거리를 커버하나?)

출력으로 커버되는 공간 길이는:
$$
L = \int_{0}^{\min(\texttt{preview\_time},\, T_{\text{end}})} v(t)\,dt
$$
속도가 상수라면, 이는 $L = v \cdot \texttt{preview\_time}$로 단순화됩니다.  
일반적으로 **포인트 개수**는 시간(`dt`, `preview_time`)으로 고정되며, **간격**은 국소 속도에 따라 달라집니다($\Delta \ell \approx v \cdot \Delta t$).

---

## 7) 엣지 케이스 & 견고성(코드 상 고려)

- **짧은 경로**: `total_T < preview_time` → 그리드가 더 일찍 끝나므로(포인트가 적어짐) 자연스럽게 조기 종료됩니다.
- **속도 누락**: `v_nom`으로 대체(상수 속도 가정).
- **0 속도**: `np.maximum(sp, 1e-3)`로 clamp하여 0으로 나누기 방지.
- **길이 불일치**: 속도 벡터는 경로 길이에 맞도록 잘라내거나(edge padding으로) 채웁니다.

---

## 8) 실무 튜닝 팁

- `dt`: 제어 주기와 비슷하게(예: 0.05–0.1 s).
- `preview_time`: 보통 1–3 s; 더 크면 지평이 길어지지만 계산량이 늘고, 먼 미래 참조로 인해 계획 변경 민감도가 커질 수 있습니다.
- 더 나은 헤딩 제어가 필요하면, **국소 탄젠트**로부터 **pose별 yaw**를 계산하여 orientation에 저장하세요.

---

## 9) 최소 의사코드

**Pseudocode**
```text
on speed_profile:
    cache sp

on path:
    X, Y = extract(path)
    d = distances(X, Y)
    v_seg = 0.5*(v[i]+v[i+1]) or v_nom
    dt_seg = d / v_seg
    T = cumsum(dt_seg) with leading 0
    t_q = 0:dt:min(T_end, preview_time)
    x_q = interp(t_q, T, X)
    y_q = interp(t_q, T, Y)
    publish Path(x_q, y_q)
```

**Code**
```python
# speed_callback
self.speed_profile = list(msg.data)

# path_callback
X = np.array([p.pose.position.x for p in path.poses])
Y = np.array([p.pose.position.y for p in path.poses])
d = np.hypot(np.diff(X), np.diff(Y))

if self.speed_profile:
    sp = np.array(self.speed_profile, float)
    sp = sp[:N] if len(sp) >= N else np.pad(sp, (0, N-len(sp)), 'edge')
    v_i = np.maximum(sp, 1e-3)
    v_seg = 0.5 * (v_i[:-1] + v_i[1:])
else:
    v_seg = np.full(N-1, self.v_nom)

dt_seg = d / v_seg
T = np.insert(np.cumsum(dt_seg), 0, 0.0)
total_T = T[-1]

t_q = np.arange(0.0, min(total_T, self.preview_time) + 1e-9, self.dt)
x_q = np.interp(t_q, T, X)
y_q = np.interp(t_q, T, Y)

traj = Path()
traj.header = path.header
traj.header.stamp = self.get_clock().now().to_msg()
for tq, x, y in zip(t_q, x_q, y_q):
    ps = PoseStamped()
    ps.header.frame_id = path.header.frame_id
    ps.pose.position.x = float(x)
    ps.pose.position.y = float(y)
    ps.pose.orientation.w = 1.0
    traj.poses.append(ps)
self.pub_traj.publish(traj)
```

---

## 10) 선택적 확장(어디에 추가할까)

- **Pose별 타임스탬프**(시간 인지 시각화/소비자 용):
  루프 내부에서 `tq`를 사용해 `ps.header.stamp`를 설정합니다.
  ```python
  # 예시(포즈마다 시각 태그를 붙이고 싶을 때)
  # ps.header.stamp = rclpy.time.Time(seconds=float(tq)).to_msg()
  ```
- **탄젠트 기반 Yaw(heading)**:
  이웃 점으로부터 yaw를 계산하고 쿼터니언으로 변환해 `ps.pose.orientation`에 넣습니다.
- **고정 길이 출력**( `T_end < preview_time` 인 경우 패딩 ):
  마지막 샘플을 반복하여 `len(traj.poses)`를 일정하게 유지할 수 있습니다.

---

**결론:** 이 스크립트는 속도 프로파일(또는 명목 속도)을 사용해 공간 경로 위에 **시간 파라미터화**를 만들고, **균일한 시간 간격**에서 위치를 보간해 퍼블리시합니다. 이는 로컬 궤적 생성을 위한 **equal-time 샘플링**의 실전 구현 그 자체입니다.