# `traj_time_parameterizer.py`에서의 Equal-Time 샘플링 (확장판)
> **목표:** 호 길이(arc-length) 기준으로 샘플링된 로컬 경로(그리고 선택적 per-waypoint 속도 프로파일)를 **동일 시간 간격(equal-time)**의 궤적으로 변환하여 `/time_param_path`로 퍼블리시한다.

이 문서는 equal-time 샘플링의 **원리**를 설명하고, 각 단계를 구현한 **정확한 코드 위치**(`src/Planning/trajectory_processing/scripts/traj_time_parameterizer.py`)를 보여줍니다.  
또한 최근 논의한 **오해 방지 포인트**, **변수 의미( s, v_seg )**, **ROI와 오차**, **예시 수치**, **튜닝 팁**을 추가했습니다.

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

> **주의:** `v_nom`은 0보다 커야 합니다. 0이면 `dt_seg = d/v_seg` 계산에서 분모가 0이 되어 무한/NaN이 나올 수 있습니다. 최소 에폭실론(예: 1e-3)로 클램프하거나 유효성 검사를 권장합니다.

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

### Step 5.3 — 구간 대표 속도 (공간 도메인 값)
**핵심 개념**
- `v_seg[i]`는 **세그먼트 i (웨이포인트 i→i+1)** 를 대표하는 **상수 속도**입니다.  
  시간의 함수가 아니라 **공간(호길이) 도메인**에 붙은 값입니다.
- 속도 프로파일이 있으면, 길이를 `N`과 정렬하고, 0 회피를 위해 clamp한 뒤, 구간마다 **양 끝 평균**을 사용합니다:
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

> **왜 평균인가요?** 호 길이 간격이 작을 때(예: 0.5 m) 구간 내 속도가 선형적으로 변한다고 보는 근사가 타당합니다. 이는 거리 대비 시간 적분에서 **사다리꼴 적분**에 해당합니다.  
> (보수적으로 하려면 `min(v_i, v_{i+1})`를 쓰는 방법도 있습니다.)

> **공간 해상도:** 등간격 샘플링 간격이 Δs=0.5 m라면, 대부분의 세그먼트 길이 `d_i≈0.5 m`이므로 `v_seg` 시퀀스는 **약 0.5 m 간격**의 공간 해상도를 갖습니다(마지막 세그먼트는 더 짧을 수 있음).

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

> **오해 바로잡기:** “인접 2개의 속도의 평균으로 다음 웨이포인트를 직접 예측한다”가 아니라, **그 평균은 세그먼트 시간을 구하는 데 쓰이고**, 위치는 **시간축 보간**으로 얻습니다. Δt가 구간 경계를 넘나들 수 있어 단일 속도로 “한 번에 다음 좌표”를 찍는 것은 일반적으로 맞지 않습니다.

---

## 7) s(호길이)와 Δs(샘플 간격)

- `s`는 **경로를 따라 누적된 거리(연속 변수)** 입니다. 시작점에서 `s=0`, 종점에서 `s=L`.
- 등간격 샘플링을 Δs=0.5 m로 했다면, 샘플들은 대략 `s_k ≈ k·0.5 m` 위치에 있습니다.
- 인접 샘플 간 세그먼트 길이 `d_i ≈ 0.5 m`가 되어, 위에서 계산한 `v_seg[i]`는 사실상 **0.5 m 간격**의 공간 해상도를 가집니다.

---

## 8) ROI(미래 창)와 오차/효율

- **Speed Planner 단계**(곡률 기반 속도)에는 ROI가 직접적 인풋이 아닙니다(전체 path에 대해 계산 가능).  
- **Time Parameterizer 단계**에서 ROI(`preview_time`)가 커지면:
  - **포인트 수 증가**: `⌊preview_time/dt⌋+1`
  - 더 먼 미래까지 보간 → 환경 변동/센서 인지 범위를 넘는 구간 포함 시 **빈번한 재계산/폐기(오버헤드)** 발생 가능
  - 모델 근사 오차 누적 가능(멀리 갈수록 가정이 깨질 여지 ↑)
- 실전에서는 **센서 인지 범위**·**제어 주기**·**계산 여력**에 맞춰 1–3 s 수준이 안정적입니다.

---

## 9) 수치 예시 (N=15 웨이포인트)

- 웨이포인트 15개(P₀…P₁₄), 세그먼트 14개, 각 세그먼트 길이 `d_i=0.5 m` 가정  
- 속도프로파일(코너→직선 가속 예):  
  `v = [5,5,5,6,7,8,9,10,11,12,12,12,11,10,9] m/s`
- 세그먼트 평균속도: `v_seg[i] = 0.5*(v_i + v_{i+1})`  
  → `[5.0, 5.0, 5.5, 6.5, 7.5, 8.5, 9.5, 10.5, 11.5, 12.0, 12.0, 11.5, 10.5, 9.5]`
- 세그먼트 시간: `Δt_i = d_i / v_seg[i]`  
  → `[0.1000, 0.1000, 0.0909, 0.0769, 0.0667, 0.0588, 0.0526, 0.0476, 0.0435, 0.0417, 0.0417, 0.0435, 0.0476, 0.0526] s`
- 누적 시간표 `T_k`: `T_0=0`, `T_k = Σ Δt_i`  
  → 예: `T_3≈0.2909, T_7≈0.5459, T_14≈0.8641 s` …
- `dt=0.1 s`, `preview_time=0.5 s`라면 등시간 질의는 `[0.0, 0.1, 0.2, 0.3, 0.4, 0.5] s`이고, 각 시각에 대해 `T` 테이블을 기준으로 위치를 **시간 비율로 보간**해 구합니다.

**코드 대응**
```python
# 평균속도(세그먼트 대표속도)와 시간 테이블
v_i  = np.maximum(sp, 1e-3)
v_seg = 0.5 * (v_i[:-1] + v_i[1:])
dt_seg = d / v_seg
T = np.insert(np.cumsum(dt_seg), 0, 0.0)

# 등시간 그리드와 시간축 보간
t_q = np.arange(0.0, min(total_T, self.preview_time) + 1e-9, self.dt)
x_q = np.interp(t_q, T, X)
y_q = np.interp(t_q, T, Y)
```

---

## 10) 포인트 수 vs. 공간 길이

- **포인트 수**: `⌊preview_time/dt⌋+1` → **속도와 무관** (단, path가 너무 짧아 total_T < preview_time 이면 감소)
- **공간 길이**: `∫ v(t) dt` → **속도에 의해 결정** (고속 구간이 많을수록 길어짐)

---

## 11) 선택적 확장(어디에 추가할까)

- **Pose별 타임스탬프**(시간 인지 시각화/소비자 용):
  ```python
  # ps.header.stamp = rclpy.time.Time(seconds=float(tq)).to_msg()
  ```
- **탄젠트 기반 Yaw(heading)**:
  이웃 점으로부터 yaw를 계산하고 쿼터니언으로 변환해 `ps.pose.orientation`에 넣습니다.
- **고정 길이 출력**( `T_end < preview_time` 인 경우 패딩 ):
  마지막 샘플을 반복하여 `len(traj.poses)`를 일정하게 유지할 수 있습니다.
- **NaN/음수 속도 방어**:
  ```python
  sp = np.nan_to_num(sp, nan=self.v_nom, posinf=self.v_nom, neginf=self.v_nom)
  sp = np.maximum(sp, 1e-3)
  ```

---

## 12) 최소 의사코드

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

**Code (핵심 라인들)**
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

## 13) 한 줄 요약

> **`v_seg`는 시간 함수가 아닌 세그먼트별 상수(공간 도메인)** 이고, 그 값으로 **세그먼트 시간을** 구해 **누적 시간표**를 만든 뒤, **고정 시간격자**에서 **좌표를 보간**해 `/time_param_path`를 만든다.  
> 포인트 수는 **시간축**으로 결정되고, 포인트 간 **거리 간격**은 **속도에 비례**해 변한다.