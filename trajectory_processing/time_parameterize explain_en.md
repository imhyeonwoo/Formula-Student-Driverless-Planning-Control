# Equal-Time Sampling in `traj_time_parameterizer.py`
> **Goal:** Convert an arc-length–sampled local path (and an optional per-waypoint speed profile) into an **equal-time** trajectory published on `/time_param_path`.

This document explains the **principles** of equal-time sampling and shows **exact code locations** in `src/Planning/trajectory_processing/scripts/traj_time_parameterizer.py` that implement each step.

---

## 0) File & Node Context

- File: `src/Planning/trajectory_processing/scripts/traj_time_parameterizer.py`  
- Node name: `traj_time_parameterizer`  
- Input topics:  
  - `/local_planned_path` (`nav_msgs/Path`) — arc-length sampled
  - `/desired_speed_profile` (`std_msgs/Float32MultiArray`) — per-waypoint **desired speeds** (optional)
- Output topic:  
  - `/time_param_path` (`nav_msgs/Path`) — **equal-time** resampled trajectory

---

## 1) Concept Recap — What is Equal-Time Sampling?

We want waypoints at **fixed time intervals** `Δt` (e.g., 0.1 s):  
$$
t_k = k\Delta t,\quad k = 0,1,2,\dots,\left\lfloor \frac{\texttt{preview\_time}}{\Delta t}\right\rfloor
$$


Given the path and its speeds, we estimate how long each **spatial segment** takes, build a **cumulative time table** \(T_k\), and then **interpolate positions** at the desired times \(t_k\).

- **Point count** is fixed by **time horizon** (`preview_time`) and **time step** (`dt`).  
- **Spatial spacing** between output points varies with local speed \(v\): faster → farther apart, slower → closer.

---

## 2) Parameters — Where they are in the code

**Meaning**
- `dt`: equal-time sampling interval [s]
- `v_nom`: fallback nominal speed (used if no speed profile yet)
- `preview_time`: time horizon (how far into the future we publish)

**Code**
```python
self.dt           = self.declare_parameter("dt", 0.1).value
self.v_nom        = self.declare_parameter("v_nom", 3.0).value
self.preview_time = self.declare_parameter("preview_time", 5.0).value
```

---

## 3) Subscriptions & Publications

**Subscriptions**
- `/local_planned_path` → triggers the resampling pipeline:
```python
self.create_subscription(Path,
                         "/local_planned_path",
                         self.path_callback, 10)
```
- `/desired_speed_profile` → caches the latest speed array:
```python
self.create_subscription(Float32MultiArray,
                         "/desired_speed_profile",
                         self.speed_callback, 10)
```

**Publications**
- `/time_param_path` (equal-time path):
```python
self.pub_traj = self.create_publisher(Path, "/time_param_path", 10)
```

---

## 4) Speed Profile Intake (speed_planning/src/speed_planner.cpp)

**Concept**: use **per-waypoint desired speeds** to compute segment travel times (more accurate than a single nominal speed).

**Code**
```python
def speed_callback(self, msg: Float32MultiArray):
    self.speed_profile = list(msg.data)
```

---

## 5) Path Callback — The Equal-Time Pipeline

### Step 5.1 — Extract coordinates
**Concept**: Get arrays of x, y from the input `Path`.
```python
X = np.array([p.pose.position.x for p in path.poses])
Y = np.array([p.pose.position.y for p in path.poses])
```

### Step 5.2 — Segment distances
**Concept**: For each consecutive pair \(i \rightarrow i+1\), compute Euclidean distance \(d_i\).
```python
d = np.hypot(np.diff(X), np.diff(Y))          # N-1
```

### Step 5.3 — Segment representative speeds
**Concept**
- If a speed profile is present, align length to `N` poses, clamp to avoid zero, and use **endpoint average** per segment:
  $$
  v_{{\text{seg},i}} = \tfrac{1}{2}(v_i + v_{i+1})
  $$
- Else use constant `v_nom`.

**Code**
```python
if self.speed_profile:
    sp   = np.array(self.speed_profile, float)
    sp   = sp[:N] if len(sp) >= N else np.pad(sp, (0, N-len(sp)), 'edge')
    v_i  = np.maximum(sp, 1e-3)               # 0 방지
    v_seg = 0.5 * (v_i[:-1] + v_i[1:])        # N-1
else:
    v_seg = np.full(N-1, self.v_nom)          # 고정 속도
```

> **Why average?** The arc-length spacing is small (e.g., 0.5 m), so linear change over a segment is a good approximation. This corresponds to trapezoidal integration for time over distance.

### Step 5.4 — Segment travel times
**Concept**: 
$$
\Delta t_i = \dfrac{d_i}{v_{{\text{seg},i}}}
$$
```python
dt_seg = d / v_seg
```

### Step 5.5 — Cumulative time table
**Concept**: Build \(T_k\) so each waypoint has an arrival time.
```python
T = np.insert(np.cumsum(dt_seg), 0, 0.0)
total_T = T[-1]
```

### Step 5.6 — Equal-time query grid
**Concept**: Times at which we want output samples: 
$t = 0, \Delta t, \dots$ up to $\min(\texttt{preview\_time}, T_{\text{end}})$

```python
t_q = np.arange(0.0,
                min(total_T, self.preview_time) + 1e-9,
                self.dt)
```

### Step 5.7 — Interpolate positions on the time axis
**Concept**: Interpolate **by time** using the table \((T_k, X_k)\), \((T_k, Y_k)\).  
This is the discrete equivalent of evaluating \(x(t), y(t)\) with the inverse mapping \(s(t)\).

**Code**
```python
x_q = np.interp(t_q, T, X)
y_q = np.interp(t_q, T, Y)
```

### Step 5.8 — Publish the equal-time `Path`
**Concept**: Repackage \((x_q, y_q)\) into a `nav_msgs/Path` and publish.
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

## 6) Mathematical View (What length do we cover?)

The spatial length covered by the output is:
$$
L = \int_{0}^{\min(\texttt{preview\_time},\, T_{\text{end}})} v(t)\,dt
$$
When speed is constant, this reduces to $L = v \cdot \texttt{preview\_time}$.  
In general, point **count** is fixed by time (`dt`, `preview_time`), while **spacing** varies with local speed ($\Delta \ell \approx v \cdot \Delta t$).


---

## 7) Edge Cases & Robustness in the Code

- **Short path**: `total_T < preview_time` → the grid stops early (you’ll get fewer points).
- **Missing speeds**: falls back to `v_nom` (constant speed assumption).
- **Zero speeds**: clamped with `np.maximum(sp, 1e-3)` to avoid division by zero.
- **Length mismatch**: speed vector is truncated/padded to match the path length (edge padding repeats the last value).

---

## 8) Practical Tuning

- `dt`: pick near your control period (e.g., 0.05–0.1 s).
- `preview_time`: pick 1–3 s typically; larger increases horizon (and compute) but risks referencing far-future parts that may change.
- For better heading control, consider computing **per-pose yaw** from local tangents and storing it in the orientations.

---

## 9) Minimal Pseudocode Recap with Code Parallels

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

## 10) Optional Enhancements (Where to add them)

- **Per-pose timestamps** (for time-aware viz/consumers):  
  Inside the loop, set `ps.header.stamp` using `tq`.
  ```python
  # example (if you want per-pose time tags)
  # ps.header.stamp = rclpy.time.Time(seconds=float(tq)).to_msg()
  ```
- **Yaw (heading) from tangent**:
  Compute yaw from neighbor points and convert to quaternion; assign to `ps.pose.orientation`.
- **Constant-length output** (pad when `T_end < preview_time`):
  Pad the last sample to keep `len(traj.poses)` constant.

---

**Bottom line:** The script builds a **time parameterization** over the spatial path using the speed profile (or a nominal speed), then **interpolates positions at uniform time steps**. That’s exactly the practical implementation of **equal-time sampling** for local trajectory generation.