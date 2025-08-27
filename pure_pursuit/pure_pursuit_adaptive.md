
# Adaptive Pure Pursuit (APP) — Design & Implementation Guide
*File:* `pure_pursuit/src/pure_pursuit_adaptive.cpp`  
*Node name:* `pure_pursuit_adaptive`  

---

## 1) Overview

This node implements an **Adaptive Pure Pursuit (APP)** controller that tracks a local path while dynamically adjusting the **look‑ahead distance (LAD)** and **target point** to improve stability and reduce **inside‑corner cutting**.

**Core features:**
- **Dynamic LAD:** speed‑based and optional curvature‑aware adjustment.
- **Robust target selection:** path‑relative (arc‑length) or Euclidean‑distance mode with *sticky* index to suppress jumps.
- **Corner‑cutting suppression:** shift the control target outward on curves (pd → pl) with safety gates and track boundary limits.
- **Command shaping:** steering rate limiting + EMA smoothing.
- **Rich RViz debugging:** consistent marker namespaces (`pp/...`) for easy inspection.

**Coordinate convention:** Path and markers are published in `frame_id_from_path` (path header’s `frame_id`) if present, otherwise `base_frame` (default `base_link`). Steering command is **degrees**, left turn **positive**.

---

## 2) Notation & Targets

- **`pw`** — *path waypoint near vehicle*: the nearest path point to the vehicle (used as a local reference).
- **`pd`** — *default look‑ahead*: the point selected using the LAD rule (arc‑length or Euclidean).
- **`pl`** — *final control target*: a possibly *outward‑shifted* version of `pd` (to reduce cutting) and then **EMA‑smoothed** in (x, y).  
  → **Steering is computed from `pl`**.

**Steering law (Pure Pursuit):**
$$
\delta = \operatorname{atan2}\!\left(\frac{2\,L\,y_{pl}}{x_{pl}^{2}+y_{pl}^{2}}\right) \;(\mathrm{rad})
$$

→ **deg** → **rate-limit** → **EMA** → **clamp**


---

## 3) Dynamic Look‑Ahead Distance (LAD)

Function: `double computeLd() const`

$$
L_d = \mathrm{clip}\!\Big(
    L_0 + k_v \,\max(0, v)
    + \big[ use\_curv ? \; \frac{k_{\text{curv}}}{|\kappa| + \epsilon} : 0 \big],
    \; L_{d,\min},\; L_{d,\max}
\Big)
$$


- **Speed term** (`L0`, `k_v`): Longer LAD at higher speed (stability on straights).
- **Curvature term** (`k_curv`): Optional; decreases LAD on tighter curves (responsiveness) and increases it on straights.
- **Clamping** (`Ld_min`, `Ld_max`): Safety bounds.

Curvature magnitude `|\kappa|` is sampled **ahead of the vehicle** (`curv_window_m`) from the path using a 3‑point circle estimate with local smoothing (see §6).

> **Note:** When using **arc‑length selection**, `L_d` is the **path distance** forward from `pw`, not necessarily equal to the **Euclidean** distance \(\|pd\|\) from the origin. Therefore the visual **lookahead circle** (radius = \(L_d\) around the origin) is an *indicator*, not a guarantee that `pd` lies on that circle.

---

## 4) Target Selection

### 4.1 Sticky nearest (`pw`)
Function: `int findNearestIndexSticky() const`  
- On first run: nearest path point to the origin (optionally `x > forward_margin_x`).
- Afterwards: search only within `idx_pw_prev_ ± sticky_window_pts` to reduce index jumps.

### 4.2 Default look‑ahead (`pd`)
Function: `int selectTargetIndexSticky(int idx_start, double Ld) const`  
Two modes controlled by **`use_arc_length_selection`**:

- **Arc‑length mode (`true`)**:  
  Find index where cumulative path length satisfies  
  $\text{cum\_s}[pd] \;\approx\; \text{cum\_s}[pw] + L_d$.
- **Euclidean mode (`false`)**:  
  First index where $\|p\| \ge L_d$.

Both modes apply:
- **Forward filtering** (`x_forward_only`, `forward_margin_x`) — only consider points with sufficient forward x.
- **Sticky clamp** to `idx_pd_prev_ ± sticky_window_pts` — suppress sudden target jumps.

### 4.3 Final control target (`pl`): outward shift (optional)
If `outer_offset_enable = true`, the node shifts `pd` outward to reduce apex‑cutting:

1. **Signed curvature** around `pw` and `pd`: $\kappa_{pw},\; \kappa_{pd} \;(\text{smoothed})$.  
2. **Weights** (0–1):  
   - Proximity factor $\alpha = \min(1, \|pw\| / \alpha_{max}$ — closer means stronger offset.  

   - Curvature ratio factor $\beta = \min\!\Big(1, \frac{|\kappa_{pd}|}{|\kappa_{pw}|} / \beta_{max}\Big)$ — pd bending more than pw increases offset.

   - Combine: $\tau = \mathrm{clamp}\big((1-\alpha)\,\beta,\; 0,\; \text{outer\_offset\_tau\_max}\big)$.

3. **Gating by curvature magnitude**: if $|\kappa_{pd}| < \text{outer\_offset\_kappa\_gate}$, set $\tau=0$ (straight → no offset).

4. **Outward direction**: use tangent at `pd` and signed curvature to pick the **outside** normal.

5. **Offset magnitude**:  
   $\text{off} = \min\big(\tau\,\|pd-pw\|,\; \text{outer\_offset\_max\_m}\big)$, and if track width is known:  
   $\text{off} \le \max\big(0,\ \text{track\_half\_width\_m} - \text{track\_margin\_m}\big)$.
6. **EMA smoothing (x, y)** for `pl` with time constant `target_ema_tau`.

If disabled, **`pl = pd`**.

---

## 5) Command Shaping

Function: `double smoothDeg(double raw_deg)`

1. **Rate limit**: clamp change by `± steer_rate_limit_deg_per_s * dt`.
2. **EMA**: filter the limited value with time constant `ema_tau_cmd`.
3. **Clamp**: final `± steer_limit_deg`.

This balances responsiveness and stability, especially on noisy or sparsely sampled paths.

---

## 6) Curvature & Tangent Estimation

- **Signed curvature (3‑point):**
$\kappa \;=\; \frac{(B-A) \times (C-A)}{\|B-A\|\,\|C-B\|\,\|C-A\|}$
(2D cross product in numerator; preserves sign: left‑turn positive)

- **Local smoothing**: average over indices `j-W … j+W` where `W = kappa_smooth_window_pts`.
- **`evalCurvatureAbsAt(s)`**: pick index by **cumulative arc‑length** near `s`, then compute smoothed \(|\kappa|\).
- **Tangent** at index `i`: `C - A` (normalized), where `A=i-1`, `C=i+1`.

---

## 7) ROS Interfaces

### Subscriptions
- **`/local_planned_path`** (`nav_msgs/Path`): tracking path.
- **`/current_speed`** (`std_msgs/Float32`): vehicle speed [m/s]; EMA(\( \tau = \) `ema_tau_speed`).

### Publications
- **`/cmd/steer`** (`std_msgs/Float32`): steering angle [deg], left turn positive.
- **`/pure_pursuit/debug_markers`** (`visualization_msgs/MarkerArray`): RViz debugging.

### RViz Markers (namespaces)
- `pp/lookahead_circle` — LINE_STRIP, radius = `L_d` (visual indicator).
- `pp/pw`, `pp/pd`, `pp/pl` — SPHERE points (near, default, final target).
- `pp/tangent_at_pd` — ARROW along local tangent at `pd`.
- `pp/pd_to_pl` — LINE_LIST from `pd` to `pl` (offset visualization).
- `pp/selected_path` — LINE_STRIP highlight around `pw … pd` (optional).
- `pp/steer_arrow`, `pp/steer_text` — steering command arrow & label near the front axle.

---

## 8) Parameters (with Units, Defaults & Effects)

### I/O & Frames
| Name | Type | Default | Meaning |
|---|---|---:|---|
| `path_topic` | string | `/local_planned_path` | Input path topic |
| `speed_topic` | string | `/current_speed` | Input speed topic [m/s] |
| `steer_topic` | string | `/cmd/steer` | Output steering [deg] |
| `debug_marker_topic` | string | `/pure_pursuit/debug_markers` | RViz markers |
| `base_frame` | string | `base_link` | Fallback frame if path has no frame |

### Vehicle
| Name | Type | Default | Meaning |
|---|---|---:|---|
| `wheelbase_m` | double | **1.3** | Wheelbase \(L\) [m] |

### LAD (Look‑Ahead Distance)
| Name | Type | Default | Meaning |
|---|---|---:|---|
| `use_speed_term` | bool | **true** | Enable \(L_0 + k_v v\) |
| `L0` | double | 1.5 | Base LAD [m] |
| `k_v` | double | 0.6 | Speed gain [s] (m per m/s) |
| `use_curvature_term` | bool | **true** | Enable curvature‑aware term |
| `k_curv` | double | 0.0 | Curvature gain |
| `epsilon_kappa` | double | 1e-6 | Denominator guard |
| `Ld_min`, `Ld_max` | double | 1.0, 6.0 | LAD clamp [m] |
| `curv_window_m` | double | 2.0 | Sampling distance for curvature |

### Target Selection & Forward Filtering
| Name | Type | Default | Meaning |
|---|---|---:|---|
| `use_arc_length_selection` | bool | **true** | Arc‑length vs Euclidean |
| `x_forward_only` | bool | **true** | Enforce forward‑looking points |
| `forward_margin_x` | double | −0.2 | Forward threshold \(x >\) margin [m] |
| `sticky_window_pts` | int | 15 | Sticky window half‑width for `pw`/`pd` |
| `kappa_smooth_window_pts` | int | 3 | Curvature smoothing half‑width |
| `target_ema_tau` | double | 0.08 | EMA for `pl` [s] |

### Outward Offset (Corner‑Cutting Suppression)
| Name | Type | Default | Meaning |
|---|---|---:|---|
| `outer_offset_enable` | bool | **true** | Enable pd→pl outward shift |
| `alpha_max_m` | double | 3.0 | Proximity scale for \(\alpha\) |
| `beta_max` | double | 3.0 | Curvature ratio scale for \(\beta\) |
| `outer_offset_max_m` | double | 1.0 | Max lateral offset [m] |
| `outer_offset_tau_max` | double | 0.7 | Max weight \(\tau\) |
| `outer_offset_kappa_gate` | double | 0.03 | Disable offset if \(|\kappa|<\) gate |
| `track_half_width_m` | double | 0.0 | Track half width (0=unknown) |
| `track_margin_m` | double | 0.2 | Safety margin to boundary [m] |

### Command Shaping
| Name | Type | Default | Meaning |
|---|---|---:|---|
| `publish_rate_hz` | double | 50.0 | Control loop rate [Hz] |
| `steer_limit_deg` | double | 30.0 | Saturation [deg] |
| `steer_rate_limit_deg_per_s` | double | 360.0 | Rate limit [deg/s] |
| `ema_tau_cmd` | double | 0.12 | EMA for steering [s] |
| `ema_tau_speed` | double | 0.20 | EMA for speed [s] |

### Visualization
| Name | Type | Default | Meaning |
|---|---|---:|---|
| `show_lookahead_circle` | bool | true | Show LAD circle |
| `show_points` | bool | true | Show pw/pd/pl spheres |
| `show_tangent` | bool | true | Show tangent at pd |
| `show_offset_vec` | bool | true | Line from pd to pl |
| `show_selected_path` | bool | false | Highlight pw…pd segment |
| `show_steer_arrow` | bool | true | Arrow showing steer cmd |
| `show_steer_text` | bool | true | Text with steer value |
| `marker_alpha` | double | 1.0 | Global marker alpha |
| `circle_points` | int | 60 | Circle tessellation |

**Colors (RGB 0–1):**
- `color_pw_r/g/b` (near point, default `0.10, 0.60, 1.00`)
- `color_pd_r/g/b` (default target, `0.10, 1.00, 0.10`)
- `color_pl_r/g/b` (final target, `1.00, 0.10, 0.80`)
- `color_tan_r/g/b` (tangent arrow, `1.00, 0.85, 0.10`)
- `color_vec_r/g/b` (offset vector, `0.00, 0.90, 1.00`)
- `color_cir_r/g/b` (lookahead circle, `1.00, 1.00, 1.00`)

---

## 9) Implementation Map (code → behavior)

- `computeLd()` → **§3 LAD formula** (speed/curvature terms + clamps).
- `onPath()` → populate `pts_`, `cum_s_`, `frame_id_from_path_`.
- `findNearestIndexSticky()` → **§4.1** sticky nearest `pw`.
- `selectTargetIndexSticky()` → **§4.2** arc‑length/Euclidean `pd` + sticky clamp + forward filter.
- `tangentAt()` / `kappaSignedRawAt()` / `signedKappaSmoothedAt()` / `evalCurvatureAbsAt()` → **§6** geometry primitives.
- Outward shift block in `onTimer()` → **§4.3** `pd → pl` outward offset + gating + track bounds + EMA(x,y).
- `smoothDeg()` → **§5** rate‑limit + EMA + clamp.
- `publishDebugMarkers()` → **§7** RViz marker namespaces and styles.

---

## 10) Tuning Guide (Quick)

- **Too twitchy / oscillatory:** increase `sticky_window_pts`, `target_ema_tau`, `ema_tau_cmd`; reduce `steer_rate_limit_deg_per_s`.
- **Cuts inside on corners:** enable `outer_offset_enable`; lower `outer_offset_kappa_gate` (e.g., 0.02–0.03); raise `outer_offset_tau_max` (0.8), `outer_offset_max_m` (1.2–1.5); set `track_half_width_m` if known.
- **Wiggly on straights at high speed:** increase `Ld_max`, `k_v`; add small positive `k_curv` (0.3–1.0).
- **Understeer (wide on corners):** decrease `Ld_min` (0.8–1.0), reduce `k_v`, set `k_curv` to 0 or lower.

---

## 11) Matching Legacy Behavior (optional)

To emulate the older `pure_pursuit_dynamic.cpp` behavior:
```yaml
use_arc_length_selection: false
x_forward_only: true
forward_margin_x: 0.0
sticky_window_pts: 100000        # effectively disables sticky clamp
use_speed_term: true
L0: 1.5
k_v: 0.6
Ld_min: 1.0
Ld_max: 5.0
use_curvature_term: false
k_curv: 0.0
outer_offset_enable: false       # pl == pd
ema_tau_cmd: 0.1
steer_rate_limit_deg_per_s: 1.0e9
```
This makes `pd` and the steering command closely resemble the previous node (still not strictly identical due to internal state and sampling).

---

## 12) FAQ

**Q. Why is `‖pd‖` not equal to `L_d` when I draw the lookahead circle?**  
A. In **arc‑length mode**, `L_d` is *along the path*, not Euclidean distance. On curves, the Euclidean distance of `pd` is typically **less** than `L_d`.

**Q. Which point does steering use?**  
A. Always **`pl`** (which equals `pd` if outward offset is disabled/gated).

**Q. Why are my markers noisy?**  
A. Increase `sticky_window_pts`, `target_ema_tau`, `ema_tau_cmd`. Ensure the input path is *evenly sampled* and low noise.

---
