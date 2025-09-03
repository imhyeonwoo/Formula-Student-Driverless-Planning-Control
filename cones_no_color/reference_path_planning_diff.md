# 로컬 경로 생성 노드 변경 요약 (`reference_path_planning.py`)

**대상 파일**
- **구버전(백업)**: `src/Planning/cones_no_color/scripts/reference_path_planning.py.BAK`  
- **신버전(현재 사용)**: `src/Planning/cones_no_color/scripts/reference_path_planning.py` (anchored & forward-stable, 2025-09-03)

본 문서는 두 파일의 **알고리즘/구조/파라미터/출력 정책** 차이를 정리합니다.

---

## 1) 한눈에 비교

| 구분 | 구버전 (`*.BAK`) | 신버전 (`reference_path_planning.py`) |
|---|---|---|
| 센터라인 후보 | Delaunay 좌↔우 **내부선 중점** | Delaunay **중점** + **중복 제거(5cm voxel)** + **PCA 1D 정렬** + **갭 브리징** |
| 거리 필터 | `max_lane_width` 상한만 | `min_lane_width` + `max_lane_width` **하·상한 모두 적용** |
| 정렬 방식 | 최근접-이웃(Nearest Neighbor) 휴리스틱 | **PCA 투영값 정렬**로 전방성 유지(필요 시 반전) |
| 스무딩(공간) | `make_interp_spline`(t 균일) 실패 시 선형 | **chord-length B-spline (`splprep/splev`)** + 길이 비례 약한 스무딩(`smooth_coef`) |
| 스무딩(시간) | 없음 | **Temporal EMA**(지수평활)로 프레임 간 흔들림 완화 |
| 출력 정책 | 등간격 샘플(자연 길이, 가변 개수) | **(0,0) 앵커**에서 시작하는 **고정 길이** `fixed_length_m`, **고정 간격** `ds_out` |
| 뒤쪽 데이터 | 별도 처리 없음 | `forward_x_margin` 기준으로 **원점 뒤쪽 제거** |
| 데이터 부족 | 좌우 모두 3점 미만이면 중단 | **단측 fallback**: 한쪽 콘만 있어도 **법선 오프셋**으로 센터 합성 |
| 차선 폭 추정 | 고정 상한 의존 | **실시간 폭 추정(중앙값 기반)** → `half_width_est` 업데이트 |
| 속도 프로파일 | 길이 불일치 허용, 가능한 범위만 반영 | 동일 + **anchored 등간격 경로**에 적용되어 시각화 안정 |
| TF/프레임 | `map→base_link` 조회, 출력 `base_link` | 동일 |
| 디버그 마커 | 내부선/중점/최종 WP/속도바 | 동일(거리 필터 로직도 시각화에 동일 적용) |

---

## 2) 로직별 상세 변화

### A. 중점 생성·정렬
- **구버전**: Delaunay 내부선 중점 → **최근접-이웃 정렬**. 프레임마다 연결 순서가 바뀌며 **지그재그/되돌림** 가능.
- **신버전**:  
  1) **중복 제거**(voxel 0.05m) → 밀집·중복점 제거  
  2) **PCA 1D 정렬**로 단조 진행(전방성 유지, 필요 시 반전)  
  3) **갭 브리징**(임계 초과 시 선형 보강)  
  ⮕ **끊김/순서 불안정 완화**, 프레임 간 일관성↑

### B. 스무딩
- **구버전**: `make_interp_spline`(균일 파라미터) → 포인트 불균일 시 굴곡·진동 가능, 실패 시 선형 보간.
- **신버전**: **chord-length 기반 B-spline** (`splprep/soplev`) + 경로 전체 길이에 비례해 **스무딩 강도(`smooth_coef`)** 적용 → **곡률 급변 억제**.

### C. 시간적 안정화(Temporal EMA)
- **구버전**: 없음.
- **신버전**: **index-wise EMA**(이전 프레임 경로를 현재 개수에 맞춰 리샘플 후 블렌딩) → **프레임 간 흔들림(LAD류 노이즈) 감쇠**.

### D. 출력 정책(Anchored & Forward)
- **구버전**: 생성된 경로를 등간격 리샘플(길이·개수 가변).
- **신버전**: `base_link (0,0)`에 **가장 가까운 경로점 Q\***를 찾아 **원점→Q\*** 연결 후, **앞으로 `fixed_length_m`** 만큼 **`ds_out` 간격**으로 샘플.  
  - 부족 시 **마지막 접선 방향**으로 자연 연장  
  - `forward_x_margin`으로 **원점 뒤쪽 제거**  
  ⮕ **Ego 기준 일정 길이·간격** 보장 → **제어기 튠 용이성↑**

### E. 차선 폭 관리/단측 복원
- **구버전**: `max_lane_width` 상한만 적용 → 너무 좁은(노이즈성) 에지 통과 가능.
- **신버전**: `min_lane_width`+`max_lane_width`로 **유효 폭**만 채택. 좌우가 모두 보일 때 **중앙값 기반 폭 추정**으로 `half_width_est` 업데이트. 한쪽만 보일 때 **법선 오프셋 fallback**으로 **주행 지속성** 확보.

### F. 속도 프로파일 & 마커
- **두 버전 공통**: `/desired_speed_profile` 길이 불일치 허용, 가능한 범위만 반영.  
- **신버전 장점**: 등간격·고정길이 경로라 **속도 막대(막대 높이=z)**와 **WP 인덱스**가 프레임 간 안정적.

---

## 3) 파라미터 변화(주요 항목)

- **출력 간격/길이**: `ds_out=0.20`, `fixed_length_m=10.0`, `forward_x_margin=0.3`
- **브릿지 폭 필터**: `min_lane_width=1.0`, `max_lane_width=6.0`
- **후처리**: `voxel_dedup_m=0.05`, `gap_bridge_th=1.2`, `bridge_step=0.6`
- **스무딩**: `smooth_coef=0.05` (길이 비례, 0.02~0.06 권장)
- **EMA**: `enable_temporal_ema=True`, `temporal_alpha=0.85`
- **단측 fallback 폭**: `half_width_est=2.5`(동적 업데이트), `half_width_min=1.0`, `half_width_max=4.0`

---

## 4) 마이그레이션 체크리스트

1. **노드 교체 후 즉시 동작**(토픽/프레임 동일) 확인
2. **컨트롤러 재튜닝**  
   - Lookahead: `lookahead ≈ k × ds_out`로 재정의(예: k=5~15)  
   - 속도 정책: 고정 길이 `fixed_length_m` 고려해 미리보기 구간 조정
3. **EMA 알파**: 흔들림이 크면 `0.85~0.9`, 응답성을 더 원하면 `0.7~0.8`
4. **폭 필터/half_width_est**: 실제 코스 폭 통계에 맞게 조정
5. **디버그 마커 확인**: Delaunay 내부선/중점/브리징이 의도대로 필터링되는지 RViz에서 점검

---

## 5) 기대 효과 요약

- **경로 떨림 감소**: PCA 정렬 + chord-length 스무딩 + EMA
- **제어기 튠 용이**: 항상 동일한 간격·길이의 **앵커드 경로**
- **센서 드롭아웃/부분가림 내성**: 단측 fallback + 폭 추정
- **아웃라이어 억제**: min/max 폭 필터, 중복 제거, 갭 브리징

---

## 6) 변경점 맵핑(코드 레벨 키워드)

- `delaunay_midpoints`: **min/max 폭 필터 추가**, 반환 전처리 파이프라인 변경
- `_dedup_points / _order_by_pca / _bridge_gaps`: **중점 후처리** 신규
- `_smooth_centerline`: **splprep/splev + 길이 비례 스무딩**
- `_anchor_fixed_length`: **(0,0) 앵커 + 고정 길이 + 고정 간격** 생성
- `_forward_sanitize`: **원점 뒤쪽 제거 + 0-length 제거**
- `_temporal_smooth`: **EMA 기반 프레임 간 블렌딩**
- `_center_from_single_side / _update_half_width_est`: **단측 fallback + 폭 추정**

---

### 부록: 기본 권장 프리셋
- **좁은 서킷/급커브**: `ds_out=0.15`, `smooth_coef=0.03`, `temporal_alpha=0.8`
- **와이드 트랙/완만**: `ds_out=0.25`, `smooth_coef=0.05`, `temporal_alpha=0.87`
- **콘 누락 잦음**: `gap_bridge_th ↑(1.5~1.8)`, `bridge_step ↓(0.4)`

