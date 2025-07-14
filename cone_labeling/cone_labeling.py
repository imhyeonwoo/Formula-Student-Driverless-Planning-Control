import cv2
import numpy as np
import math
import matplotlib.pyplot as plt
from scipy.spatial import KDTree
import time

def euclidean_distance(p1, p2): # 유클리드 거리
    return np.sqrt((p1[0] - p2[0])**2 + (p1[1] - p2[1])**2)

def angle_between_vectors(v1, v2): # 라디안측정
    dot_product = v1[0]*v2[0] + v1[1]*v2[1]
    norm_v1 = math.hypot(*v1)
    norm_v2 = math.hypot(*v2)
    if norm_v1 == 0 or norm_v2 == 0:
        return 0.0
    cos_theta = np.clip(dot_product / (norm_v1 * norm_v2), -1.0, 1.0)
    return math.acos(cos_theta)

#######################################
# 2) 이미지에서 빨간점, 검은점 검출 함수 #
#######################################

def detect_points_with_red_origin(image_path):
    """
    이미지에서 빨간 점을 (0,0) 기준으로 설정하고,
    검은 점들의 상대좌표를 반환
    """
    img_bgr = cv2.imread(image_path)
    if img_bgr is None:
        raise FileNotFoundError(f"이미지 로딩 실패: {image_path}")
    
    img_rgb = cv2.cvtColor(img_bgr, cv2.COLOR_BGR2RGB)

    # 색상 범위 정의 (RGB 기준)
    lower_black = np.array([0, 0, 0])
    upper_black = np.array([50, 50, 50])
    lower_red = np.array([150, 0, 0])
    upper_red = np.array([255, 100, 100])

    # 마스크 생성
    mask_black = cv2.inRange(img_rgb, lower_black, upper_black)
    mask_red   = cv2.inRange(img_rgb, lower_red,   upper_red)

    # 컨투어 검출
    contours_black, _ = cv2.findContours(mask_black, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    contours_red, _   = cv2.findContours(mask_red,   cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    # 첫 번째 빨간 점을 기준점으로 삼음
    red_origin = None
    for cnt in contours_red:
        M = cv2.moments(cnt)
        if M["m00"] != 0:
            cx = int(M["m10"] / M["m00"])
            cy = int(M["m01"] / M["m00"])
            red_origin = (cx, cy)
            break

    if red_origin is None:
        raise ValueError("빨간 점이 감지되지 않았습니다.")

    rx, ry = red_origin
    black_points = []

    # 검은 점의 중심 좌표를 상대좌표로 변환
    for cnt in contours_black:
        M = cv2.moments(cnt)
        if M["m00"] != 0:
            cx = int(M["m10"] / M["m00"])
            cy = int(M["m01"] / M["m00"])
            new_x = cx - rx
            new_y = -(cy - ry)  # y 방향은 위쪽이 +이 되도록 반전
            black_points.append((new_x, new_y))

    return (0, 0), black_points

########################################
# 3) 방향 제한 + 거리 제한 경로 생성 함수 #
########################################

def generate_multi_stage_oriented_path_with_min_distance(
    start_point, all_points,
    radius=170, min_distance=50,
    angle_limits=[math.radians(30), math.radians(45), math.radians(60), math.radians(90)],
    initial_direction=(0, 1)
):
    """
    시작점부터 방향제한 + 최소거리 조건을 만족하는 점을 찾아 경로 생성
    - 반경 radius 내에서
    - 거리 ≥ min_distance
    - 각도 제한 조건을 점차 완화(30°, 45°, 60°, 90°)
    """
    if not all_points:
        return []

    tree = KDTree(all_points)
    path = [start_point]
    indexed_path = [(start_point, 0)]
    current_point = start_point
    direction_vector = initial_direction
    index = 1

    while True:
        # 반경 내 후보 탐색 (이미 지나온 점은 제외)
        candidates_idx = tree.query_ball_point(current_point, radius)
        candidates = [
            all_points[i]
            for i in candidates_idx
            if all_points[i] not in path and euclidean_distance(current_point, all_points[i]) >= min_distance
        ]
        if not candidates:
            break

        # 각도 제한을 점차 완화하며 적합한 후보 탐색
        valid_candidates = None
        for angle_limit in angle_limits:
            filtered = []
            for pt in candidates:
                vec = (pt[0] - current_point[0], pt[1] - current_point[1])
                angle = angle_between_vectors(direction_vector, vec)
                if angle <= angle_limit:
                    filtered.append(pt)
            if filtered:
                valid_candidates = filtered
                break

        if not valid_candidates:
            break

        # 가장 가까운 후보 선택
        next_point = min(valid_candidates, key=lambda p: euclidean_distance(current_point, p))
        path.append(next_point)
        indexed_path.append((next_point, index))

        # 다음 방향 설정
        direction_vector = (next_point[0] - current_point[0], next_point[1] - current_point[1])
        current_point = next_point
        index += 1

    return indexed_path

##################################
# 4) 시작점 자동 설정 함수 정의 #
##################################

def get_start_point(black_points, side='left', radius=200):
    """
    차량 기준 좌측 또는 우측에서 반경 내 가장 가까운 점을 시작점으로 설정
    - 없으면 강제로 (-radius, 0) 또는 (radius, 0)
    """
    filtered = []
    for pt in black_points:
        if (side == 'left' and pt[0] < 0) or (side == 'right' and pt[0] > 0):
            if euclidean_distance((0,0), pt) <= radius:
                filtered.append(pt)

    if filtered:
        return min(filtered, key=lambda p: euclidean_distance((0,0), p))
    else:
        return (-radius, 0) if side == 'left' else (radius, 0)

####################################
# 5) 전체 시각화 함수 정의 (matplotlib) #
####################################

def visualize_path(black_points, red_origin, left_path, right_path, title="Multi-Stage Oriented Path"):
    """
    검은 점들, 기준점(빨간 점), 좌/우 경로를 시각적으로 표현
    """
    plt.figure(figsize=(8, 6))

    if black_points:
        x, y = zip(*black_points)
        plt.scatter(x, y, color='black', label='Detected Cones')

    # 차량 기준점 (빨간 점)
    plt.scatter(*red_origin, color='red', s=80, label='Vehicle')
    plt.text(red_origin[0]+2, red_origin[1]+2, "(0, 0)", fontsize=9, color='red')

    # 왼쪽 경로: 초록색
    for pt, idx in left_path:
        plt.scatter(*pt, color='green', s=50)
        plt.text(pt[0]+2, pt[1]+2, str(idx), fontsize=9, color='green')

    # 오른쪽 경로: 보라색
    for pt, idx in right_path:
        plt.scatter(*pt, color='purple', s=50)
        plt.text(pt[0]+2, pt[1]+2, str(idx), fontsize=9, color='purple')

    plt.axhline(0, color='gray', linestyle='--')
    plt.axvline(0, color='gray', linestyle='--')
    plt.xlabel("X-axis")
    plt.ylabel("Y-axis")
    plt.title(title)
    plt.grid(True)
    plt.legend()
    plt.show()

##############################
# 6) 메인 로직 (실행 예시용) #
##############################

def main():
    start_time = time.time()

    image_path = "test2.jpg"  # 이미지 파일명

    # 이미지에서 점 추출 (빨간 점 = (0,0), 검은 점 = 상대좌표)
    red_origin, black_points = detect_points_with_red_origin(image_path)

    # 좌/우 시작점 자동 설정, 근데 얘는 임의로 설정한거긴함.
    left_start = get_start_point(black_points, side='left', radius=200)
    right_start = get_start_point(black_points, side='right', radius=200)

    # 초기 방향: 차량이 위쪽(양의 y축)을 바라본다고 가정
    init_dir = (0, 1)

    # 양쪽 경로 생성
    left_path = generate_multi_stage_oriented_path_with_min_distance(
        start_point=left_start,
        all_points=black_points,
        radius=120,
        min_distance=10,
        angle_limits=[math.radians(15), math.radians(30), math.radians(45), math.radians(60)],
        initial_direction=init_dir
    )
    right_path = generate_multi_stage_oriented_path_with_min_distance(
        start_point=right_start,
        all_points=black_points,
        radius=120,
        min_distance=10,
        angle_limits=[math.radians(15), math.radians(30), math.radians(45), math.radians(60)],
        initial_direction=init_dir
    )

    # 처리 시간 출력
    print(f"Processing Time: {time.time() - start_time:.3f} sec")

    # 시각화
    visualize_path(black_points, red_origin, left_path, right_path, title="Cone Path Planning")

if __name__ == "__main__":
    main()

