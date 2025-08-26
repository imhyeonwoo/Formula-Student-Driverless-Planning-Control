// src/speed_planning.cpp
// ──────────────────────────────────────────────────────────────
// ROS 2 Node for curvature-based local speed planning with RViz bar-height visualization
// • 입력: /local_planned_path (nav_msgs/Path)
// • 출력 1: /desired_speed_profile (std_msgs/Float32MultiArray)
// • 출력 2: /speed_markers        (visualization_msgs/MarkerArray)
//   – 각 경로점 위에 속도값에 비례하는 높이(막대기) 표시
// ──────────────────────────────────────────────────────────────

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/path.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <vector>
#include <algorithm>
#include <cmath>
#include <limits>

using visualization_msgs::msg::Marker;
using visualization_msgs::msg::MarkerArray;

class SpeedPlanner : public rclcpp::Node {
public:
  SpeedPlanner()
  : Node("speed_planner")
  {
    // 파라미터 선언 및 초기값
    declare_parameter<double>("a_long_max", 3.0);
    declare_parameter<double>("a_brake_max", 3.5);
    declare_parameter<double>("a_lat_max", 3.0);
    declare_parameter<double>("diff_eps",   0.05);
    get_parameter("a_long_max",  a_long_max_);
    get_parameter("a_brake_max", a_brake_max_);
    get_parameter("a_lat_max",   a_lat_max_);
    get_parameter("diff_eps",    diff_eps_);

    // 퍼블리셔
    speed_pub_  = create_publisher<std_msgs::msg::Float32MultiArray>(
      "/desired_speed_profile", 10);
    marker_pub_ = create_publisher<MarkerArray>(
      "/speed_markers", 10);

    // 구독자
    path_sub_ = create_subscription<nav_msgs::msg::Path>(
      "/local_planned_path", 10,
      std::bind(&SpeedPlanner::pathCallback, this, std::placeholders::_1)
    );
  }

private:
  void pathCallback(const nav_msgs::msg::Path::SharedPtr msg) {
    const auto &poses = msg->poses;
    size_t n = poses.size();
    if (n == 0) return;

    // 경로가 하나면 속도 0 고정
    if (n == 1) {
      std_msgs::msg::Float32MultiArray speed_msg;
      speed_msg.data = {0.0f};
      speed_pub_->publish(speed_msg);
      publishSpeedMarkers(poses, {0.0});
      prev_speed_ = {0.0};
      return;
    }

    // 거리 계산
    std::vector<double> dist(n, 0.0);
    for (size_t i = 0; i < n-1; ++i) {
      double dx = poses[i+1].pose.position.x - poses[i].pose.position.x;
      double dy = poses[i+1].pose.position.y - poses[i].pose.position.y;
      double dz = poses[i+1].pose.position.z - poses[i].pose.position.z;
      dist[i] = std::hypot(dx, dy, dz);
      if (dist[i] < 1e-6) dist[i] = 1e-6;
    }

    // 곡률 기반 속도 한계
    std::vector<double> curvature_speed(n, std::numeric_limits<double>::infinity());
    for (size_t i = 1; i < n-1; ++i) {
      double x0 = poses[i-1].pose.position.x, y0 = poses[i-1].pose.position.y;
      double x1 = poses[i  ].pose.position.x, y1 = poses[i  ].pose.position.y;
      double x2 = poses[i+1].pose.position.x, y2 = poses[i+1].pose.position.y;
      double dx1 = x1 - x0, dy1 = y1 - y0;
      double dx2 = x2 - x0, dy2 = y2 - y0;
      double cross = std::abs(dx1*(y2 - y0) - dy1*(x2 - x0));
      double d01 = std::hypot(dx1, dy1),
             d02 = std::hypot(dx2, dy2),
             d12 = std::hypot(x2 - x1, y2 - y1);
      if (d01 < 1e-6 || d02 < 1e-6 || d12 < 1e-6) continue;
      double curvature = (2.0 * cross) / (d01 * d02 * d12);
      if (curvature > 1e-9) {
        curvature_speed[i] = std::sqrt(a_lat_max_ / curvature);
      }
    }

    // 초기 속도 프로파일 = 곡률 한계
    std::vector<double> speed(n);
    for (size_t i = 0; i < n; ++i) {
      speed[i] = std::isfinite(curvature_speed[i]) ? curvature_speed[i] : 1e6;
    }

    // 가속 한계 (forward pass)
    speed[0] = 0.0;
    for (size_t i = 1; i < n; ++i) {
      double vlim = std::sqrt(speed[i-1]*speed[i-1] + 2.0*a_long_max_*dist[i-1]);
      speed[i] = std::min(speed[i], vlim);
    }

    // 감속 한계 (backward pass)
    for (int i = static_cast<int>(n)-2; i >= 0; --i) {
      double vlim = std::sqrt(speed[i+1]*speed[i+1] + 2.0*a_brake_max_*dist[i]);
      speed[i] = std::min(speed[i], vlim);
    }

    // 3-point 이동평균 스무딩
    std::vector<double> smooth_speed(n);
    if (n >= 3) {
      smooth_speed[0]   = speed[0];
      smooth_speed[n-1] = speed[n-1];
      for (size_t i = 1; i < n-1; ++i) {
        double s = 0.25*speed[i-1] + 0.5*speed[i] + 0.25*speed[i+1];
        smooth_speed[i] = std::min(s, speed[i]);
      }
    } else {
      smooth_speed = speed;
    }

    // 변화량 검사
    bool changed = (prev_speed_.size() != n);
    if (!changed) {
      for (size_t i = 0; i < n; ++i) {
        if (std::fabs(smooth_speed[i] - prev_speed_[i]) > diff_eps_) {
          changed = true;
          break;
        }
      }
    }
    if (!changed) return;

    // 속도 프로파일 퍼블리시
    std_msgs::msg::Float32MultiArray speed_msg;
    speed_msg.data.reserve(n);
    for (double v : smooth_speed) {
      speed_msg.data.push_back(static_cast<float>(v));
    }
    speed_pub_->publish(speed_msg);

    // RViz 막대 높이 시각화
    publishSpeedMarkers(poses, smooth_speed);

    // 캐시 갱신
    prev_speed_.swap(smooth_speed);
  }

  // MarkerArray로 각 경로점에 속도 비례 막대기 표시
  void publishSpeedMarkers(
    const std::vector<geometry_msgs::msg::PoseStamped>& poses,
    const std::vector<double>& speeds)
  {
    if (poses.empty() || poses.size() != speeds.size()) return;

    MarkerArray ma;

    // 이전 마커 삭제
    Marker clear;
    clear.header.frame_id = poses[0].header.frame_id;
    clear.header.stamp    = now();
    clear.action          = Marker::DELETEALL;
    ma.markers.push_back(clear);

    // 새로운 막대기 생성
    for (size_t i = 0; i < poses.size(); ++i) {
      Marker m;
      m.header = poses[i].header;
      m.ns     = "speed_bars";
      m.id     = static_cast<int>(i);
      m.type   = Marker::CUBE;
      m.action = Marker::ADD;

      double h = speeds[i];
      // 막대기 중심을 높이의 절반만큼 올려서 Z=0에서 바닥에 놓이도록
      m.pose.position.x = poses[i].pose.position.x;
      m.pose.position.y = poses[i].pose.position.y;
      m.pose.position.z = h * 0.5;

      // 막대 굵기: X,Y 0.2m, Z는 속도값
      m.scale.x = 0.2;
      m.scale.y = 0.2;
      m.scale.z = h;

      // 색상(예시: 파랑 계열, 투명도 0.8)
      m.color.r = 0.0f;
      m.color.g = 0.5f;
      m.color.b = 1.0f;
      m.color.a = 0.8f;

      ma.markers.push_back(m);
    }

    marker_pub_->publish(ma);
  }

  // 멤버
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr       path_sub_;
  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr speed_pub_;
  rclcpp::Publisher<MarkerArray>::SharedPtr                  marker_pub_;

  double a_long_max_, a_brake_max_, a_lat_max_, diff_eps_;
  std::vector<double> prev_speed_;
};

int main(int argc, char * argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SpeedPlanner>());
  rclcpp::shutdown();
  return 0;
}
