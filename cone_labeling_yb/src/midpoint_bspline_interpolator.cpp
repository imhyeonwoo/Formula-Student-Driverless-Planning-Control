// src/midpoint_bspline_to_path.cpp
#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <vector>
#include <algorithm>
#include <cmath>

using visualization_msgs::msg::MarkerArray;
using visualization_msgs::msg::Marker;
using geometry_msgs::msg::Point;
using nav_msgs::msg::Path;
using geometry_msgs::msg::PoseStamped;

class MidpointBSplineToPath : public rclcpp::Node {
public:
  MidpointBSplineToPath()
  : Node("midpoint_bspline_to_path")
  {
    sub_ = create_subscription<MarkerArray>(
      "/cone_delaunay_midpoints",
      rclcpp::SystemDefaultsQoS(),
      std::bind(&MidpointBSplineToPath::callback, this, std::placeholders::_1)
    );
    // B-spline 점들을 Path로 퍼블리시 (속도 플래너가 구독)
    path_pub_ = create_publisher<Path>("/local_planned_path", 10);
    RCLCPP_INFO(get_logger(), "MidpointBSplineToPath started");
  }

private:
  void callback(const MarkerArray::SharedPtr msg) {
    if (msg->markers.empty()) return;

    // 1) 중점들 수집 (ADD 액션만)
    std::vector<Point> pts;
    for (const auto &m : msg->markers) {
      if (m.action != Marker::ADD) continue;
      pts.push_back(m.pose.position);
    }
    if (pts.size() < 4) {
      RCLCPP_WARN(get_logger(),
                  "Need at least 4 points for B-spline (got %zu)", pts.size());
      return;
    }

    // 2) 원점(0,0)으로부터 거리 오름차순 정렬
    std::sort(pts.begin(), pts.end(),
      [](const Point &a, const Point &b) {
        double da2 = a.x * a.x + a.y * a.y;
        double db2 = b.x * b.x + b.y * b.y;
        return da2 < db2;
      }
    );

    // 3) Vec2 벡터로 변환
    struct Vec2 { double x, y; };
    std::vector<Vec2> ctrl;
    ctrl.reserve(pts.size());
    for (auto &p : pts) {
      ctrl.push_back({p.x, p.y});
    }

    int n_ctrl = static_cast<int>(ctrl.size());
    int degree = 3;
    int k = degree;
    int m = n_ctrl + k + 1;

    // 4) 균등 클램프드 knot 벡터 생성
    std::vector<double> knots(m);
    for (int i = 0; i < m; ++i) {
      if (i <= k)           knots[i] = 0.0;
      else if (i >= n_ctrl) knots[i] = 1.0;
      else                  knots[i] = double(i - k) / double(n_ctrl - k);
    }

    // 5) de Boor 알고리즘으로 raw 곡선 샘플링
    std::vector<Point> raw;
    const double dt = 0.005;
    raw.reserve(static_cast<size_t>(1.0/dt) + 1);
    for (double t = 0.0; t <= 1.0; t += dt) {
      int span = findSpan(n_ctrl - 1, degree, t, knots);
      std::vector<Vec2> d(degree+1);
      for (int j = 0; j <= degree; ++j)
        d[j] = ctrl[span - degree + j];
      for (int r = 1; r <= degree; ++r) {
        for (int j = degree; j >= r; --j) {
          double denom = knots[span + 1 + j - r] -
                         knots[span - degree + j];
          double alpha = (denom > 0.0)
                       ? (t - knots[span - degree + j]) / denom
                       : 0.0;
          d[j].x = (1.0 - alpha) * d[j-1].x + alpha * d[j].x;
          d[j].y = (1.0 - alpha) * d[j-1].y + alpha * d[j].y;
        }
      }
      Point pp;
      pp.x = d[degree].x;
      pp.y = d[degree].y;
      pp.z = 0.0;
      raw.push_back(pp);
    }

    // 6) 약 0.5m 간격으로 재샘플링
    std::vector<Point> samples;
    const double target_dist = 0.5;
    double acc = 0.0;
    Point prev = raw.front();
    samples.push_back(prev);
    for (size_t i = 1; i < raw.size(); ++i) {
      auto &cur = raw[i];
      double d = std::hypot(cur.x - prev.x, cur.y - prev.y);
      acc += d;
      if (acc >= target_dist) {
        samples.push_back(cur);
        acc = 0.0;
      }
      prev = cur;
    }

    // 7) Path 메시지로 퍼블리시
    Path path_msg;
    path_msg.header.stamp    = now();
    path_msg.header.frame_id = msg->markers.front().header.frame_id;

    for (const auto &p : samples) {
      PoseStamped ps;
      ps.header = path_msg.header;
      ps.pose.position = p;
      // 방향은 정의되지 않으므로 단위 쿼터니언으로 설정
      ps.pose.orientation.x = 0.0;
      ps.pose.orientation.y = 0.0;
      ps.pose.orientation.z = 0.0;
      ps.pose.orientation.w = 1.0;
      path_msg.poses.push_back(ps);
    }
    path_pub_->publish(path_msg);
  }

  // knot 벡터에서 span 찾기 (원본과 동일)
  int findSpan(int n, int p, double u,
               const std::vector<double> &U)
  {
    if (u >= U[n+1]) return n;
    if (u <= U[p])   return p;
    int low = p, high = n+1, mid = (low + high) / 2;
    while (u < U[mid] || u >= U[mid+1]) {
      if (u < U[mid]) high = mid;
      else            low  = mid;
      mid = (low + high) / 2;
    }
    return mid;
  }

  rclcpp::Subscription<MarkerArray>::SharedPtr sub_;
  rclcpp::Publisher<Path>::SharedPtr           path_pub_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MidpointBSplineToPath>());
  rclcpp::shutdown();
  return 0;
}
