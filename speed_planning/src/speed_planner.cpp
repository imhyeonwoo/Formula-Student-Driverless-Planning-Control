/****************************************************************
 * speed_planner.cpp  (ROS 2 Humble · 2025-07-14 rev-3)
 ****************************************************************/
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <Eigen/Dense>
#include <vector>
#include <algorithm>
#include <cmath>

using rclcpp::Node;
using visualization_msgs::msg::Marker;
using visualization_msgs::msg::MarkerArray;
using std_msgs::msg::Float32MultiArray;
using Eigen::Vector2d;

/* ───────── 세 점 곡률 ───────── */
static double curvature(const Vector2d &p_prev,
                        const Vector2d &p_curr,
                        const Vector2d &p_next)
{
  const double a = (p_curr - p_prev).norm();
  const double b = (p_next - p_curr).norm();
  const double c = (p_next - p_prev ).norm();
  if (a < 1e-4 || b < 1e-4 || c < 1e-4) return 0.0;
  const double s  = 0.5 * (a + b + c);
  const double A2 = s * (s - a) * (s - b) * (s - c);
  if (A2 <= 0.0) return 0.0;
  return 4.0 * std::sqrt(A2) / (a * b * c);
}

/* ───────────── SpeedPlanner ───────────── */
class SpeedPlanner : public Node
{
public:
  SpeedPlanner() : Node("speed_planner")
  {
    /* 파라미터 */
    a_lat_max_   = declare_parameter("a_lat_max",    3.0);
    a_long_max_  = declare_parameter("a_long_max",   2.0);
    lookahead_n_ = declare_parameter("lookahead_n", 10);
    arc_step_    = declare_parameter("arc_step",    0.5);
    v_min_       = declare_parameter("min_speed",   0.5);
    v_max_       = declare_parameter("max_speed",  20.0);
    publish_debug_= declare_parameter("publish_debug_marker", false);

    /* Pub/Sub */
    sub_wp_ = create_subscription<MarkerArray>(
        "/final_waypoints", 10,
        std::bind(&SpeedPlanner::cbWaypoints, this, std::placeholders::_1));

    pub_profile_ = create_publisher<Float32MultiArray>(
        "/desired_speed_profile", 10);

    if (publish_debug_) {
      pub_debug_ = create_publisher<MarkerArray>("/planned_waypoints", 10);
      dbg_timer_ = create_wall_timer(std::chrono::milliseconds(100),   // 10 Hz
                    std::bind(&SpeedPlanner::publishDebug, this));
    }

    RCLCPP_INFO(get_logger(), "SpeedPlanner ready");
  }

private:
  /* Waypoint 수신 → 속도 프로파일 계산 */
  void cbWaypoints(const MarkerArray::SharedPtr msg)
  {
    pts_.clear();
    for (const auto &mk : msg->markers)
      if (mk.type == Marker::SPHERE)
        pts_.emplace_back(mk.pose.position.x, mk.pose.position.y);

    const size_t N = pts_.size();
    if (N < 3) return;

    /* 1) 곡률 */
    std::vector<double> k(N, 0.0);
    for (size_t i = 1; i + 1 < N; ++i)
      k[i] = curvature(pts_[i-1], pts_[i], pts_[i+1]);

    /* 2) look-ahead 평균 */
    std::vector<double> k_avg(N, 0.0);
    for (size_t i = 0; i < N; ++i) {
      double sum = 0; int cnt = 0;
      for (size_t j = i; j < std::min(N, i+static_cast<size_t>(lookahead_n_)); ++j)
        { sum += k[j]; ++cnt; }
      k_avg[i] = (cnt ? sum/cnt : 0.0);
    }

    /* 3) 제한 속도 v */
    v_des_.assign(N, v_max_);
    const double eps = 1e-6;
    for (size_t i = 0; i < N; ++i)
      if (k_avg[i] > eps) v_des_[i] = std::sqrt(a_lat_max_/k_avg[i]);

    for (size_t i = 1; i < N; ++i)   /* 앞→뒤 가속 */
      v_des_[i] = std::min(v_des_[i],
                 std::sqrt(v_des_[i-1]*v_des_[i-1] + 2*a_long_max_*arc_step_));
    for (int i = static_cast<int>(N)-2; i >= 0; --i) /* 뒤→앞 감속 */
      v_des_[i] = std::min(v_des_[i],
                 std::sqrt(v_des_[i+1]*v_des_[i+1] + 2*a_long_max_*arc_step_));

    for (double &v : v_des_) v = std::clamp(v, v_min_, v_max_);

    /* 4) 퍼블리시 */
    Float32MultiArray arr;
    arr.data.reserve(N);
    for (double v : v_des_) arr.data.push_back(static_cast<float>(v));
    pub_profile_->publish(arr);
  }

  /* 디버그 MarkerArray (10 Hz 타이머) */
  void publishDebug()
  {
    if (!publish_debug_ || pts_.empty()) return;

    rclcpp::Time stamp = get_clock()->now();
    MarkerArray arr;

    for (size_t i = 0; i < pts_.size(); ++i) {
      Marker m;
      m.header.stamp = stamp; m.header.frame_id = "os_sensor";
      m.pose.orientation.w = 1.0;
      m.id = static_cast<int>(i);

      /* Waypoint 구체 */
      m.ns = "planned_wp"; m.type = Marker::SPHERE; m.action = Marker::ADD;
      m.pose.position.x = pts_[i].x(); m.pose.position.y = pts_[i].y();
      m.scale.x = m.scale.y = m.scale.z = 0.15;
      m.color.set__r(0.0f).set__g(0.4f).set__b(1.0f).set__a(1.0f);
      arr.markers.push_back(m);

      /* 속도 막대 */
      m.ns = "planned_speed"; m.type = Marker::CUBE;
      m.scale.x = m.scale.y = 0.2; m.scale.z = std::max(v_des_[i], 0.01);
      m.pose.position.z = m.scale.z * 0.5;
      m.color.set__r(1.0f).set__g(0.6f).set__b(0.0f).set__a(0.8f);
      arr.markers.push_back(m);
    }

    /* 잔상 제거용 DELETE */
    for (int id = static_cast<int>(pts_.size());
         id < prev_dbg_n_; ++id)
    {
      Marker del;
      del.header = arr.markers[0].header;
      del.action = Marker::DELETE;
      del.ns = "planned_wp";     del.id = id; arr.markers.push_back(del);
      del.ns = "planned_speed";  del.id = id; arr.markers.push_back(del);
    }
    prev_dbg_n_ = static_cast<int>(pts_.size());

    pub_debug_->publish(arr);
  }

  /* 멤버 */
  rclcpp::Subscription<MarkerArray>::SharedPtr sub_wp_;
  rclcpp::Publisher<Float32MultiArray>::SharedPtr pub_profile_;
  rclcpp::Publisher<MarkerArray>::SharedPtr pub_debug_;
  rclcpp::TimerBase::SharedPtr dbg_timer_;

  std::vector<Vector2d> pts_;
  std::vector<double>   v_des_;
  int prev_dbg_n_{0};

  /* 파라미터 */
  double a_lat_max_, a_long_max_, arc_step_;
  int    lookahead_n_;
  double v_min_, v_max_;
  bool   publish_debug_;
};

/* ─── main ─── */
int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SpeedPlanner>());
  rclcpp::shutdown();
  return 0;
}
