#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32.hpp>
#include <nav_msgs/msg/path.hpp>
#include <visualization_msgs/msg/marker.hpp>

#include <vector>
#include <cmath>
#include <algorithm>
#include <string>

class PurePursuitStatic : public rclcpp::Node {
public:
  PurePursuitStatic() : rclcpp::Node("pure_pursuit_static") {
    // Parameters
    path_topic_    = declare_parameter<std::string>("path_topic", "/local_planned_path");
    steer_topic_   = declare_parameter<std::string>("steer_topic", "/cmd/steer");
    marker_topic_  = declare_parameter<std::string>("lookahead_marker_topic", "/lookahead_point_marker");

    wheelbase_m_   = declare_parameter<double>("wheelbase_m", 1.295);
    lookahead_m_   = declare_parameter<double>("lookahead_m", 2.5);

    publish_rate_hz_     = declare_parameter<double>("publish_rate_hz", 50.0);
    steer_limit_deg_     = declare_parameter<double>("steer_limit_deg", 30.0); // deg
    use_x_forward_only_  = declare_parameter<bool>("use_x_forward_only", true);

    marker_scale_ = declare_parameter<double>("marker_scale", 0.3);
    marker_alpha_ = declare_parameter<double>("marker_alpha", 1.0);
    marker_r_     = declare_parameter<double>("marker_r", 0.0);
    marker_g_     = declare_parameter<double>("marker_g", 1.0);
    marker_b_     = declare_parameter<double>("marker_b", 0.8);

    // Pub/Sub
    sub_path_ = create_subscription<nav_msgs::msg::Path>(
      path_topic_, rclcpp::QoS(10),
      std::bind(&PurePursuitStatic::onPath, this, std::placeholders::_1));

    pub_steer_  = create_publisher<std_msgs::msg::Float32>(steer_topic_, rclcpp::QoS(10));
    pub_marker_ = create_publisher<visualization_msgs::msg::Marker>(marker_topic_, rclcpp::QoS(10));

    // Timer
    using namespace std::chrono_literals;
    const auto period = std::chrono::duration<double>(1.0 / std::max(1e-3, publish_rate_hz_));
    timer_ = create_wall_timer(std::chrono::duration_cast<std::chrono::milliseconds>(period),
              std::bind(&PurePursuitStatic::onTimer, this));

    RCLCPP_INFO(get_logger(),
      "PurePursuitStatic started. path='%s' -> steer(deg)='%s', LAD=%.2f m, L=%.3f m, limit=±%.1f deg",
      path_topic_.c_str(), steer_topic_.c_str(), lookahead_m_, wheelbase_m_, steer_limit_deg_);
  }

private:
  struct Pt { double x, y; };
  static constexpr double kPi = 3.14159265358979323846;

  void onPath(const nav_msgs::msg::Path::SharedPtr msg) {
    last_path_ = *msg;
    pts_.clear();
    pts_.reserve(last_path_.poses.size());
    for (const auto& ps : last_path_.poses) {
      pts_.push_back({static_cast<double>(ps.pose.position.x),
                      static_cast<double>(ps.pose.position.y)});
    }
    frame_id_ = last_path_.header.frame_id.empty() ? "base_link" : last_path_.header.frame_id;
  }

  void onTimer() {
    if (pts_.empty()) {
      publishSteerDeg(0.0);
      return;
    }

    // 1) Look-ahead 포인트 선택
    const int idx = selectLookaheadIndex();
    if (idx < 0) {
      publishSteerDeg(0.0);
      return;
    }
    const Pt target = pts_[static_cast<size_t>(idx)];

    // 2) Pure Pursuit 조향 계산 (내부 계산은 rad, 결과는 deg)
    // 좌회전이 + 가 되도록: base_link에서 y>0이면 +조향이 나옴(ROS REP-103과 일치)
    const double Ld2 = std::max(1e-9, target.x * target.x + target.y * target.y);
    const double delta_rad = std::atan2(2.0 * wheelbase_m_ * target.y, Ld2);
    double delta_deg = delta_rad * 180.0 / kPi;

    // 3) 한계 클램프 (deg)
    delta_deg = std::clamp(delta_deg, -steer_limit_deg_, steer_limit_deg_);

    // 4) Publish
    publishSteerDeg(delta_deg);
    publishMarker(target);
  }

  int selectLookaheadIndex() const {
    if (pts_.empty()) return -1;
    const bool forward_only = use_x_forward_only_;

    // 첫 r >= Ld (전방만 쓸 경우 x>0) 찾기
    for (size_t i = 0; i < pts_.size(); ++i) {
      const auto &p = pts_[i];
      if (forward_only && p.x <= 0.0) continue;
      if (std::hypot(p.x, p.y) >= lookahead_m_) {
        return static_cast<int>(i);
      }
    }
    // 없으면 끝점(가능하면 전방 x>0 우선)
    for (int i = static_cast<int>(pts_.size()) - 1; i >= 0; --i) {
      if (!forward_only || pts_[static_cast<size_t>(i)].x > 0.0) return i;
    }
    return static_cast<int>(pts_.size()) - 1;
  }

  void publishSteerDeg(double steer_deg) {
    std_msgs::msg::Float32 out;
    out.data = static_cast<float>(steer_deg); // deg
    pub_steer_->publish(out);
  }

  void publishMarker(const Pt& p) {
    visualization_msgs::msg::Marker m;
    m.header.stamp = now();
    m.header.frame_id = frame_id_.empty() ? "base_link" : frame_id_;
    m.ns = "lookahead_point";
    m.id = 0;
    m.type = visualization_msgs::msg::Marker::SPHERE;
    m.action = visualization_msgs::msg::Marker::ADD;

    m.pose.position.x = p.x;
    m.pose.position.y = p.y;
    m.pose.position.z = 0.0;
    m.pose.orientation.w = 1.0;

    m.scale.x = marker_scale_;
    m.scale.y = marker_scale_;
    m.scale.z = marker_scale_;

    m.color.a = marker_alpha_;
    m.color.r = marker_r_;
    m.color.g = marker_g_; // mint
    m.color.b = marker_b_;

    m.lifetime = rclcpp::Duration(0, 0); // 영구
    pub_marker_->publish(m);
  }

  // Params
  std::string path_topic_, steer_topic_, marker_topic_;
  double wheelbase_m_{1.295};
  double lookahead_m_{2.5};
  double publish_rate_hz_{50.0};
  double steer_limit_deg_{30.0};
  bool   use_x_forward_only_{true};

  double marker_scale_{0.3}, marker_alpha_{1.0};
  double marker_r_{0.0}, marker_g_{1.0}, marker_b_{0.8};

  // State
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr sub_path_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr pub_steer_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr pub_marker_;
  rclcpp::TimerBase::SharedPtr timer_;

  nav_msgs::msg::Path last_path_;
  std::vector<Pt> pts_;
  std::string frame_id_{"base_link"};
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PurePursuitStatic>());
  rclcpp::shutdown();
  return 0;
}
