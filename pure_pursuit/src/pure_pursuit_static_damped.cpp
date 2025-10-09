#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32.hpp>
#include <nav_msgs/msg/path.hpp>
#include <visualization_msgs/msg/marker.hpp>

#include <vector>
#include <cmath>
#include <algorithm>
#include <string>

class PurePursuitDamped : public rclcpp::Node {
public:
  PurePursuitDamped() : rclcpp::Node("pure_pursuit_static_damped") {
    // Parameters
    path_topic_    = declare_parameter<std::string>("path_topic", "/local_planned_path");
    steer_topic_   = declare_parameter<std::string>("steer_topic", "/cmd/steer");
    marker_topic_  = declare_parameter<std::string>("lookahead_marker_topic", "/lookahead_point_marker_damped");

    wheelbase_m_   = declare_parameter<double>("wheelbase_m", 1.3);
    lookahead_m_   = declare_parameter<double>("lookahead_m", 11.0);

    publish_rate_hz_     = declare_parameter<double>("publish_rate_hz", 50.0);
    steer_limit_deg_     = declare_parameter<double>("steer_limit_deg", 30.0); // deg
    use_x_forward_only_  = declare_parameter<bool>("use_x_forward_only", true);

    // D-term (heading error rate) parameters
    enable_d_term_       = declare_parameter<bool>("enable_d_term", true);
    kd_heading_rate_     = declare_parameter<double>("kd_heading_rate", 0.15); // [s], small to start
    deriv_lpf_tau_       = declare_parameter<double>("deriv_lpf_tau", 0.20);   // [s], LPF time constant

    // Output sign convention (keep compatibility with user's current system)
    // true: publishSteerDeg() multiplies by -1 (right:+, left:-)
    // false: publish as-is (left:+, right:-)
    invert_steer_sign_   = declare_parameter<bool>("invert_steer_sign", true);

    // Marker appearance
    marker_scale_ = declare_parameter<double>("marker_scale", 0.3);
    marker_alpha_ = declare_parameter<double>("marker_alpha", 1.0);
    marker_r_     = declare_parameter<double>("marker_r", 0.0);
    marker_g_     = declare_parameter<double>("marker_g", 1.0);
    marker_b_     = declare_parameter<double>("marker_b", 0.8);

    // Pub/Sub
    sub_path_ = create_subscription<nav_msgs::msg::Path>(
      path_topic_, rclcpp::QoS(10),
      std::bind(&PurePursuitDamped::onPath, this, std::placeholders::_1));

    pub_steer_  = create_publisher<std_msgs::msg::Float32>(steer_topic_, rclcpp::QoS(10));
    pub_marker_ = create_publisher<visualization_msgs::msg::Marker>(marker_topic_, rclcpp::QoS(10));

    // Timer
    using namespace std::chrono_literals;
    const auto period = std::chrono::duration<double>(1.0 / std::max(1e-3, publish_rate_hz_));
    timer_ = create_wall_timer(std::chrono::duration_cast<std::chrono::milliseconds>(period),
              std::bind(&PurePursuitDamped::onTimer, this));

    RCLCPP_INFO(get_logger(),
      "PurePursuitDamped started. path='%s' -> steer(deg)='%s', LAD=%.2f m, L=%.3f m, limit=±%.1f deg, D:%s kd=%.3f, tau=%.3f s",
      path_topic_.c_str(), steer_topic_.c_str(), lookahead_m_, wheelbase_m_, steer_limit_deg_,
      enable_d_term_ ? "on" : "off", kd_heading_rate_, deriv_lpf_tau_);
  }

private:
  struct Pt { double x, y; };
  static constexpr double kPi = 3.14159265358979323846;

  // --- Angle utilities ---
  static double wrapPi(double a) {
    while (a >  M_PI) a -= 2.0*M_PI;
    while (a < -M_PI) a += 2.0*M_PI;
    return a;
  }
  static double angDiff(double a, double b) { return wrapPi(a - b); }

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

    // 1) Look-ahead point selection
    const int idx = selectLookaheadIndex();
    if (idx < 0) { publishSteerDeg(0.0); return; }
    const Pt target = pts_[static_cast<size_t>(idx)];

    // 2) Pure Pursuit steering (internal rad)
    const double Ld2 = std::max(1e-9, target.x * target.x + target.y * target.y);
    const double delta_pp_rad = std::atan2(2.0 * wheelbase_m_ * target.y, Ld2);

    // 3) Heading error alpha (base_link 기준, target bearing)
    const double alpha_rad = std::atan2(target.y, target.x);

    // 4) Derivative of heading error with 1st-order LPF
    const auto now_t = now();
    double delta_rad = delta_pp_rad; // initialize with PP term

    if (enable_d_term_) {
      const double dt = [&](){
        if (!has_last_) return 0.0;
        const double s = (now_t - last_time_).seconds();
        return std::max(1e-3, s);
      }();

      if (has_last_) {
        const double d_alpha = angDiff(alpha_rad, last_alpha_rad_) / dt; // [rad/s]
        const double tau = std::max(1e-3, deriv_lpf_tau_);
        alpha_dot_filt_ += (dt / tau) * (d_alpha - alpha_dot_filt_);    // LPF
        delta_rad += kd_heading_rate_ * alpha_dot_filt_;                // add D term (rad)
      } else {
        alpha_dot_filt_ = 0.0;
        has_last_ = true;
      }

      last_time_ = now_t;
      last_alpha_rad_ = alpha_rad;
    }

    // 5) Clamp and publish (deg)
    double delta_deg = delta_rad * 180.0 / kPi;
    delta_deg = std::clamp(delta_deg, -steer_limit_deg_, steer_limit_deg_);

    publishSteerDeg(delta_deg);
    publishMarker(target);
  }

  int selectLookaheadIndex() const {
    if (pts_.empty()) return -1;
    const bool forward_only = use_x_forward_only_;

    // First point with r >= Ld (and x>0 if forward-only)
    for (size_t i = 0; i < pts_.size(); ++i) {
      const auto &p = pts_[i];
      if (forward_only && p.x <= 0.0) continue;
      if (std::hypot(p.x, p.y) >= lookahead_m_) {
        return static_cast<int>(i);
      }
    }
    // Otherwise fallback to last valid (prefer x>0 if forward-only)
    for (int i = static_cast<int>(pts_.size()) - 1; i >= 0; --i) {
      if (!forward_only || pts_[static_cast<size_t>(i)].x > 0.0) return i;
    }
    return static_cast<int>(pts_.size()) - 1;
  }

  void publishSteerDeg(double steer_deg) {
    std_msgs::msg::Float32 out;
    out.data = static_cast<float>(invert_steer_sign_ ? -steer_deg : steer_deg);
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
    m.color.g = marker_g_;
    m.color.b = marker_b_;

    m.lifetime = rclcpp::Duration(0, 0);
    pub_marker_->publish(m);
  }

  // Params
  std::string path_topic_, steer_topic_, marker_topic_;
  double wheelbase_m_{1.295};
  double lookahead_m_{2.5};
  double publish_rate_hz_{50.0};
  double steer_limit_deg_{30.0};
  bool   use_x_forward_only_{true};

  bool   enable_d_term_{true};
  double kd_heading_rate_{0.15};
  double deriv_lpf_tau_{0.20};
  bool   invert_steer_sign_{true};

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

  // D-term state
  double last_alpha_rad_{0.0};
  double alpha_dot_filt_{0.0};
  rclcpp::Time last_time_{0,0, get_clock()->get_clock_type()};
  bool   has_last_{false};
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PurePursuitDamped>());
  rclcpp::shutdown();
  return 0;
}
