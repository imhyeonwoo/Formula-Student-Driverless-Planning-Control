#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32.hpp>
#include <nav_msgs/msg/path.hpp>
#include <visualization_msgs/msg/marker.hpp>

#include <vector>
#include <cmath>
#include <algorithm>
#include <string>
#include <chrono>

class PurePursuitDynamic : public rclcpp::Node {
public:
  PurePursuitDynamic() : rclcpp::Node("pure_pursuit_dynamic") {
    // Parameters
    path_topic_    = declare_parameter<std::string>("path_topic", "/local_planned_path");
    speed_topic_   = declare_parameter<std::string>("speed_topic", "/current_speed");
    steer_topic_   = declare_parameter<std::string>("steer_topic", "/cmd/steer");
    marker_topic_  = declare_parameter<std::string>("lookahead_marker_topic", "/lookahead_point_marker");

    wheelbase_m_   = declare_parameter<double>("wheelbase_m", 1.295);

    L0_            = declare_parameter<double>("L0", 1.5);
    k_v_           = declare_parameter<double>("k_v", 0.6);
    Ld_min_        = declare_parameter<double>("Ld_min", 1.0);
    Ld_max_        = declare_parameter<double>("Ld_max", 5.0);

    use_curv_term_ = declare_parameter<bool>("use_curvature_term", false);
    k_k_           = declare_parameter<double>("k_k", 0.0);
    eps_kappa_     = declare_parameter<double>("epsilon_kappa", 1e-6);
    curv_window_m_ = declare_parameter<double>("curv_window_m", 2.0);

    publish_rate_hz_     = declare_parameter<double>("publish_rate_hz", 50.0);
    steer_limit_deg_     = declare_parameter<double>("steer_limit_deg", 30.0);
    use_x_forward_only_  = declare_parameter<bool>("use_x_forward_only", true);

    ema_tau_speed_ = declare_parameter<double>("ema_tau_speed", 0.2);
    ema_tau_cmd_   = declare_parameter<double>("ema_tau_cmd", 0.1);

    marker_scale_ = declare_parameter<double>("marker_scale", 0.3);
    marker_alpha_ = declare_parameter<double>("marker_alpha", 1.0);
    marker_r_     = declare_parameter<double>("marker_r", 0.0);
    marker_g_     = declare_parameter<double>("marker_g", 1.0);
    marker_b_     = declare_parameter<double>("marker_b", 0.8);

    // Sub/Pub
    sub_path_ = create_subscription<nav_msgs::msg::Path>(
      path_topic_, rclcpp::QoS(10),
      std::bind(&PurePursuitDynamic::onPath, this, std::placeholders::_1));

    sub_speed_ = create_subscription<std_msgs::msg::Float32>(
      speed_topic_, rclcpp::SensorDataQoS(),
      std::bind(&PurePursuitDynamic::onSpeed, this, std::placeholders::_1));

    pub_steer_  = create_publisher<std_msgs::msg::Float32>(steer_topic_, rclcpp::QoS(10));
    pub_marker_ = create_publisher<visualization_msgs::msg::Marker>(marker_topic_, rclcpp::QoS(10));

    // Timer
    using namespace std::chrono_literals;
    const auto period = std::chrono::duration<double>(1.0 / std::max(1e-3, publish_rate_hz_));
    timer_ = create_wall_timer(std::chrono::duration_cast<std::chrono::milliseconds>(period),
              std::bind(&PurePursuitDynamic::onTimer, this));

    RCLCPP_INFO(get_logger(),
      "PurePursuitDynamic started. path='%s', speed='%s' -> steer(deg)='%s' | Ld = L0(%.2f) + k_v(%.2f)*v [+ curv]",
      path_topic_.c_str(), speed_topic_.c_str(), steer_topic_.c_str(), L0_, k_v_);
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

    // cumulative distance for curvature window search
    cum_s_.assign(pts_.size(), 0.0);
    for (size_t i = 1; i < pts_.size(); ++i) {
      double ds = std::hypot(pts_[i].x - pts_[i-1].x, pts_[i].y - pts_[i-1].y);
      cum_s_[i] = cum_s_[i-1] + ds;
    }
  }

  void onSpeed(const std_msgs::msg::Float32::SharedPtr m) {
    const rclcpp::Time now = now_();
    const double meas = static_cast<double>(m->data);
    if (!have_speed_) {
      v_filt_ = meas;
      have_speed_ = true;
    } else {
      const double dt = (now - last_speed_time_).seconds();
      const double tau = std::max(1e-3, ema_tau_speed_);
      const double alpha = std::exp(-std::max(0.0, dt) / tau);
      v_filt_ = alpha * v_filt_ + (1.0 - alpha) * meas;
    }
    last_speed_time_ = now;
  }

  void onTimer() {
    if (pts_.empty()) {
      publishSteerDeg(smoothDeg(0.0));
      return;
    }

    // 1) 동적 Ld 계산
    double Ld_raw = L0_ + k_v_ * std::max(0.0, v_filt_);
    if (use_curv_term_ && pts_.size() >= 3 && k_k_ != 0.0) {
      const double kappa = evalCurvatureAt(curv_window_m_);
      if (std::isfinite(kappa)) {
        Ld_raw += k_k_ / (std::abs(kappa) + eps_kappa_);
      }
    }
    const double Ld = std::clamp(Ld_raw, Ld_min_, Ld_max_);

    // 2) Look-ahead 포인트 선택 (||p|| >= Ld, 전방만 옵션)
    const int idx = selectLookaheadIndex(Ld);
    if (idx < 0) {
      publishSteerDeg(smoothDeg(0.0));
      return;
    }
    const Pt target = pts_[static_cast<size_t>(idx)];

    // 3) Pure Pursuit 조향 (내부 rad → 출력 deg). 좌회전 +
    const double Ld2 = std::max(1e-9, target.x * target.x + target.y * target.y);
    const double delta_rad = std::atan2(2.0 * wheelbase_m_ * target.y, Ld2);
    double delta_deg = delta_rad * 180.0 / kPi;

    // 4) 한계 + 평활
    delta_deg = std::clamp(delta_deg, -steer_limit_deg_, steer_limit_deg_);
    const double cmd_deg = smoothDeg(delta_deg);

    // 5) Publish
    publishSteerDeg(cmd_deg);
    publishMarker(target);
  }

  int selectLookaheadIndex(double Ld) const {
    if (pts_.empty()) return -1;
    const bool forward_only = use_x_forward_only_;

    for (size_t i = 0; i < pts_.size(); ++i) {
      const auto &p = pts_[i];
      if (forward_only && p.x <= 0.0) continue;
      if (std::hypot(p.x, p.y) >= Ld) return static_cast<int>(i);
    }
    // 없으면 끝점(가능하면 전방 x>0 우선)
    for (int i = static_cast<int>(pts_.size()) - 1; i >= 0; --i) {
      if (!forward_only || pts_[static_cast<size_t>(i)].x > 0.0) return i;
    }
    return static_cast<int>(pts_.size()) - 1;
  }

  double evalCurvatureAt(double s_eval) const {
    if (pts_.size() < 3) return 0.0;
    // s_eval과 가장 가까운 index
    size_t j = 0;
    if (!cum_s_.empty()) {
      auto it = std::lower_bound(cum_s_.begin(), cum_s_.end(), s_eval);
      j = (it == cum_s_.end()) ? (cum_s_.size() - 1) : static_cast<size_t>(it - cum_s_.begin());
    }
    if (j == 0) j = 1;
    if (j + 1 >= pts_.size()) j = pts_.size() - 2;

    const auto &A = pts_[j-1];
    const auto &B = pts_[j];
    const auto &C = pts_[j+1];
    const double ax = B.x - A.x, ay = B.y - A.y;
    const double bx = C.x - B.x, by = C.y - B.y;
    const double cx = C.x - A.x, cy = C.y - A.y;
    const double a = std::hypot(ax, ay);
    const double b = std::hypot(bx, by);
    const double c = std::hypot(cx, cy);
    const double area2 = std::abs(ax * cy - ay * cx); // 2*Area
    const double denom = std::max(1e-12, a * b * c);
    return area2 / denom; // |κ|
  }

  double smoothDeg(double raw_deg) {
    const rclcpp::Time now = now_();
    const double dt = (last_cmd_time_.nanoseconds() == 0)
                      ? (1.0 / std::max(1e-3, publish_rate_hz_))
                      : (now - last_cmd_time_).seconds();
    last_cmd_time_ = now;

    const double tau = std::max(1e-3, ema_tau_cmd_);
    const double alpha = std::exp(-std::max(0.0, dt) / tau);
    cmd_deg_prev_ = alpha * cmd_deg_prev_ + (1.0 - alpha) * raw_deg;
    // limit 최종 보장
    cmd_deg_prev_ = std::clamp(cmd_deg_prev_, -steer_limit_deg_, steer_limit_deg_);
    return cmd_deg_prev_;
  }

  void publishSteerDeg(double steer_deg) {
    std_msgs::msg::Float32 out;
    out.data = static_cast<float>(steer_deg);
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

    m.lifetime = rclcpp::Duration(0, 0); // 영구
    pub_marker_->publish(m);
  }

  rclcpp::Time now_() { return this->get_clock()->now(); }

  // Params
  std::string path_topic_, speed_topic_, steer_topic_, marker_topic_;
  double wheelbase_m_{1.295};

  double L0_{1.5}, k_v_{0.6}, Ld_min_{1.0}, Ld_max_{5.0};
  bool   use_curv_term_{false};
  double k_k_{0.0}, eps_kappa_{1e-6}, curv_window_m_{2.0};

  double publish_rate_hz_{50.0};
  double steer_limit_deg_{30.0};
  bool   use_x_forward_only_{true};

  double ema_tau_speed_{0.2}, ema_tau_cmd_{0.1};

  double marker_scale_{0.3}, marker_alpha_{1.0};
  double marker_r_{0.0}, marker_g_{1.0}, marker_b_{0.8};

  // State
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr sub_path_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr sub_speed_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr pub_steer_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr pub_marker_;
  rclcpp::TimerBase::SharedPtr timer_;

  nav_msgs::msg::Path last_path_;
  std::vector<Pt> pts_;
  std::vector<double> cum_s_;
  std::string frame_id_{"base_link"};

  bool have_speed_{false};
  double v_filt_{0.0};
  rclcpp::Time last_speed_time_{};

  rclcpp::Time last_cmd_time_{};
  double cmd_deg_prev_{0.0};
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PurePursuitDynamic>());
  rclcpp::shutdown();
  return 0;
}
