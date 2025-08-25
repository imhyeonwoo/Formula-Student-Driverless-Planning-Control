#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <nav_msgs/msg/path.hpp>

#include <vector>
#include <cmath>
#include <algorithm>
#include <chrono>

class SimpleSpeedPlanner : public rclcpp::Node {
public:
  SimpleSpeedPlanner() : rclcpp::Node("simple_speed_planner") {
    // Parameters
    path_topic_ = declare_parameter<std::string>("path_topic", "/local_planned_path");
    speed_topic_ = declare_parameter<std::string>("speed_topic", "/current_speed");
    out_topic_ = declare_parameter<std::string>("out_topic", "/desired_speed_profile");
    desired_speed_topic_ = declare_parameter<std::string>("desired_speed_topic", "/desired_speed");
    desired_rpm_topic_ = declare_parameter<std::string>("desired_rpm_topic", "/desired_rpm");

    recalc_rate_hz_ = declare_parameter<double>("recalc_rate_hz", 15.0);
    desired_pub_rate_hz_ = declare_parameter<double>("desired_pub_rate_hz", 50.0);
    horizon_m_ = declare_parameter<double>("horizon_m", -1.0);

    v_max_ = declare_parameter<double>("v_max", 4.0);
    v_min_ = declare_parameter<double>("v_min", 0.3);
    v_end_ = declare_parameter<double>("v_end", 0.0);

    a_acc_ = declare_parameter<double>("a_acc", 2.0);
    a_brk_ = declare_parameter<double>("a_brk", 2.5);
    ay_max_ = declare_parameter<double>("ay_max", 1.5);

    kappa_ma_window_ = declare_parameter<int>("kappa_ma_window", 5);
    epsilon_kappa_ = declare_parameter<double>("epsilon_kappa", 1e-6);

    ema_tau_speed_ = declare_parameter<double>("ema_tau_speed", 0.2);

    preview_t_ = declare_parameter<double>("preview_t", 0.4);
    preview_s_min_ = declare_parameter<double>("preview_s_min", 0.5);
    preview_s_max_ = declare_parameter<double>("preview_s_max", 5.0);
    cmd_acc_limit_ = declare_parameter<double>("cmd_acc_limit", 1.5);
    ema_tau_cmd_ = declare_parameter<double>("ema_tau_cmd", 0.2);

    log_throttle_sec_ = declare_parameter<double>("log_throttle_sec", 2.0);

    // Pub/Sub
    pub_profile_ = create_publisher<std_msgs::msg::Float32MultiArray>(out_topic_, 10);
    pub_desired_ = create_publisher<std_msgs::msg::Float32>(desired_speed_topic_, 10);

    sub_path_ = create_subscription<nav_msgs::msg::Path>(
        path_topic_, rclcpp::QoS(10),
        std::bind(&SimpleSpeedPlanner::onPath, this, std::placeholders::_1));

    sub_speed_ = create_subscription<std_msgs::msg::Float32>(
        speed_topic_, rclcpp::SensorDataQoS(),
        std::bind(&SimpleSpeedPlanner::onSpeed, this, std::placeholders::_1));

    // Timers
    using namespace std::chrono;
    const auto t_recalc = duration<double>(1.0 / std::max(1e-3, recalc_rate_hz_));
    timer_recalc_ = create_wall_timer(duration_cast<milliseconds>(t_recalc),
                               std::bind(&SimpleSpeedPlanner::onRecalcTimer, this));

    const auto t_cmd = duration<double>(1.0 / std::max(1e-3, desired_pub_rate_hz_));
    timer_cmd_ = create_wall_timer(duration_cast<milliseconds>(t_cmd),
                               std::bind(&SimpleSpeedPlanner::onCmdTimer, this));

    RCLCPP_INFO(get_logger(), "SimpleSpeedPlanner started. path='%s' speed='%s' → out='%s', desired='%s'",
                path_topic_.c_str(), speed_topic_.c_str(), out_topic_.c_str(), desired_speed_topic_.c_str());
  }

private:
  struct Pt { double x, y; };

  void onSpeed(const std_msgs::msg::Float32::SharedPtr msg) {
    const rclcpp::Time now = now_();
    double meas = static_cast<double>(msg->data);
    if (!have_speed_) {
      v_filt_ = meas;
      have_speed_ = true;
    } else {
      const double dt = (now - last_speed_time_).seconds();
      const double tau = std::max(1e-3, ema_tau_speed_);
      const double alpha = std::exp(-std::max(0.0, dt) / tau); // EMA
      v_filt_ = alpha * v_filt_ + (1.0 - alpha) * meas;
    }
    last_speed_time_ = now;
  }

  void onPath(const nav_msgs::msg::Path::SharedPtr msg) {
    last_path_ = *msg;
    have_path_ = !last_path_.poses.empty();
    last_path_stamp_ = last_path_.header.stamp;
  }

  void onRecalcTimer() {
    if (!have_path_) return;

    // Extract points
    std::vector<Pt> pts; pts.reserve(last_path_.poses.size());
    for (const auto &ps : last_path_.poses) {
      pts.push_back({ps.pose.position.x, ps.pose.position.y});
    }
    size_t N = pts.size();
    if (N == 0) return;

    // If horizon is set (>0), optionally crop by distance
    if (horizon_m_ > 0.0) {
      std::vector<Pt> cropped; cropped.reserve(N);
      cropped.push_back(pts.front());
      double acc = 0.0;
      for (size_t i = 1; i < N; ++i) {
        double ds = hypot(pts[i].x - pts[i-1].x, pts[i].y - pts[i-1].y);
        acc += ds;
        cropped.push_back(pts[i]);
        if (acc >= horizon_m_) break;
      }
      pts.swap(cropped);
      N = pts.size();
    }

    if (N == 1) {
      // Single point edge case → keep desired speed close to current, publish profile of length 1
      std_msgs::msg::Float32MultiArray out;
      out.data.resize(1);
      out.data[0] = static_cast<float>(clip(v_filt_, v_min_, v_max_));
      pub_profile_->publish(out);
      last_profile_.assign(1, clip(v_filt_, v_min_, v_max_));
      cum_s_.assign(1, 0.0);
      return;
    }

    // Segment distances (N-1)
    std::vector<double> ds(N-1, 0.0);
    for (size_t i = 1; i < N; ++i) {
      ds[i-1] = std::max(1e-6, hypot(pts[i].x - pts[i-1].x, pts[i].y - pts[i-1].y));
    }

    // Cumulative distances (N)
    cum_s_.assign(N, 0.0);
    for (size_t i = 1; i < N; ++i) cum_s_[i] = cum_s_[i-1] + ds[i-1];

    // Curvature per point (N). Using 3-point circle curvature, abs value.
    std::vector<double> kappa(N, 0.0);
    if (N >= 3) {
      for (size_t i = 1; i + 1 < N; ++i) {
        const auto &A = pts[i-1];
        const auto &B = pts[i];
        const auto &C = pts[i+1];
        const double ax = B.x - A.x, ay = B.y - A.y;
        const double bx = C.x - B.x, by = C.y - B.y;
        const double cx = C.x - A.x, cy = C.y - A.y;
        const double a = std::hypot(ax, ay);
        const double b = std::hypot(bx, by);
        const double c = std::hypot(cx, cy);
        const double area2 = std::abs(ax * cy - ay * cx); // 2*Area of triangle ABC
        const double denom = std::max(epsilon_kappa_, a * b * c);
        kappa[i] = area2 / denom; // >= 0
      }
      // Boundary copy
      kappa[0] = kappa[1];
      kappa[N-1] = kappa[N-2];

      // Moving average smoothing
      if (kappa_ma_window_ > 1) {
        const int w = kappa_ma_window_;
        std::vector<double> sm(N, 0.0);
        int half = w / 2;
        for (size_t i = 0; i < N; ++i) {
          int L = std::max<int>(0, static_cast<int>(i) - half);
          int R = std::min<int>(static_cast<int>(N)-1, static_cast<int>(i) + half);
          double sum = 0.0;
          for (int j = L; j <= R; ++j) sum += kappa[j];
          sm[i] = sum / std::max(1, R - L + 1);
        }
        kappa.swap(sm);
      }
    }

    // Curvature-limited speed
    std::vector<double> vlim(N, v_max_);
    for (size_t i = 0; i < N; ++i) {
      const double vmax_by_kappa = std::sqrt(ay_max_ / (std::abs(kappa[i]) + epsilon_kappa_));
      vlim[i] = std::min(v_max_, vmax_by_kappa);
    }

    // Forward pass (accel limit)
    std::vector<double> v(N, 0.0);
    const double v0 = clip(have_speed_ ? v_filt_ : 0.0, v_min_, vlim[0]);
    v[0] = v0;
    for (size_t i = 1; i < N; ++i) {
      const double v_acc = std::sqrt(std::max(0.0, v[i-1]*v[i-1] + 2.0 * a_acc_ * ds[i-1]));
      v[i] = std::min(vlim[i], v_acc);
    }

    // Enforce terminal desired speed and Backward pass (brake limit)
    v[N-1] = std::min(v[N-1], std::max(v_min_, v_end_));
    for (size_t i = N - 1; i-- > 0; ) {
      const double v_brk = std::sqrt(std::max(0.0, v[i+1]*v[i+1] + 2.0 * a_brk_ * ds[i]));
      v[i] = std::min(v[i], v_brk);
    }

    // Store for command timer
    last_profile_ = v;

    // Publish profile (Float32MultiArray, length = N)
    std_msgs::msg::Float32MultiArray out;
    out.data.resize(N);
    for (size_t i = 0; i < N; ++i) out.data[i] = static_cast<float>(clip(v[i], v_min_, v_max_));
    pub_profile_->publish(out);

    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), static_cast<uint64_t>(log_throttle_sec_ * 1000),
                         "Published speed profile: N=%zu, v0=%.2f, v_end=%.2f", N, v.front(), v.back());
  }

  void onCmdTimer() {
    if (last_profile_.empty() || cum_s_.empty()) return;

    // Preview distance based on current filtered speed
    const double v_cur = clip(v_filt_, 0.0, v_max_);
    double s_cmd = preview_s_min_ + preview_t_ * v_cur;
    s_cmd = clip(s_cmd, preview_s_min_, preview_s_max_);

    // Find index by cumulative distance (lower_bound)
    const size_t N = cum_s_.size();
    size_t j = std::lower_bound(cum_s_.begin(), cum_s_.end(), s_cmd) - cum_s_.begin();

    double v_raw;
    if (j == 0) {
      v_raw = last_profile_[0];
    } else if (j >= N) {
      v_raw = last_profile_.back();
      j = N - 1;
    } else {
      const double s0 = cum_s_[j-1], s1 = cum_s_[j];
      const double w = (s1 > s0) ? ( (s_cmd - s0) / (s1 - s0) ) : 0.0;
      v_raw = (1.0 - w) * last_profile_[j-1] + w * last_profile_[j];
    }

    // Rate limiting + EMA smoothing
    const rclcpp::Time now = now_();
    const double dt = (last_cmd_time_.nanoseconds() == 0) ? (1.0 / std::max(1e-3, desired_pub_rate_hz_)) : (now - last_cmd_time_).seconds();
    last_cmd_time_ = now;

    const double dv_max = cmd_acc_limit_ * std::max(1e-3, dt);
    double v_limited = v_cmd_prev_ + std::clamp(v_raw - v_cmd_prev_, -dv_max, dv_max);

    // Optional EMA smoothing
    const double tau = std::max(1e-3, ema_tau_cmd_);
    const double alpha = std::exp(-std::max(0.0, dt) / tau);
    double v_cmd = alpha * v_cmd_prev_ + (1.0 - alpha) * v_limited;

    v_cmd = clip(v_cmd, v_min_, v_max_);
    v_cmd_prev_ = v_cmd;

    std_msgs::msg::Float32 msg;
    msg.data = static_cast<float>(v_cmd);
    pub_desired_->publish(msg);
  }

  inline double clip(double x, double lo, double hi) const {
    return std::max(lo, std::min(hi, x));
  }

  rclcpp::Time now_() { return this->get_clock()->now(); }

  // Parameters
  std::string path_topic_, speed_topic_, out_topic_, desired_speed_topic_;
  double recalc_rate_hz_{};
  double desired_pub_rate_hz_{};
  double horizon_m_{};
  double v_max_{}, v_min_{}, v_end_{};
  double a_acc_{}, a_brk_{}, ay_max_{};
  int    kappa_ma_window_{};
  double epsilon_kappa_{};
  double ema_tau_speed_{};
  double preview_t_{}; double preview_s_min_{}; double preview_s_max_{};
  double cmd_acc_limit_{}; double ema_tau_cmd_{};
  double log_throttle_sec_{};

  // State
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr sub_path_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr sub_speed_;
  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr pub_profile_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr pub_desired_;
  rclcpp::TimerBase::SharedPtr timer_recalc_;
  rclcpp::TimerBase::SharedPtr timer_cmd_;

  nav_msgs::msg::Path last_path_;
  bool have_path_{false};
  rclcpp::Time last_path_stamp_{};

  bool have_speed_{false};
  double v_filt_{0.0};
  rclcpp::Time last_speed_time_{};

  std::vector<double> last_profile_; // length = number of points in last path horizon
  std::vector<double> cum_s_;        // cumulative distance for each point

  rclcpp::Time last_cmd_time_{};
  double v_cmd_prev_{0.0};
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SimpleSpeedPlanner>());
  rclcpp::shutdown();
  return 0;
}