#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <nav_msgs/msg/path.hpp>

#include <vector>
#include <cmath>
#include <algorithm>

class SimpleSpeedPlanner : public rclcpp::Node {
public:
  SimpleSpeedPlanner() : rclcpp::Node("simple_speed_planner") {
    // Parameters
    path_topic_ = declare_parameter<std::string>("path_topic", "/local_planned_path");
    speed_topic_ = declare_parameter<std::string>("speed_topic", "/current_speed");
    out_topic_ = declare_parameter<std::string>("out_topic", "/desired_speed_profile");

    recalc_rate_hz_ = declare_parameter<double>("recalc_rate_hz", 15.0);
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
    log_throttle_sec_ = declare_parameter<double>("log_throttle_sec", 2.0);

    // Pub/Sub
    pub_profile_ = create_publisher<std_msgs::msg::Float32MultiArray>(out_topic_, 10);

    sub_path_ = create_subscription<nav_msgs::msg::Path>(
        path_topic_, rclcpp::QoS(10),
        std::bind(&SimpleSpeedPlanner::onPath, this, std::placeholders::_1));

    sub_speed_ = create_subscription<std_msgs::msg::Float32>(
        speed_topic_, rclcpp::SensorDataQoS(),
        std::bind(&SimpleSpeedPlanner::onSpeed, this, std::placeholders::_1));

    // Timer
    const auto period = std::chrono::duration<double>(1.0 / std::max(1e-3, recalc_rate_hz_));
    timer_ = create_wall_timer(std::chrono::duration_cast<std::chrono::milliseconds>(period),
                               std::bind(&SimpleSpeedPlanner::onTimer, this));

    RCLCPP_INFO(get_logger(), "SimpleSpeedPlanner started. path='%s' speed='%s' → out='%s'",
                path_topic_.c_str(), speed_topic_.c_str(), out_topic_.c_str());
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

  void onTimer() {
    if (!have_path_) return;

    // Extract points
    std::vector<Pt> pts; pts.reserve(last_path_.poses.size());
    for (const auto &ps : last_path_.poses) {
      pts.push_back({ps.pose.position.x, ps.pose.position.y});
    }
    const size_t N = pts.size();
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
    }

    const size_t M = pts.size();
    if (M == 1) {
      // Edge case: single point → publish single desired speed (anchored to current)
      std_msgs::msg::Float32MultiArray out;
      out.data.resize(1);
      out.data[0] = static_cast<float>(clip(v_filt_, v_min_, v_max_));
      pub_profile_->publish(out);
      return;
    }

    // Segment distances (M-1)
    std::vector<double> ds(M-1, 0.0);
    for (size_t i = 1; i < M; ++i) {
      ds[i-1] = std::max(1e-6, hypot(pts[i].x - pts[i-1].x, pts[i].y - pts[i-1].y));
    }

    // Curvature per point (M). Using 3-point circle curvature, abs value.
    std::vector<double> kappa(M, 0.0);
    if (M >= 3) {
      for (size_t i = 1; i + 1 < M; ++i) {
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
      kappa[M-1] = kappa[M-2];

      // Moving average smoothing
      if (kappa_ma_window_ > 1) {
        const int w = kappa_ma_window_;
        std::vector<double> sm(M, 0.0);
        int half = w / 2;
        double acc = 0.0;
        // initial window
        for (int i = 0; i < std::min<int>(M, w); ++i) acc += kappa[i];
        for (size_t i = 0; i < M; ++i) {
          int L = std::max<int>(0, static_cast<int>(i) - half);
          int R = std::min<int>(M-1, static_cast<int>(i) + half);
          double sum = 0.0;
          for (int j = L; j <= R; ++j) sum += kappa[j];
          sm[i] = sum / std::max(1, R - L + 1);
        }
        kappa.swap(sm);
      }
    }

    // Curvature-limited speed
    std::vector<double> vlim(M, v_max_);
    for (size_t i = 0; i < M; ++i) {
      const double vmax_by_kappa = std::sqrt(ay_max_ / (std::abs(kappa[i]) + epsilon_kappa_));
      vlim[i] = std::min(v_max_, vmax_by_kappa);
    }

    // Forward pass (accel limit)
    std::vector<double> v(M, 0.0);
    const double v0 = clip(have_speed_ ? v_filt_ : 0.0, v_min_, vlim[0]);
    v[0] = v0;
    for (size_t i = 1; i < M; ++i) {
      const double v_acc = std::sqrt(std::max(0.0, v[i-1]*v[i-1] + 2.0 * a_acc_ * ds[i-1]));
      v[i] = std::min(vlim[i], v_acc);
    }

    // Enforce terminal desired speed and Backward pass (brake limit)
    v[M-1] = std::min(v[M-1], std::max(v_min_, v_end_));
    for (size_t i = M - 1; i-- > 0; ) {
      const double v_brk = std::sqrt(std::max(0.0, v[i+1]*v[i+1] + 2.0 * a_brk_ * ds[i]));
      v[i] = std::min(v[i], v_brk);
    }

    // Publish profile (Float32MultiArray, length = number of path points)
    std_msgs::msg::Float32MultiArray out;
    out.data.resize(M);
    for (size_t i = 0; i < M; ++i) {
      out.data[i] = static_cast<float>(clip(v[i], v_min_, v_max_));
    }
    pub_profile_->publish(out);

    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), static_cast<uint64_t>(log_throttle_sec_ * 1000),
                         "Published speed profile: N=%zu, v0=%.2f, v_end=%.2f", M, v[0], v.back());
  }

  inline double clip(double x, double lo, double hi) const {
    return std::max(lo, std::min(hi, x));
  }

  rclcpp::Time now_() { return this->get_clock()->now(); }

  // Parameters
  std::string path_topic_, speed_topic_, out_topic_;
  double recalc_rate_hz_{};
  double horizon_m_{};
  double v_max_{}, v_min_{}, v_end_{};
  double a_acc_{}, a_brk_{}, ay_max_{};
  int    kappa_ma_window_{};
  double epsilon_kappa_{};
  double ema_tau_speed_{};
  double log_throttle_sec_{};

  // State
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr sub_path_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr sub_speed_;
  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr pub_profile_;
  rclcpp::TimerBase::SharedPtr timer_;

  nav_msgs::msg::Path last_path_;
  bool have_path_{false};
  rclcpp::Time last_path_stamp_{};

  bool have_speed_{false};
  double v_filt_{0.0};
  rclcpp::Time last_speed_time_{};
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SimpleSpeedPlanner>());
  rclcpp::shutdown();
  return 0;
}