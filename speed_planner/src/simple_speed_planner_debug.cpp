#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <nav_msgs/msg/path.hpp>

#include <algorithm>
#include <cmath>
#include <chrono>
#include <fstream>
#include <iomanip>
#include <limits>
#include <string>
#include <utility>
#include <vector>
#include <filesystem>

// Matplot++(선택) — CMake에서 Matplot++::matplot 링크 & HAVE_MATPLOT 정의
#ifdef HAVE_MATPLOT
#  include <matplot/matplot.h>
using namespace matplot;
#endif

class SimpleSpeedPlanner : public rclcpp::Node {
public:
  SimpleSpeedPlanner() : rclcpp::Node("simple_speed_planner") {
    // ==== Parameters ====
    path_topic_ = declare_parameter<std::string>("path_topic", "/local_planned_path");
    speed_topic_ = declare_parameter<std::string>("speed_topic", "/current_speed");
    out_topic_ = declare_parameter<std::string>("out_topic", "/desired_speed_profile");
    desired_speed_topic_ = declare_parameter<std::string>("desired_speed_topic", "/desired_speed");

    // RPM 변환 파라미터
    desired_rpm_topic_ = declare_parameter<std::string>("desired_rpm_topic", "/desired_rpm");
    wheel_diameter_m_ = declare_parameter<double>("wheel_diameter_m", 0.47);
    gear_ratio_motor_to_wheel_ = declare_parameter<double>("gear_ratio", 4.6);

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

    // === Debug & Logging ===
    debug_enable_ = declare_parameter<bool>("debug_enable", true);
    debug_dir_ = declare_parameter<std::string>("debug_dir", std::string("/home/ihw/workspace/kai_2025/src/Planning/speed_planner/data"));
    plot_enable_ = declare_parameter<bool>("plot_enable", true);
    inv_eps_ = declare_parameter<double>("inv_tolerance", 1e-3);

    // m/s → RPM 변환 계수(모터)
    static constexpr double kPi = 3.14159265358979323846;
    motor_rpm_per_mps_ = (60.0 * gear_ratio_motor_to_wheel_) / (kPi * wheel_diameter_m_);

    // Pub/Sub
    pub_profile_ = create_publisher<std_msgs::msg::Float32MultiArray>(out_topic_, 10);
    pub_desired_ = create_publisher<std_msgs::msg::Float32>(desired_speed_topic_, 10);
    pub_desired_rpm_ = create_publisher<std_msgs::msg::Float32>(desired_rpm_topic_, 10);

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

    // Debug dir 준비
    try {
      std::filesystem::create_directories(debug_dir_);
    } catch (...) {
      RCLCPP_WARN(get_logger(), "Failed to create debug dir: %s", debug_dir_.c_str());
    }

    start_time_ = now_();

    RCLCPP_INFO(get_logger(),
      "SimpleSpeedPlanner started. path='%s' speed='%s' → out='%s', desired='%s', desired_rpm='%s' (%.2f rpm per m/s) debug_dir='%s'",
      path_topic_.c_str(), speed_topic_.c_str(), out_topic_.c_str(),
      desired_speed_topic_.c_str(), desired_rpm_topic_.c_str(), motor_rpm_per_mps_, debug_dir_.c_str());
  }

  ~SimpleSpeedPlanner() override {
    if (debug_enable_) {
      dumpCsvAndPlots();
    }
  }

private:
  struct Pt { double x, y; };
  struct Snapshot {
    double t;                              // seconds since start
    std::vector<double> s, kappa, vlim, v_fwd, v_final, ay, slack;
    double min_slack; int violations;
  };
  struct CmdPoint { double t; double v_cmd; double v_meas; };

  // === Callbacks ===
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
    if (N < 2) return;

    // Horizon crop (optional)
    if (horizon_m_ > 0.0) {
      std::vector<Pt> cropped; cropped.reserve(N);
      cropped.push_back(pts.front());
      double acc = 0.0;
      for (size_t i = 1; i < N; ++i) {
        double ds = hypot(pts[i].x - pts[i-1].x, pts[i].y - pts[i-1].y);
        acc += ds; cropped.push_back(pts[i]);
        if (acc >= horizon_m_) break;
      }
      pts.swap(cropped); N = pts.size();
      if (N < 2) return;
    }

    // ds & cumulative s
    std::vector<double> ds(N-1, 0.0);
    for (size_t i = 1; i < N; ++i)
      ds[i-1] = std::max(1e-6, hypot(pts[i].x-pts[i-1].x, pts[i].y-pts[i-1].y));

    cum_s_.assign(N, 0.0);
    for (size_t i = 1; i < N; ++i) cum_s_[i] = cum_s_[i-1] + ds[i-1];

    // κ (3-point) + smoothing
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
        const double area2 = std::abs(ax * cy - ay * cx);
        const double denom = std::max(epsilon_kappa_, a * b * c);
        kappa[i] = area2 / denom;
      }
      kappa[0] = kappa[1]; kappa[N-1] = kappa[N-2];

      if (kappa_ma_window_ > 1) {
        const int w = kappa_ma_window_;
        std::vector<double> sm(N, 0.0);
        int half = w / 2;
        for (size_t i = 0; i < N; ++i) {
          int L = std::max<int>(0, static_cast<int>(i) - half);
          int R = std::min<int>(static_cast<int>(N)-1, static_cast<int>(i) + half);
          double sum = 0.0; for (int j = L; j <= R; ++j) sum += kappa[j];
          sm[i] = sum / std::max(1, R - L + 1);
        }
        kappa.swap(sm);
      }
    }

    // κ-기반 제한 속도(전역 vmax 함께 적용)
    std::vector<double> vlim(N, v_max_);
    for (size_t i = 0; i < N; ++i) {
      const double vmax_by_kappa = std::sqrt(ay_max_ / (std::abs(kappa[i]) + epsilon_kappa_));
      vlim[i] = std::min(v_max_, vmax_by_kappa);
    }

    // Forward pass
    std::vector<double> v(N, 0.0);
    const double v0 = clip(have_speed_ ? v_filt_ : 0.0, v_min_, vlim[0]);
    v[0] = v0;
    for (size_t i = 1; i < N; ++i) {
      const double v_acc = std::sqrt(std::max(0.0, v[i-1]*v[i-1] + 2.0 * a_acc_ * ds[i-1]));
      v[i] = std::min(vlim[i], v_acc);
    }
    std::vector<double> v_fwd = v; // 스냅샷용

    // Terminal + Backward pass
    v[N-1] = std::min(v[N-1], std::max(v_min_, v_end_));
    for (size_t i = N - 1; i-- > 0; ) {
      const double v_brk = std::sqrt(std::max(0.0, v[i+1]*v[i+1] + 2.0 * a_brk_ * ds[i]));
      v[i] = std::min(v[i], v_brk);
    }

    last_profile_ = v;

    // Output profile
    std_msgs::msg::Float32MultiArray out; out.data.resize(N);
    for (size_t i = 0; i < N; ++i) out.data[i] = static_cast<float>(clip(v[i], v_min_, v_max_));
    pub_profile_->publish(out);

    // === Debug snapshot ===
    if (debug_enable_) {
      Snapshot snap; snap.t = (now_() - start_time_).seconds();
      snap.s = cum_s_;
      snap.kappa = kappa;
      snap.vlim = vlim;
      snap.v_fwd = v_fwd;
      snap.v_final = v;
      snap.ay.resize(N); snap.slack.resize(N);
      double min_slack = std::numeric_limits<double>::infinity();
      int violations = 0;
      for (size_t i = 0; i < N; ++i) {
        const double ay = v[i]*v[i]*std::abs(kappa[i]);
        const double slack = ay_max_ - ay;
        snap.ay[i] = ay; snap.slack[i] = slack;
        if (slack < min_slack) min_slack = slack;
        if (slack < -inv_eps_) ++violations;
      }
      // forward/backward 제약 위반 체크
      for (size_t i = 0; i + 1 < N; ++i) {
        const double ds_i = std::max(1e-6, cum_s_[i+1]-cum_s_[i]);
        const double lhs_f = v[i+1]*v[i+1] - v[i]*v[i];
        const double rhs_f = 2.0 * a_acc_ * ds_i;
        if (lhs_f > rhs_f + inv_eps_) ++violations;
        const double lhs_b = v[i]*v[i] - v[i+1]*v[i+1];
        const double rhs_b = 2.0 * a_brk_ * ds_i;
        if (lhs_b > rhs_b + inv_eps_) ++violations;
      }
      snap.min_slack = min_slack; snap.violations = violations;
      snaps_.push_back(std::move(snap));
    }

    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), static_cast<uint64_t>(log_throttle_sec_ * 1000),
                         "Published speed profile: N=%zu, v0=%.2f, v_end=%.2f", N, v.front(), v.back());
  }

  void onCmdTimer() {
    if (last_profile_.empty() || cum_s_.empty()) return;

    // Preview distance
    const double v_cur = clip(v_filt_, 0.0, v_max_);
    double s_cmd = preview_s_min_ + preview_t_ * v_cur;
    s_cmd = clip(s_cmd, preview_s_min_, preview_s_max_);

    const size_t N = cum_s_.size();
    size_t j = std::lower_bound(cum_s_.begin(), cum_s_.end(), s_cmd) - cum_s_.begin();

    double v_raw;
    if (j == 0) v_raw = last_profile_[0];
    else if (j >= N) { v_raw = last_profile_.back(); j = N - 1; }
    else {
      const double s0 = cum_s_[j-1], s1 = cum_s_[j];
      const double w = (s1 > s0) ? ((s_cmd - s0) / (s1 - s0)) : 0.0;
      v_raw = (1.0 - w) * last_profile_[j-1] + w * last_profile_[j];
    }

    // Rate limiting + EMA
    const rclcpp::Time now = now_();
    const double dt = (last_cmd_time_.nanoseconds() == 0) ? (1.0 / std::max(1e-3, desired_pub_rate_hz_)) : (now - last_cmd_time_).seconds();
    last_cmd_time_ = now;

    const double dv_max = cmd_acc_limit_ * std::max(1e-3, dt);
    double v_limited = v_cmd_prev_ + std::clamp(v_raw - v_cmd_prev_, -dv_max, dv_max);

    const double tau = std::max(1e-3, ema_tau_cmd_);
    const double alpha = std::exp(-std::max(0.0, dt) / tau);
    double v_cmd = alpha * v_cmd_prev_ + (1.0 - alpha) * v_limited;

    v_cmd = clip(v_cmd, v_min_, v_max_);
    v_cmd_prev_ = v_cmd;

    // Publish desired speed & rpm
    std_msgs::msg::Float32 msg_speed; msg_speed.data = static_cast<float>(v_cmd);
    pub_desired_->publish(msg_speed);
    std_msgs::msg::Float32 msg_rpm; msg_rpm.data = static_cast<float>(v_cmd * motor_rpm_per_mps_);
    pub_desired_rpm_->publish(msg_rpm);

    // Log command trace (for time-series plots)
    if (debug_enable_) {
      CmdPoint cp; cp.t = (now - start_time_).seconds(); cp.v_cmd = v_cmd; cp.v_meas = v_filt_;
      cmd_log_.push_back(cp);
    }
  }

  inline double clip(double x, double lo, double hi) const {
    return std::max(lo, std::min(hi, x));
  }

  rclcpp::Time now_() { return this->get_clock()->now(); }

  // === Dump CSV + Plot ===
  void dumpCsvAndPlots() {
    if (snaps_.empty()) {
      RCLCPP_WARN(get_logger(), "No snapshots to dump/plot.");
      return;
    }

    // 1) recalc time series
    const std::string ts_csv = debug_dir_ + "/recalc_timeseries.csv";
    {
      std::ofstream ofs(ts_csv);
      ofs << "t,min_slack,violations\n";
      ofs.setf(std::ios::fixed); ofs << std::setprecision(6);
      for (const auto &s : snaps_) ofs << s.t << "," << s.min_slack << "," << s.violations << "\n";
    }

    // 2) command time series
    const std::string cmd_csv = debug_dir_ + "/cmd_timeseries.csv";
    {
      std::ofstream ofs(cmd_csv);
      ofs << "t,v_cmd,v_meas\n";
      ofs.setf(std::ios::fixed); ofs << std::setprecision(6);
      for (const auto &c : cmd_log_) ofs << c.t << "," << c.v_cmd << "," << c.v_meas << "\n";
    }

    // 3) last snapshot profile arrays
    const Snapshot &L = snaps_.back();
    const std::string prof_csv = debug_dir_ + "/last_profile.csv";
    {
      std::ofstream ofs(prof_csv);
      ofs << "i,s,kappa,vlim,v_fwd,v_final,ay,slack\n";
      ofs.setf(std::ios::fixed); ofs << std::setprecision(6);
      for (size_t i = 0; i < L.s.size(); ++i) {
        ofs << i << "," << L.s[i] << "," << L.kappa[i] << "," << L.vlim[i]
            << "," << L.v_fwd[i] << "," << L.v_final[i]
            << "," << L.ay[i] << "," << L.slack[i] << "\n";
      }
    }

#ifdef HAVE_MATPLOT
    if (plot_enable_) {
      try {
        // Figure 1: s vs vcurves (last snapshot)
        auto f1 = figure(true);
        f1->size(900, 600);
        auto ax1 = gca();
        plot(L.s, L.vlim)->display_name("vlim_kappa"); hold(on);
        plot(L.s, L.v_fwd)->display_name("v_forward");
        plot(L.s, L.v_final)->display_name("v_final");
        legend(); xlabel("s [m]"); ylabel("v [m/s]"); grid(on);
        save(f1, debug_dir_ + "/v_profile.png");

        // Figure 2: s vs kappa
        auto f2 = figure(true); f2->size(900, 300);
        plot(L.s, L.kappa);
        xlabel("s [m]"); ylabel("kappa [1/m]"); grid(on);
        save(f2, debug_dir_ + "/kappa.png");

        // Figure 3: t vs min_slack
        std::vector<double> T(snaps_.size()), S(snaps_.size());
        for (size_t i = 0; i < snaps_.size(); ++i) { T[i] = snaps_[i].t; S[i] = snaps_[i].min_slack; }
        auto f3 = figure(true); f3->size(900, 300);
        plot(T, S); xlabel("t [s]"); ylabel("min_slack [m/s^2]"); grid(on);
        save(f3, debug_dir_ + "/min_slack.png");

        // Figure 4: t vs v_cmd & v_meas
        std::vector<double> Tc(cmd_log_.size()), Vc(cmd_log_.size()), Vm(cmd_log_.size());
        for (size_t i = 0; i < cmd_log_.size(); ++i) { Tc[i] = cmd_log_[i].t; Vc[i] = cmd_log_[i].v_cmd; Vm[i] = cmd_log_[i].v_meas; }
        auto f4 = figure(true); f4->size(900, 300);
        plot(Tc, Vc)->display_name("v_cmd"); hold(on);
        plot(Tc, Vm)->display_name("v_meas"); legend(); xlabel("t [s]"); ylabel("v [m/s]"); grid(on);
        save(f4, debug_dir_ + "/v_cmd_meas.png");

        RCLCPP_INFO(get_logger(), "Saved plots to %s", debug_dir_.c_str());
      } catch (const std::exception &e) {
        RCLCPP_WARN(get_logger(), "Matplot++ plot failed: %s", e.what());
      }
    }
#else
    (void)plot_enable_; // unused
#endif

    RCLCPP_INFO(get_logger(), "Wrote CSV: %s, %s, %s", ts_csv.c_str(), cmd_csv.c_str(), prof_csv.c_str());
  }

private:
  // Params
  std::string path_topic_, speed_topic_, out_topic_, desired_speed_topic_;
  std::string desired_rpm_topic_;
  double wheel_diameter_m_{};
  double gear_ratio_motor_to_wheel_{};
  double motor_rpm_per_mps_{};

  double recalc_rate_hz_{};
  double desired_pub_rate_hz_{};
  double horizon_m_{};
  double v_max_{}, v_min_{}, v_end_{};
  double a_acc_{}, a_brk_{}, ay_max_{};
  int    kappa_ma_window_{}; double epsilon_kappa_{};
  double ema_tau_speed_{};
  double preview_t_{}; double preview_s_min_{}; double preview_s_max_{};
  double cmd_acc_limit_{}; double ema_tau_cmd_{};
  double log_throttle_sec_{};

  bool   debug_enable_{true};
  std::string debug_dir_{"/tmp/speed_debug"};
  bool   plot_enable_{true};
  double inv_eps_{1e-3};

  // State
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr sub_path_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr sub_speed_;
  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr pub_profile_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr pub_desired_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr pub_desired_rpm_;
  rclcpp::TimerBase::SharedPtr timer_recalc_;
  rclcpp::TimerBase::SharedPtr timer_cmd_;

  nav_msgs::msg::Path last_path_;
  bool have_path_{false};
  rclcpp::Time last_path_stamp_{};

  bool have_speed_{false};
  double v_filt_{0.0};
  rclcpp::Time last_speed_time_{};

  std::vector<double> last_profile_;
  std::vector<double> cum_s_;

  rclcpp::Time last_cmd_time_{};
  double v_cmd_prev_{0.0};

  rclcpp::Time start_time_{};
  std::vector<Snapshot> snaps_;
  std::vector<CmdPoint> cmd_log_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SimpleSpeedPlanner>());
  rclcpp::shutdown();
  return 0;
}
