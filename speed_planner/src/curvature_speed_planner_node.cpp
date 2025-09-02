#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/path.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <geometry_msgs/msg/point.hpp>

#include <vector>
#include <string>
#include <algorithm>
#include <cmath>

class CurvatureSpeedPlannerNode : public rclcpp::Node {
public:
  CurvatureSpeedPlannerNode() : Node("speed_planner")
  {
    // ========== 파라미터 ==========
    path_topic_    = declare_parameter<std::string>("path_topic", "/local_planned_path");
    cur_spd_topic_ = declare_parameter<std::string>("current_speed_topic", "/current_speed");
    profile_topic_ = declare_parameter<std::string>("desired_speed_profile_topic", "/desired_speed_profile");
    vcmd_topic_    = declare_parameter<std::string>("v_cmd_topic", "/cmd/speed");

    publish_v_cmd_   = declare_parameter<bool>("publish_v_cmd", true);
    publish_rate_hz_ = declare_parameter<double>("publish_rate_hz", 20.0);

    // v_cmd 인덱싱/옵션
    cmd_index_mode_  = declare_parameter<std::string>("cmd_index_mode", "adaptive"); // "distance"|"index"|"time"|"adaptive"
    cmd_offset_m_    = declare_parameter<double>("cmd_offset_m", 1.5);
    fixed_index_offset_ = declare_parameter<int>("fixed_index_offset", 3);
    preview_time_       = declare_parameter<double>("preview_time", 0.5);

    preview_time_min_ = declare_parameter<double>("preview_time_min", 0.25);
    preview_time_max_ = declare_parameter<double>("preview_time_max", 0.80);
    kappa_beta_       = declare_parameter<double>("kappa_beta", 5.0);
    kappa_window_     = declare_parameter<int>("kappa_window", 5);

    use_window_min_ = declare_parameter<bool>("use_window_min", true);
    look_ahead_count_ = declare_parameter<int>("look_ahead_count", 3);
    seed_from_current_speed_ = declare_parameter<bool>("seed_from_current_speed", true);
    clamp_accel_ = declare_parameter<bool>("clamp_accel", true);

    ds_ = declare_parameter<double>("ds", 0.5);
    v_min_ = declare_parameter<double>("v_min", 0.2);
    v_max_ = declare_parameter<double>("v_max", 4.0);
    a_long_max_ = declare_parameter<double>("a_long_max", 3.0);
    a_long_min_ = declare_parameter<double>("a_long_min", -3.0);
    a_lat_max_  = declare_parameter<double>("a_lat_max", 3.0);

    eps_kappa_ = declare_parameter<double>("eps_kappa", 1e-6);
    smooth_window_ = declare_parameter<int>("smooth_window", 3);

    publish_debug_markers_ = declare_parameter<bool>("publish_debug_markers", false);

    // ===== RPM 발행 관련 파라미터 =====
    publish_rpm_cmd_   = declare_parameter<bool>("publish_rpm_cmd", true);
    rpm_topic_         = declare_parameter<std::string>("rpm_topic", "/cmd/rpm");
    wheel_diameter_m_  = declare_parameter<double>("wheel_diameter_m", 0.47); // 47cm
    gear_ratio_        = declare_parameter<double>("gear_ratio", 4.6);        // motor:wheel
    rpm_min_           = declare_parameter<double>("rpm_min", 0.0);           // 필요시 사용
    rpm_max_           = declare_parameter<double>("rpm_max", 6000.0);        // 컨트롤러 한계
    rpm_round_to_int_  = declare_parameter<bool>("rpm_round_to_int", false);  // true면 반올림

    // ========== I/O ==========
    sub_path_ = create_subscription<nav_msgs::msg::Path>(
      path_topic_, rclcpp::QoS(10),
      std::bind(&CurvatureSpeedPlannerNode::onPath, this, std::placeholders::_1));

    sub_current_speed_ = create_subscription<std_msgs::msg::Float32>(
      cur_spd_topic_, rclcpp::QoS(10),
      std::bind(&CurvatureSpeedPlannerNode::onCurrentSpeed, this, std::placeholders::_1));

    pub_profile_ = create_publisher<std_msgs::msg::Float32MultiArray>(profile_topic_, 10);
    if (publish_v_cmd_) {
      pub_vcmd_ = create_publisher<std_msgs::msg::Float32>(vcmd_topic_, 10);
    }
    if (publish_rpm_cmd_) {
      pub_rpm_ = create_publisher<std_msgs::msg::Float32>(rpm_topic_, 10);
    }
    if (publish_debug_markers_) {
      pub_dbg_ = create_publisher<visualization_msgs::msg::MarkerArray>("/speed_planner/debug", 1);
    }

    // v_cmd 타이머
    if (publish_v_cmd_ && publish_rate_hz_ > 0.0) {
      using namespace std::chrono_literals;
      auto period = std::chrono::duration<double>(1.0 / publish_rate_hz_);
      timer_ = create_wall_timer(
        std::chrono::duration_cast<std::chrono::milliseconds>(period),
        std::bind(&CurvatureSpeedPlannerNode::onTimer, this));
    }

    RCLCPP_INFO(get_logger(),
      "CurvatureSpeedPlannerNode ready. mode=%s, ds=%.3f, v:[%.2f,%.2f], a_long:[%.2f,%.2f], a_lat_max=%.2f, rpm_topic=%s",
      cmd_index_mode_.c_str(), ds_, v_min_, v_max_, a_long_min_, a_long_max_, a_lat_max_, rpm_topic_.c_str());
  }

private:
  // ========== 콜백: 현재 속도 ==========
  void onCurrentSpeed(const std_msgs::msg::Float32::SharedPtr msg) {
    current_speed_ = static_cast<double>(msg->data);
    last_speed_time_ = now();
  }

  // ========== 콜백: 경로 수신 → 속도 프로파일 계산/발행 ==========
  void onPath(const nav_msgs::msg::Path::SharedPtr msg) {
    const auto& poses = msg->poses;
    if (poses.size() < 3) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                           "Path too short (<3). Skip profile calc.");
      return;
    }

    const size_t N = poses.size();
    std::vector<double> xs(N), ys(N);
    for (size_t i = 0; i < N; ++i) {
      xs[i] = poses[i].pose.position.x;
      ys[i] = poses[i].pose.position.y;
    }

    // --- 곡률 계산(중심차분, 등간격 ds) ---
    std::vector<double> kappa(N, 0.0);
    for (size_t i = 1; i + 1 < N; ++i) {
      const double xm1 = xs[i-1], x0 = xs[i], xp1 = xs[i+1];
      const double ym1 = ys[i-1], y0 = ys[i], yp1 = ys[i+1];

      const double dx  = (xp1 - xm1) / (2.0 * ds_);
      const double dy  = (yp1 - ym1) / (2.0 * ds_);
      const double ddx = (xp1 - 2.0 * x0 + xm1) / (ds_ * ds_);
      const double ddy = (yp1 - 2.0 * y0 + ym1) / (ds_ * ds_);

      const double num = std::abs(dx * ddy - dy * ddx);
      const double den = std::pow(dx*dx + dy*dy, 1.5) + 1e-12;
      kappa[i] = num / den;
    }
    kappa.front() = kappa[1];
    kappa.back()  = kappa[N-2];

    // --- 곡률 기반 속도 한계 v_kappa ---
    std::vector<double> vcap(N);
    for (size_t i = 0; i < N; ++i) {
      const double v_curv = std::sqrt(a_lat_max_ / (std::abs(kappa[i]) + eps_kappa_));
      vcap[i] = std::clamp(v_curv, v_min_, v_max_);
    }

    // --- Forward pass (가속 한계) ---
    std::vector<double> vf(N, 0.0);
    double v0 = std::clamp(current_speed_, v_min_, vcap[0]);
    vf[0] = v0;
    const double amax = std::max(0.0, a_long_max_);
    for (size_t i = 1; i < N; ++i) {
      const double v_lim_acc = std::sqrt(vf[i-1]*vf[i-1] + 2.0 * amax * ds_);
      vf[i] = std::min(vcap[i], v_lim_acc);
    }

    // --- Backward pass (제동 한계) ---
    std::vector<double> v(N, 0.0);
    v[N-1] = vf[N-1];
    const double amin_abs = std::abs(std::min(0.0, a_long_min_));
    for (size_t i = N-1; i > 0; --i) {
      const double v_lim_brake = std::sqrt(v[i]*v[i] + 2.0 * amin_abs * ds_);
      v[i-1] = std::min(vf[i-1], v_lim_brake);
    }

    // --- 스무딩(이동평균, 경계 보존) ---
    if (smooth_window_ > 1) {
      v = movingAverageCenterKeepEdge(v, smooth_window_);
      for (auto &vi : v) vi = std::clamp(vi, v_min_, v_max_);
    }

    latest_profile_ = v;
    latest_kappa_   = kappa;
    publishProfile(latest_profile_);

    if (publish_debug_markers_) {
      publishDebugMarkers(xs, ys, kappa, latest_profile_, msg->header);
    }
  }

  // ========== 주기 타이머: v_cmd(+ rpm) 발행 ==========
  void onTimer() {
    if (!publish_v_cmd_ || latest_profile_.empty()) return;

    const size_t N = latest_profile_.size();
    const double v_now = std::clamp(current_speed_, v_min_, v_max_);

    // --- 1) 인덱스 시작점 i0 계산 (모드별) ---
    size_t i0 = 0;
    if (cmd_index_mode_ == "distance") {
      i0 = static_cast<size_t>(std::round(std::max(0.0, cmd_offset_m_) / std::max(1e-6, ds_)));
    } else if (cmd_index_mode_ == "index") {
      i0 = static_cast<size_t>(std::max(0, fixed_index_offset_));
    } else if (cmd_index_mode_ == "time") {
      double tau = std::clamp(preview_time_, 0.05, 2.0);
      i0 = static_cast<size_t>(std::round((v_now * tau) / std::max(1e-6, ds_)));
    } else { // "adaptive"
      double k_now = 0.0;
      if (!latest_kappa_.empty() && kappa_window_ > 0) {
        int W = std::min<int>(kappa_window_, static_cast<int>(latest_kappa_.size()));
        double sum = 0.0;
        for (int i = 0; i < W; ++i) sum += std::abs(latest_kappa_[i]);
        k_now = sum / static_cast<double>(W);
      } else if (!latest_kappa_.empty()) {
        k_now = std::abs(latest_kappa_.front());
      }
      double tau = preview_time_min_ + (preview_time_max_ - preview_time_min_) / (1.0 + kappa_beta_ * k_now);
      tau = std::clamp(tau, preview_time_min_, preview_time_max_);
      i0 = static_cast<size_t>(std::round((v_now * tau) / std::max(1e-6, ds_)));
    }
    if (i0 >= N) i0 = N - 1;

    // --- 2) lookahead 최솟값(보수적 옵션) ---
    size_t end_idx = std::min(N - 1, i0 + static_cast<size_t>(std::max(0, look_ahead_count_)));
    double v_ref = latest_profile_[i0];
    if (use_window_min_) {
      for (size_t i = i0 + 1; i <= end_idx; ++i)
        v_ref = std::min(v_ref, latest_profile_[i]);
    }

    // --- 3) 첫 발행 시 가속 클램프 기준 시드 ---
    const rclcpp::Time nowt = now();
    if (first_cmd_pub_) {
      last_v_cmd_ = seed_from_current_speed_ ? v_now : v_ref;
      last_cmd_time_ = nowt;
      first_cmd_pub_ = false;
    }

    // --- 4) 가/감속 클램프 ---
    double v_cmd = v_ref;
    if (clamp_accel_) {
      const double dt = (last_cmd_time_.nanoseconds() > 0)
                        ? (nowt - last_cmd_time_).seconds()
                        : (1.0 / std::max(1.0, publish_rate_hz_));
      const double v_lo = last_v_cmd_ - std::abs(a_long_min_) * dt;
      const double v_hi = last_v_cmd_ + a_long_max_ * dt;
      v_cmd = std::clamp(v_cmd, v_lo, v_hi);
    }
    last_cmd_time_ = nowt;

    // --- 5) 범위 클램프 + 발행 (/cmd/speed) ---
    v_cmd = std::clamp(v_cmd, v_min_, v_max_);
    last_v_cmd_ = v_cmd;

    std_msgs::msg::Float32 out;
    out.data = static_cast<float>(v_cmd);
    pub_vcmd_->publish(out);

    // --- 6) RPM으로 변환해 발행 (/cmd/rpm) ---
    if (publish_rpm_cmd_) {
      // 파라미터 유효성 체크
      if (wheel_diameter_m_ > 1e-6 && gear_ratio_ > 1e-6) {
        constexpr double PI = 3.14159265358979323846;
        // RPM_motor = v[m/s] * 60 * gear_ratio / (pi * diameter)
        double rpm_cmd = v_cmd * 60.0 * gear_ratio_ / (PI * wheel_diameter_m_);
        // (옵션) 클램프 및 반올림
        if (rpm_max_ > rpm_min_) {
          rpm_cmd = std::clamp(rpm_cmd, rpm_min_, rpm_max_);
        }
        if (rpm_round_to_int_) {
          rpm_cmd = std::round(rpm_cmd);
        }
        std_msgs::msg::Float32 rpm_msg;
        rpm_msg.data = static_cast<float>(rpm_cmd);
        pub_rpm_->publish(rpm_msg);
      } else {
        // 한 번만 경고
        if (!warned_bad_rpm_params_) {
          RCLCPP_WARN(get_logger(),
                      "RPM publish skipped: invalid wheel_diameter_m(%.3f) or gear_ratio(%.3f).",
                      wheel_diameter_m_, gear_ratio_);
          warned_bad_rpm_params_ = true;
        }
      }
    }
  }

  // ========== 프로파일 발행 ==========
  void publishProfile(const std::vector<double>& v) {
    std_msgs::msg::Float32MultiArray arr;
    arr.data.resize(v.size());
    for (size_t i = 0; i < v.size(); ++i) arr.data[i] = static_cast<float>(v[i]);
    pub_profile_->publish(arr);
  }

  // ========== 이동평균(중앙창, 경계 보존) ==========
  static std::vector<double> movingAverageCenterKeepEdge(const std::vector<double>& x, int W) {
    if (W <= 1 || (int)x.size() <= W) return x;
    std::vector<double> y(x.size());
    const int half = W / 2;

    double run = 0.0;
    for (int i = 0; i < W; ++i) run += x[i];
    for (size_t i = 0; i < x.size(); ++i) {
      if ((int)i - half < 0 || i + half >= x.size()) {
        y[i] = x[i]; // 경계: 원본 유지
      } else {
        if (i == (size_t)half) {
          y[i] = run / W;
        } else {
          run += x[i + half] - x[i - half - 1];
          y[i] = run / W;
        }
      }
    }
    return y;
  }

  // ========== 디버그 마커 ==========
  void publishDebugMarkers(const std::vector<double>& xs,
                           const std::vector<double>& ys,
                           const std::vector<double>& kappa,
                           const std::vector<double>& v,
                           const std_msgs::msg::Header& hdr_in)
  {
    visualization_msgs::msg::MarkerArray arr;

    // 곡률 라인 (z축=|kappa|)
    {
      visualization_msgs::msg::Marker m;
      m.header = hdr_in; // frame_id = base_link
      m.ns = "kappa_line";
      m.id = 0;
      m.type = visualization_msgs::msg::Marker::LINE_STRIP;
      m.action = visualization_msgs::msg::Marker::ADD;
      m.scale.x = 0.02;
      m.color.r = 1.0; m.color.g = 1.0; m.color.b = 1.0; m.color.a = 1.0;
      const double scale_z = 0.5;
      for (size_t i = 0; i < xs.size(); ++i) {
        geometry_msgs::msg::Point p;
        p.x = xs[i];
        p.y = ys[i];
        p.z = std::min(1.5, std::abs(kappa[i]) * scale_z);
        m.points.push_back(p);
      }
      arr.markers.push_back(m);
    }

    // 속도 라인 (z축=v)
    {
      visualization_msgs::msg::Marker m;
      m.header = hdr_in;
      m.ns = "v_profile_line";
      m.id = 1;
      m.type = visualization_msgs::msg::Marker::LINE_STRIP;
      m.action = visualization_msgs::msg::Marker::ADD;
      m.scale.x = 0.02;
      m.color.r = 0.0; m.color.g = 1.0; m.color.b = 0.0; m.color.a = 1.0;
      const double scale_z = 0.3;
      for (size_t i = 0; i < xs.size() && i < v.size(); ++i) {
        geometry_msgs::msg::Point p;
        p.x = xs[i];
        p.y = ys[i];
        p.z = v[i] * scale_z;
        m.points.push_back(p);
      }
      arr.markers.push_back(m);
    }

    pub_dbg_->publish(arr);
  }

private:
  // 파라미터/옵션
  std::string path_topic_, cur_spd_topic_, profile_topic_, vcmd_topic_;
  bool publish_v_cmd_{true}, clamp_accel_{true}, publish_debug_markers_{false};
  double publish_rate_hz_{20.0};

  // 인덱싱 모드 및 파라미터
  std::string cmd_index_mode_{"adaptive"};
  double cmd_offset_m_{1.5};
  int    fixed_index_offset_{3};
  double preview_time_{0.5};
  double preview_time_min_{0.25}, preview_time_max_{0.80};
  double kappa_beta_{5.0};
  int    kappa_window_{5};
  bool   use_window_min_{true};
  int    look_ahead_count_{3};
  bool   seed_from_current_speed_{true};

  // 한계/샘플
  double ds_{0.5};
  double v_min_{0.2}, v_max_{4.0};
  double a_long_max_{3.0}, a_long_min_{-3.0}, a_lat_max_{3.0};
  double eps_kappa_{1e-6};
  int    smooth_window_{3};

  // RPM 관련
  bool publish_rpm_cmd_{true};
  std::string rpm_topic_{"/cmd/rpm"};
  double wheel_diameter_m_{0.47};
  double gear_ratio_{4.6};
  double rpm_min_{0.0}, rpm_max_{6000.0};
  bool   rpm_round_to_int_{false};
  bool   warned_bad_rpm_params_{false};

  // 상태
  double current_speed_{0.0};
  rclcpp::Time last_speed_time_;
  std::vector<double> latest_profile_;
  std::vector<double> latest_kappa_;
  double last_v_cmd_{0.0};
  rclcpp::Time last_cmd_time_;
  bool first_cmd_pub_{true};

  // ROS I/O
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr sub_path_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr sub_current_speed_;
  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr pub_profile_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr pub_vcmd_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr pub_rpm_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_dbg_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CurvatureSpeedPlannerNode>());
  rclcpp::shutdown();
  return 0;
}
