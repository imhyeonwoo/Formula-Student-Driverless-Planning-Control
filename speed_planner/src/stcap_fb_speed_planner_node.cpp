#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <nav_msgs/msg/path.hpp>

#include <algorithm>
#include <cmath>
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
#include <string>
#include <vector>

/*
 * stcap_fb_speed_planner_node.cpp
 * ─────────────────────────────────────────────────────────────
 * 콘 트랙 + 로컬 경로만 있는 환경에서, 곡률을 쓰지 않고도 속도 프로파일을 생성하는 노드.
 * 방법: (1) 직선성(윈도우 기반 헤딩 변화량 & Chord 오차)으로 속도 상한 산출
 *       (2) 로컬 경로 끝까지의 정지거리 상한 결합
 *       (3) Forward/Backward 패스로 가/감속 한계 반영
 *       (4) (옵션) s-도메인 이동평균으로 평활화
 * 입력:  /local_planned_path (nav_msgs/Path, base_link 기준 등간격 권장)
 *        /current_speed (std_msgs/Float32, m/s)
 * 출력:  /desired_speed_profile (std_msgs/Float32MultiArray, Path 점 순서와 1:1 매칭)
 *
 * 참고: reference_path_planning.py 노드가 본 토픽을 구독하여 Waypoint.speed와 속도 막대(height=z)를 갱신.
 */

using std::placeholders::_1;

class StcapFbSpeedPlannerNode : public rclcpp::Node {
public:
  StcapFbSpeedPlannerNode() : Node("stcap_fb_speed_planner") {
    // ── Parameters (기본값은 보수적으로 시작)
    v_max_ = declare_parameter<double>("v_max", 6.0);           // [m/s]
    v_min_ = declare_parameter<double>("v_min", 1.0);           // [m/s]
    a_max_ = declare_parameter<double>("a_max", 1.8);           // [m/s^2]
    d_max_ = declare_parameter<double>("d_max", 2.2);           // [m/s^2] (decel limit, 양수)
    a_brake_ = declare_parameter<double>("a_brake", 2.5);       // [m/s^2] for stop-distance cap

    window_len_m_ = declare_parameter<double>("window_len_m", 6.0); // 직선성 평가 윈도 길이
    k_theta_ = declare_parameter<double>("k_theta", 8.0);       // (m/s)/rad
    k_e_ = declare_parameter<double>("k_e", 1.2);               // (m/s)/m

    smooth_window_m_ = declare_parameter<double>("smooth_window_m", 1.0); // 이동평균 창 길이 [m]

    // Subscriptions & Publisher
    sub_path_ = create_subscription<nav_msgs::msg::Path>(
        "/local_planned_path", rclcpp::QoS(10).best_effort(),
        std::bind(&StcapFbSpeedPlannerNode::onPath, this, _1));

    sub_speed_ = create_subscription<std_msgs::msg::Float32>(
        "/current_speed", rclcpp::QoS(10).best_effort(),
        std::bind(&StcapFbSpeedPlannerNode::onSpeed, this, _1));

    pub_profile_ = create_publisher<std_msgs::msg::Float32MultiArray>(
        "/desired_speed_profile", rclcpp::QoS(10).reliable());

    RCLCPP_INFO(get_logger(), "stcap_fb_speed_planner started.");
  }

private:
  // ────────────────────────────────────────────────
  void onSpeed(const std_msgs::msg::Float32::SharedPtr msg) {
    current_speed_ = static_cast<double>(msg->data);
  }

  void onPath(const nav_msgs::msg::Path::SharedPtr msg) {
    const auto &poses = msg->poses;
    const size_t N = poses.size();

    if (N < 2) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                           "Path too short: %zu", N);
      return;
    }

    // 좌표 추출
    std::vector<double> X(N), Y(N);
    for (size_t i = 0; i < N; ++i) {
      X[i] = poses[i].pose.position.x;
      Y[i] = poses[i].pose.position.y;
    }

    // 호길이/세그먼트 길이 계산
    std::vector<double> ds(N - 1);
    std::vector<double> s(N, 0.0);
    double sum_ds = 0.0;
    for (size_t i = 0; i + 1 < N; ++i) {
      const double dx = X[i + 1] - X[i];
      const double dy = Y[i + 1] - Y[i];
      const double seg = std::hypot(dx, dy);
      ds[i] = seg;
      sum_ds += seg;
      s[i + 1] = s[i] + seg;
    }
    const double path_len = s.back();
    if (path_len < 1e-6) {
      RCLCPP_WARN(get_logger(), "Degenerate path (length≈0). Skip.");
      return;
    }

    const double mean_ds = (N > 1) ? (path_len / (N - 1)) : path_len;

    // 헤딩(세그먼트 기준)
    std::vector<double> psi(N, 0.0);
    for (size_t i = 0; i + 1 < N; ++i) {
      psi[i] = std::atan2(Y[i + 1] - Y[i], X[i + 1] - X[i]);
    }
    psi[N - 1] = psi[N - 2];

    // 직선성 기반 상한 v_cap_S
    std::vector<double> v_cap_S(N, v_max_);
    for (size_t i = 0; i < N; ++i) {
      // j: s[j] - s[i] >= window_len_m_
      size_t j = i;
      const double target = s[i] + window_len_m_;
      while (j + 1 < N && s[j] < target) ++j;
      if (j <= i) j = std::min(i + 1, N - 1);

      const double theta = std::fabs(angleDiff(psi[j], psi[i])); // |Δψ|

      // Chord 오차 최대값
      const double e = maxChordError(X, Y, i, j);

      double v_cap_turn = v_max_ - k_theta_ * theta;
      double v_cap_chord = v_max_ - k_e_ * e;
      double v_cap = std::min(v_cap_turn, v_cap_chord);
      v_cap = std::clamp(v_cap, v_min_, v_max_);
      v_cap_S[i] = v_cap;
    }

    // 끝점 정지거리 상한 v_cap_stop
    std::vector<double> v_cap_stop(N, v_max_);
    for (size_t i = 0; i < N; ++i) {
      const double d_remain = s.back() - s[i];
      const double vstop = std::sqrt(std::max(0.0, 2.0 * a_brake_ * d_remain));
      v_cap_stop[i] = std::min(vstop, v_max_);
    }

    // 최종 상한 결합
    std::vector<double> v_cap(N);
    for (size_t i = 0; i < N; ++i) v_cap[i] = std::min(v_cap_S[i], v_cap_stop[i]);

    // Forward / Backward 패스
    std::vector<double> v(N, v_min_);
    v[0] = std::clamp(current_speed_, v_min_, v_cap[0]);

    // Forward (가속)
    for (size_t i = 0; i + 1 < N; ++i) {
      const double v_next_from_acc = std::sqrt(std::max(0.0, v[i] * v[i] + 2.0 * a_max_ * ds[i]));
      v[i + 1] = std::min(v_cap[i + 1], v_next_from_acc);
      v[i + 1] = std::clamp(v[i + 1], v_min_, v_cap[i + 1]);
    }

    // Backward (감속)
    for (size_t i = N - 1; i >= 1; --i) {
      const double v_prev_from_dec = std::sqrt(std::max(0.0, v[i] * v[i] + 2.0 * d_max_ * ds[i - 1]));
      v[i - 1] = std::min(v[i - 1], v_prev_from_dec);
      if (i == 1) break; // size_t underflow 방지
    }

    // (옵션) s-도메인 이동평균 평활화
    if (smooth_window_m_ > 1e-6) {
      const int win = std::max(1, static_cast<int>(std::round(smooth_window_m_ / std::max(1e-3, mean_ds))));
      movingAverageInPlace(v, win);
    }

    // 퍼블리시 (Path 점수와 동일 길이)
    std_msgs::msg::Float32MultiArray out;
    out.data.resize(N);
    for (size_t i = 0; i < N; ++i) out.data[i] = static_cast<float>(v[i]);

    out.layout.dim.resize(1);
    out.layout.dim[0].label = "s"; // 호길이 인덱스
    out.layout.dim[0].size = N;
    out.layout.dim[0].stride = N;

    pub_profile_->publish(out);
  }

  // ────────────────────────────────────────────────
  static double angleWrap(double a) {
    // [-pi, pi]
    while (a > M_PI) a -= 2.0 * M_PI;
    while (a < -M_PI) a += 2.0 * M_PI;
    return a;
  }

  static double angleDiff(double a, double b) { return angleWrap(a - b); }

  static double maxChordError(const std::vector<double> &X, const std::vector<double> &Y,
                              size_t i, size_t j) {
    if (j <= i) return 0.0;
    const double x1 = X[i], y1 = Y[i];
    const double x2 = X[j], y2 = Y[j];
    const double vx = x2 - x1, vy = y2 - y1;
    const double denom = std::hypot(vx, vy);
    if (denom < 1e-9) return 0.0;

    double emax = 0.0;
    for (size_t k = i; k <= j; ++k) {
      const double num = std::fabs(vy * X[k] - vx * Y[k] + x2 * y1 - y2 * x1);
      const double d = num / denom;
      if (d > emax) emax = d;
    }
    return emax;
  }

  static void movingAverageInPlace(std::vector<double> &v, int win) {
    if (win <= 1) return;
    const int N = static_cast<int>(v.size());
    std::vector<double> acc(N + 1, 0.0);
    for (int i = 0; i < N; ++i) acc[i + 1] = acc[i] + v[i];

    std::vector<double> out(N);
    for (int i = 0; i < N; ++i) {
      const int a = std::max(0, i - win);
      const int b = std::min(N - 1, i + win);
      const int cnt = b - a + 1;
      const double sum = acc[b + 1] - acc[a];
      out[i] = sum / static_cast<double>(cnt);
    }
    v.swap(out);
  }

private:
  // Params
  double v_max_{6.0}, v_min_{1.0};
  double a_max_{1.8}, d_max_{2.2}, a_brake_{2.5};
  double window_len_m_{6.0};
  double k_theta_{8.0}, k_e_{1.2};
  double smooth_window_m_{1.0};

  // State
  double current_speed_{0.0};

  // ROS I/F
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr sub_path_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr sub_speed_;
  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr pub_profile_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<StcapFbSpeedPlannerNode>());
  rclcpp::shutdown();
  return 0;
}
