#include <cmath>
#include <vector>
#include <algorithm>
#include <numeric>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

/*
cmd_lookahead_time(기본 0.3s), cmd_lookahead_min/max(0.5~3.0m)를 먼저 만져보세요.
하드웨어가 민감하면 cmd_rate_up=1.0, cmd_rate_down=2.0처럼 더 보수적으로.
로컬 경로가 항상 ego에서 시작하지 않는 구조라면, 나중에 /current_pose(혹은 odom)를 받아 가장 가까운 인덱스(i0) + i_ahead로 바꾸면 더 정확해집니다.
*/

/*
speed[] 벡터는 앞으로 갈 M개 지점의 목표 속도 프로파일
그 중 speed[0] 은 현재 시점(즉, 다음 제어 사이클에서 적용할) 즉시 목표 속도
하드웨어 제어 노드는 이 값을 받아서 실제 차량이 그 속도로 가도록 제어
즉, 한 문장으로 말하면:
"모든 제약 조건을 반영해 계산한 속도 프로파일의 첫 번째 값(speed[0])을 하드웨어 제어 파트에 전달한다."

하드웨어 제어 파트에 전달하는 속도값은 **최종 속도 프로파일의 첫 번째 값(speed[0])**
*/

class LocalSpeedPlannerPreview : public rclcpp::Node
{
public:
  LocalSpeedPlannerPreview()
  : Node("speed_planner_preview")
  , current_speed_(0.0f)
  , current_speed_received_(false)
  , have_last_cmd_(false)
  {
    // Parameters
    declare_parameter<double>("max_speed",       20.0);   // [m/s]
    declare_parameter<double>("max_accel",        3.0);   // [m/s^2]
    declare_parameter<double>("max_decel",        3.0);   // [m/s^2]
    declare_parameter<double>("max_lat_accel",    2.5);   // [m/s^2]
    declare_parameter<double>("resample_ds",      0.30);  // [m] 등간격 리샘플 간격
    declare_parameter<double>("preview_distance", 6.0);   // [m] 프리뷰 길이
    declare_parameter<int>("smooth_window",       5);     // v_curve 이동평균 윈도우(홀수 권장)
    declare_parameter<double>("kappa_floor",      1e-4);  // 너무 작은 κ에 대한 바닥값(0 처리 방지)
    declare_parameter<double>("start_speed_fallback", 0.0); // /current_speed 없을 때 시작속도

    // Command (하드웨어용) 파라미터
    declare_parameter<double>("cmd_lookahead_time", 0.3); // [s] 응답시간 기반 look-ahead
    declare_parameter<double>("cmd_lookahead_min",  0.5); // [m] 최소 선행거리
    declare_parameter<double>("cmd_lookahead_max",  3.0); // [m] 최대 선행거리
    declare_parameter<double>("cmd_rate_up",        1.5); // [m/s^2] 상승 rate-limit
    declare_parameter<double>("cmd_rate_down",      3.0); // [m/s^2] 하강 rate-limit

    max_speed_        = get_parameter("max_speed").as_double();
    max_accel_        = get_parameter("max_accel").as_double();
    max_decel_        = get_parameter("max_decel").as_double();
    max_lat_accel_    = get_parameter("max_lat_accel").as_double();
    resample_ds_      = get_parameter("resample_ds").as_double();
    preview_dist_     = get_parameter("preview_distance").as_double();
    smooth_window_    = get_parameter("smooth_window").as_int();
    kappa_floor_      = get_parameter("kappa_floor").as_double();
    start_speed_fb_   = get_parameter("start_speed_fallback").as_double();

    cmd_lookahead_time_ = get_parameter("cmd_lookahead_time").as_double();
    cmd_lookahead_min_  = get_parameter("cmd_lookahead_min").as_double();
    cmd_lookahead_max_  = get_parameter("cmd_lookahead_max").as_double();
    cmd_rate_up_        = std::max(0.0, get_parameter("cmd_rate_up").as_double());
    cmd_rate_down_      = std::max(0.0, get_parameter("cmd_rate_down").as_double());

    // Publishers
    speed_pub_ = create_publisher<std_msgs::msg::Float32MultiArray>(
        "/desired_speed_profile", 10);
    desired_speed_pub_ = create_publisher<std_msgs::msg::Float32>(
        "/desired_speed", 10);

    vcurve_pub_ = create_publisher<std_msgs::msg::Float32MultiArray>(
        "/debug/v_curve_raw", 10);
    vpreview_pub_ = create_publisher<std_msgs::msg::Float32MultiArray>(
        "/debug/v_limit_preview", 10);
    kappa_pub_ = create_publisher<std_msgs::msg::Float32MultiArray>(
        "/debug/kappa", 10);
    resampled_path_pub_ = create_publisher<nav_msgs::msg::Path>(
        "/debug/resampled_path", 10);

    // Subscriptions
    path_sub_ = create_subscription<nav_msgs::msg::Path>(
        "/local_planned_path", 10,
        std::bind(&LocalSpeedPlannerPreview::pathCallback, this, std::placeholders::_1));

    speed_sub_ = create_subscription<std_msgs::msg::Float32>(
        "/current_speed", rclcpp::SensorDataQoS(),
        std::bind(&LocalSpeedPlannerPreview::speedCallback, this, std::placeholders::_1));
  }

private:
  // ---------- Utils ----------
  static inline double angleWrap(double a) {
    while (a > M_PI)  a -= 2.0*M_PI;
    while (a < -M_PI) a += 2.0*M_PI;
    return a;
  }
  static inline double angleDiff(double a, double b) {
    return angleWrap(a - b);
  }

  // 누적 s 계산
  static std::vector<double> cumulativeS(const std::vector<double>& xs,
                                         const std::vector<double>& ys)
  {
    const size_t N = xs.size();
    std::vector<double> s(N, 0.0);
    for (size_t i=1; i<N; ++i) {
      double dx = xs[i]-xs[i-1], dy = ys[i]-ys[i-1];
      s[i] = s[i-1] + std::hypot(dx, dy);
    }
    return s;
  }

  // s좌표 기반 선형보간
  static void interpAtS(const std::vector<double>& xs,
                        const std::vector<double>& ys,
                        const std::vector<double>& s,
                        double st, double& xo, double& yo)
  {
    // st in [0, s.back()]
    auto it = std::lower_bound(s.begin(), s.end(), st);
    if (it == s.begin()) { xo = xs.front(); yo = ys.front(); return; }
    if (it == s.end())   { xo = xs.back();  yo = ys.back();  return; }
    size_t i1 = std::distance(s.begin(), it);
    size_t i0 = i1 - 1;
    double t = (st - s[i0]) / (s[i1] - s[i0] + 1e-12);
    xo = xs[i0] + t * (xs[i1] - xs[i0]);
    yo = ys[i0] + t * (ys[i1] - ys[i0]);
  }

  static std::vector<double> movingAverage(const std::vector<double>& v, int w)
  {
    if (w <= 1) return v;
    if (w % 2 == 0) w += 1; // 홀수로
    const int hw = w / 2;
    const int N = static_cast<int>(v.size());
    std::vector<double> out(N, 0.0);
    double acc = 0.0;
    int cnt = 0;

    // 초기 윈도우
    for (int i=0; i<std::min(N, hw); ++i) { acc += v[i]; cnt++; }
    for (int i=0; i<N; ++i) {
      // 윈도우 [i-hw, i+hw]
      while (cnt < w && i+ (cnt - hw) < N) { acc += v[i+(cnt-hw)]; cnt++; }
      // 좌측 빠질 것
      int left = i - hw - 1;
      if (left >= 0) { acc -= v[left]; cnt--; }
      int right = i + hw;
      if (right < N) {
        // nothing, 이미 while에서 채움
      }
      out[i] = acc / std::max(1, cnt);
    }
    return out;
  }

  // ---------- ROS Callbacks ----------
  void speedCallback(const std_msgs::msg::Float32::SharedPtr msg)
  {
    current_speed_ = msg->data;
    current_speed_received_ = true;
  }

  void pathCallback(const nav_msgs::msg::Path::SharedPtr path_msg)
  {
    const size_t N_in = path_msg->poses.size();
    if (N_in < 2) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                           "Path too short (%zu) – skip", N_in);
      return;
    }

    // 1) 원본 xy 추출
    std::vector<double> x_in, y_in; x_in.reserve(N_in); y_in.reserve(N_in);
    for (const auto& ps : path_msg->poses) {
      x_in.push_back(ps.pose.position.x);
      y_in.push_back(ps.pose.position.y);
    }
    auto s_in = cumulativeS(x_in, y_in);
    const double L_total = s_in.back();
    if (L_total < resample_ds_*2.5) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                           "Path length %.2f < %.2fm – skip", L_total, resample_ds_*2.5);
      return;
    }

    // 2) 등간격 리샘플
    const int M = static_cast<int>(std::floor(L_total / resample_ds_)) + 1;
    std::vector<double> x(M), y(M), s(M);
    for (int i=0; i<M; ++i) {
      s[i] = std::min(i*resample_ds_, L_total);
      interpAtS(x_in, y_in, s_in, s[i], x[i], y[i]);
    }

    // 3) 헤딩 및 곡률(중앙차분: κ ≈ Δθ / Δs)
    std::vector<double> heading(M, 0.0);
    for (int i=0; i<M-1; ++i) {
      double dx = x[i+1]-x[i], dy = y[i+1]-y[i];
      heading[i] = std::atan2(dy, dx);
    }
    heading[M-1] = heading[M-2];

    std::vector<double> kappa(M, 0.0);
    if (M >= 3) {
      for (int i=1; i<M-1; ++i) {
        double dtheta = angleDiff(heading[i+1], heading[i-1]) * 0.5; // (θ_{i+1}-θ_{i-1})/2
        kappa[i] = dtheta / resample_ds_;
      }
      kappa[0]   = kappa[1];
      kappa[M-1] = kappa[M-2];
    }

    // 4) κ -> v_curve (바닥값 적용 + 이동평균 스무딩)
    std::vector<double> v_curve(M, max_speed_);
    for (int i=0; i<M; ++i) {
      double ak = std::fabs(kappa[i]);
      if (ak < kappa_floor_) { v_curve[i] = max_speed_; }
      else {
        v_curve[i] = std::sqrt(std::max(0.0, max_lat_accel_ / ak));
        v_curve[i] = std::min(v_curve[i], max_speed_);
      }
    }
    if (smooth_window_ > 1) {
      v_curve = movingAverage(v_curve, smooth_window_);
    }

    // 5) 프리뷰 기반 상한 v_limit_preview
    std::vector<double> v_limit_prev(M, max_speed_);
    const int win = std::max(1, static_cast<int>(std::round(preview_dist_ / resample_ds_)));
    for (int i=0; i<M; ++i) {
      int j_end = std::min(M-1, i + win);
      double max_abs_kappa = 0.0;
      for (int j=i; j<=j_end; ++j) {
        max_abs_kappa = std::max(max_abs_kappa, std::fabs(kappa[j]));
      }
      if (max_abs_kappa < kappa_floor_) {
        v_limit_prev[i] = max_speed_;
      } else {
        v_limit_prev[i] = std::sqrt(std::max(0.0, max_lat_accel_ / max_abs_kappa));
        v_limit_prev[i] = std::min(v_limit_prev[i], max_speed_);
      }
    }

    // 6) 초기 speed = preview 상한 복사
    std::vector<double> speed(M, max_speed_);
    for (int i=0; i<M; ++i) speed[i] = std::min(max_speed_, v_limit_prev[i]);

    // 시작점 현재속도 적용
    const double init_v = current_speed_received_ ? static_cast<double>(current_speed_) : start_speed_fb_;
    speed[0] = std::min(speed[0], init_v);

    // 7) Forward (가속 한계)
    for (int i=0; i<M-1; ++i) {
      double v_allow = std::sqrt(speed[i]*speed[i] + 2.0*max_accel_*resample_ds_);
      if (speed[i+1] > v_allow) speed[i+1] = v_allow;
    }

    // 8) Backward (감속 한계)
    for (int i=M-2; i>=0; --i) {
      double v_allow = std::sqrt(speed[i+1]*speed[i+1] + 2.0*max_decel_*resample_ds_);
      if (speed[i] > v_allow) speed[i] = v_allow;
    }

    // 9) (옵션) 매끈함을 위해 Forward 한 번 더
    for (int i=0; i<M-1; ++i) {
      double v_allow = std::sqrt(speed[i]*speed[i] + 2.0*max_accel_*resample_ds_);
      if (speed[i+1] > v_allow) speed[i+1] = v_allow;
    }

    // 10) 퍼블리시 (speed, 디버그 배열)
    std_msgs::msg::Float32MultiArray spd_msg, vcurve_msg, vprev_msg, kappa_msg;
    spd_msg.data.reserve(M);
    vcurve_msg.data.reserve(M);
    vprev_msg.data.reserve(M);
    kappa_msg.data.reserve(M);

    for (int i=0; i<M; ++i) {
      spd_msg.data.push_back(static_cast<float>(speed[i]));
      vcurve_msg.data.push_back(static_cast<float>(v_curve[i]));
      vprev_msg.data.push_back(static_cast<float>(v_limit_prev[i]));
      kappa_msg.data.push_back(static_cast<float>(kappa[i]));
    }
    speed_pub_->publish(spd_msg);
    vcurve_pub_->publish(vcurve_msg);
    vpreview_pub_->publish(vprev_msg);
    kappa_pub_->publish(kappa_msg);

    // 11) 하드웨어용 단일 속도 커맨드 계산 및 퍼블리시 (/desired_speed)
    const rclcpp::Time now = this->get_clock()->now();

    // 현재 속도 추정
    double v_curr = current_speed_received_ ? static_cast<double>(current_speed_) : speed[0];

    // look-ahead 거리
    double L_ahead = v_curr * cmd_lookahead_time_;
    L_ahead = std::clamp(L_ahead, cmd_lookahead_min_, cmd_lookahead_max_);

    int i_ahead = std::min(M-1, static_cast<int>(std::round(L_ahead / std::max(1e-6, resample_ds_))));
    double v_cmd_raw = speed[i_ahead];

    // 프레임 간 rate-limit
    double v_cmd = v_cmd_raw;
    if (have_last_cmd_) {
      double dt = (now - last_cmd_stamp_).seconds();
      if (dt > 0.0) {
        double up   = cmd_rate_up_   * dt;  // 상승 한계
        double down = cmd_rate_down_ * dt;  // 하강 한계
        v_cmd = std::min(v_cmd, last_cmd_speed_ + up);
        v_cmd = std::max(v_cmd, last_cmd_speed_ - down);
      }
    }
    last_cmd_speed_ = v_cmd;
    last_cmd_stamp_ = now;
    have_last_cmd_  = true;

    std_msgs::msg::Float32 vmsg;
    vmsg.data = static_cast<float>(v_cmd);
    desired_speed_pub_->publish(vmsg);

    // 12) 리샘플 경로 퍼블리시
    nav_msgs::msg::Path path_out;
    path_out.header = path_msg->header;
    path_out.header.stamp = now;
    path_out.poses.resize(M);
    for (int i=0; i<M; ++i) {
      path_out.poses[i].header = path_out.header;
      path_out.poses[i].pose.position.x = x[i];
      path_out.poses[i].pose.position.y = y[i];
      path_out.poses[i].pose.position.z = 0.0;
      path_out.poses[i].pose.orientation.w = 1.0;
    }
    resampled_path_pub_->publish(path_out);
  }

  // ---------- Members ----------
  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr speed_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr vcurve_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr vpreview_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr kappa_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr              resampled_path_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr           desired_speed_pub_;

  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr speed_sub_;

  float current_speed_;
  bool  current_speed_received_;

  // params
  double max_speed_, max_accel_, max_decel_, max_lat_accel_;
  double resample_ds_, preview_dist_, kappa_floor_, start_speed_fb_;
  int    smooth_window_;

  // command params/state
  double cmd_lookahead_time_, cmd_lookahead_min_, cmd_lookahead_max_;
  double cmd_rate_up_, cmd_rate_down_;
  double last_cmd_speed_;
  rclcpp::Time last_cmd_stamp_;
  bool have_last_cmd_;
};

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LocalSpeedPlannerPreview>());
  rclcpp::shutdown();
  return 0;
}
