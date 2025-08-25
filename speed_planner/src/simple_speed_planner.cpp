#include <cmath>
#include <vector>
#include <algorithm>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

class SimpleLocalSpeedPlanner : public rclcpp::Node
{
public:
  SimpleLocalSpeedPlanner()
  : Node("simple_speed_planner"),
    current_speed_(0.0f),
    have_curr_speed_(false),
    have_last_cmd_(false),
    last_cmd_speed_(0.0),
    last_cmd_accel_(0.0)
  {
    // ------ Parameters ------
    declare_parameter<double>("max_speed",        10.0);   // [m/s]
    declare_parameter<double>("max_accel",         3.0);   // [m/s^2]
    declare_parameter<double>("max_decel",         3.0);   // [m/s^2]
    declare_parameter<double>("max_lat_accel",     2.5);   // [m/s^2]
    declare_parameter<double>("resample_ds",       0.30);  // [m]
    declare_parameter<double>("kappa_floor",       1e-4);  // [-]
    declare_parameter<int>("smooth_window",        5);     // odd recommended
    declare_parameter<double>("start_speed_fallback", 0.0);// [m/s]

    // command (low-level) limiter
    declare_parameter<double>("cmd_lookahead_time", 0.3);  // [s]
    declare_parameter<double>("cmd_lookahead_min",  0.5);  // [m]
    declare_parameter<double>("cmd_lookahead_max",  3.0);  // [m]
    declare_parameter<double>("cmd_accel_up",       1.5);  // [m/s^2] max up
    declare_parameter<double>("cmd_accel_down",     3.0);  // [m/s^2] max down
    declare_parameter<double>("cmd_jerk_max",       4.0);  // [m/s^3] jerk limit

    // read params
    max_speed_     = get_parameter("max_speed").as_double();
    max_accel_     = get_parameter("max_accel").as_double();
    max_decel_     = get_parameter("max_decel").as_double();
    max_lat_acc_   = get_parameter("max_lat_accel").as_double();
    ds_            = get_parameter("resample_ds").as_double();
    kappa_floor_   = get_parameter("kappa_floor").as_double();
    smooth_win_    = get_parameter("smooth_window").as_int();
    start_fb_      = get_parameter("start_speed_fallback").as_double();

    look_t_        = get_parameter("cmd_lookahead_time").as_double();
    look_min_      = get_parameter("cmd_lookahead_min").as_double();
    look_max_      = get_parameter("cmd_lookahead_max").as_double();
    a_up_          = std::max(0.0, get_parameter("cmd_accel_up").as_double());
    a_down_        = std::max(0.0, get_parameter("cmd_accel_down").as_double());
    j_max_         = std::max(0.0, get_parameter("cmd_jerk_max").as_double());

    // ------ Publishers ------
    spd_profile_pub_ = create_publisher<std_msgs::msg::Float32MultiArray>("/desired_speed_profile", 10);
    desired_speed_pub_ = create_publisher<std_msgs::msg::Float32>("/desired_speed", 10);

    // ------ Subscriptions ------
    path_sub_ = create_subscription<nav_msgs::msg::Path>(
      "/local_planned_path", 10,
      std::bind(&SimpleLocalSpeedPlanner::onPath, this, std::placeholders::_1));

    // 현재 속도: 너의 환경은 /current_speed (Float32)
    speed_sub_ = create_subscription<std_msgs::msg::Float32>(
      "/current_speed", rclcpp::SensorDataQoS(),
      std::bind(&SimpleLocalSpeedPlanner::onCurrentSpeed, this, std::placeholders::_1));
  }

private:
  // ---------- Utils ----------
  static inline double wrap(double a){
    while(a>M_PI) a-=2*M_PI;
    while(a<-M_PI)a+=2*M_PI;
    return a;
  }
  static inline double angdiff(double a, double b){ return wrap(a-b); }

  static std::vector<double> cumS(const std::vector<double>& x, const std::vector<double>& y){
    size_t N=x.size(); std::vector<double> s(N,0.0);
    for(size_t i=1;i<N;++i){ s[i]=s[i-1]+std::hypot(x[i]-x[i-1], y[i]-y[i-1]); }
    return s;
  }

  static void interp_s(const std::vector<double>& x,const std::vector<double>& y,const std::vector<double>& s,
                       double st,double& xo,double& yo){
    auto it = std::lower_bound(s.begin(), s.end(), st);
    if(it==s.begin()){ xo=x.front(); yo=y.front(); return; }
    if(it==s.end()){   xo=x.back();  yo=y.back();  return; }
    size_t i1 = std::distance(s.begin(), it), i0=i1-1;
    double t = (st - s[i0]) / std::max(1e-12, s[i1]-s[i0]);
    xo = x[i0] + t*(x[i1]-x[i0]);
    yo = y[i0] + t*(y[i1]-y[i0]);
  }

  static std::vector<double> movavg(const std::vector<double>& v, int w){
    if(w<=1) return v;
    if(w%2==0) ++w;
    int N=(int)v.size(), hw=w/2;
    std::vector<double> out(N,0.0);
    double acc=0.0; int cnt=0;

    for(int i=0;i<std::min(N,hw);++i){ acc+=v[i]; ++cnt; }
    for(int i=0;i<N;++i){
      while(cnt<w && i+(cnt-hw)<N){ acc += v[i+(cnt-hw)]; ++cnt; }
      int left=i-hw-1;
      if(left>=0){ acc -= v[left]; --cnt; }
      out[i] = acc/std::max(1,cnt);
    }
    return out;
  }

  // ---------- Callbacks ----------
  void onCurrentSpeed(const std_msgs::msg::Float32::SharedPtr msg){
    current_speed_ = msg->data;
    have_curr_speed_=true;
  }

  void onPath(const nav_msgs::msg::Path::SharedPtr path_msg)
  {
    const size_t Nin = path_msg->poses.size();
    if(Nin<2){
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "Path too short (%zu) – skip", Nin);
      return;
    }

    // 1) 원본 좌표 추출
    std::vector<double> x,y; x.reserve(Nin); y.reserve(Nin);
    for(const auto& ps : path_msg->poses){ x.push_back(ps.pose.position.x); y.push_back(ps.pose.position.y); }
    auto s_in = cumS(x,y);
    const double L = s_in.back();
    if(L < ds_*2.5){
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "Path length %.2f < %.2f – skip", L, ds_*2.5);
      return;
    }

    // 2) 등간격 리샘플
    const int M = (int)std::floor(L/ds_) + 1;
    std::vector<double> xr(M), yr(M), sr(M);
    for(int i=0;i<M;++i){
      sr[i] = std::min(i*ds_, L);
      interp_s(x,y,s_in,sr[i], xr[i], yr[i]);
    }

    // 3) 헤딩/곡률 (중앙차분)
    std::vector<double> theta(M,0.0);
    for(int i=0;i<M-1;++i){
      double dx=xr[i+1]-xr[i], dy=yr[i+1]-yr[i];
      theta[i]=std::atan2(dy,dx);
    }
    theta[M-1]=theta[M-2];

    std::vector<double> kappa(M,0.0);
    if(M>=3){
      for(int i=1;i<M-1;++i){
        double dth = angdiff(theta[i+1], theta[i-1])*0.5;
        kappa[i] = dth / ds_;
      }
      kappa[0]=kappa[1]; kappa[M-1]=kappa[M-2];
    }
    if(smooth_win_>1) kappa = movavg(kappa, smooth_win_);

    // 4) 곡률 기반 상한
    std::vector<double> v(M, max_speed_);
    for(int i=0;i<M;++i){
      double ak = std::fabs(kappa[i]);
      if(ak < kappa_floor_) v[i]=max_speed_;
      else{
        v[i] = std::min(max_speed_, std::sqrt(std::max(0.0, max_lat_acc_/ak)));
      }
    }

    // 5) 출발점 현재속도 상한
    const double v0 = have_curr_speed_ ? (double)current_speed_ : start_fb_;
    v[0] = std::min(v[0], v0);

    // 6) Forward/Backward accel 한계 (거리 기반)
    for(int i=0;i<M-1;++i){
      double v_allow = std::sqrt(v[i]*v[i] + 2.0*max_accel_*ds_);
      if(v[i+1] > v_allow) v[i+1]=v_allow;
    }
    for(int i=M-2;i>=0;--i){
      double v_allow = std::sqrt(v[i+1]*v[i+1] + 2.0*max_decel_*ds_);
      if(v[i] > v_allow) v[i]=v_allow;
    }
    // 매끈함 개선 Forward 1회
    for(int i=0;i<M-1;++i){
      double v_allow = std::sqrt(v[i]*v[i] + 2.0*max_accel_*ds_);
      if(v[i+1] > v_allow) v[i+1]=v_allow;
    }

    // 7) 프로파일 퍼블리시
    std_msgs::msg::Float32MultiArray prof;
    prof.data.reserve(M);
    for(int i=0;i<M;++i) prof.data.push_back((float)v[i]);
    spd_profile_pub_->publish(prof);

    // 8) 단일 커맨드 (/desired_speed): lookahead + 2차 rate-limit(가속, 저크)
    const rclcpp::Time now = this->get_clock()->now();
    double v_curr = have_curr_speed_ ? (double)current_speed_ : v[0];

    double L_ahead = std::clamp(v_curr*look_t_, look_min_, look_max_);
    int i_ahead = std::min(M-1, (int)std::round(L_ahead / std::max(1e-6, ds_)));
    double v_target = v[i_ahead];

    double v_cmd = v_target;
    if(have_last_cmd_){
      double dt = std::max(1e-3, (now - last_cmd_stamp_).seconds());
      // 1) 속도->가속 목표
      double a_raw = (v_target - last_cmd_speed_) / dt;
      // 2) 가속 한계 (상/하행)
      double a_clamp = std::clamp(a_raw, -a_down_, a_up_);
      // 3) 저크 한계 (가속 변화 제한)
      double da = j_max_ * dt;
      double a_limited = std::clamp(a_clamp, last_cmd_accel_ - da, last_cmd_accel_ + da);
      // 4) 적분
      v_cmd = last_cmd_speed_ + a_limited * dt;
      last_cmd_accel_ = a_limited;
    }
    last_cmd_speed_  = v_cmd;
    last_cmd_stamp_  = now;
    have_last_cmd_   = true;

    std_msgs::msg::Float32 cmdmsg; cmdmsg.data = (float)v_cmd;
    desired_speed_pub_->publish(cmdmsg);
  }

  // ---------- Members ----------
  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr spd_profile_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr           desired_speed_pub_;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr           path_sub_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr        speed_sub_;

  float  current_speed_;
  bool   have_curr_speed_;

  // params
  double max_speed_, max_accel_, max_decel_, max_lat_acc_;
  double ds_, kappa_floor_, start_fb_;
  int    smooth_win_;

  // command params/state
  double look_t_, look_min_, look_max_;
  double a_up_, a_down_, j_max_;
  bool   have_last_cmd_;
  double last_cmd_speed_, last_cmd_accel_;
  rclcpp::Time last_cmd_stamp_;
};

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SimpleLocalSpeedPlanner>());
  rclcpp::shutdown();
  return 0;
}
