#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32.hpp>
#include <nav_msgs/msg/path.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <vector>
#include <string>
#include <algorithm>
#include <cmath>
#include <chrono>
#include <array>

class PurePursuitAdaptive : public rclcpp::Node {
public:
  PurePursuitAdaptive() : rclcpp::Node("pure_pursuit_adaptive") {
    // ----- Parameters -----
    path_topic_    = declare_parameter<std::string>("path_topic", "/local_planned_path");
    speed_topic_   = declare_parameter<std::string>("speed_topic", "/current_speed");
    steer_topic_   = declare_parameter<std::string>("steer_topic", "/cmd/steer");
    debug_topic_   = declare_parameter<std::string>("debug_marker_topic", "/pure_pursuit/debug_markers");
    base_frame_    = declare_parameter<std::string>("base_frame", "base_link");

    wheelbase_m_   = declare_parameter<double>("wheelbase_m", 1.295);

    // LAD 구성: 속도형 + (옵션) 곡률 보정
    use_speed_term_ = declare_parameter<bool>("use_speed_term", true);
    L0_             = declare_parameter<double>("L0", 1.5);
    k_v_            = declare_parameter<double>("k_v", 0.6);
    use_curv_term_  = declare_parameter<bool>("use_curvature_term", true);
    k_curv_         = declare_parameter<double>("k_curv", 0.0);       // >0면 직선에서 LAD 커짐
    eps_kappa_      = declare_parameter<double>("epsilon_kappa", 1e-6);
    Ld_min_         = declare_parameter<double>("Ld_min", 1.0);
    Ld_max_         = declare_parameter<double>("Ld_max", 6.0);

    // 타깃 선택
    use_arc_length_selection_ = declare_parameter<bool>("use_arc_length_selection", true);
    x_forward_only_           = declare_parameter<bool>("x_forward_only", true);
    forward_margin_x_         = declare_parameter<double>("forward_margin_x", -0.2); // x>-0.2만 전방으로 간주

    // 커팅 억제(pd->pl 바깥 이동)
    outer_offset_enable_  = declare_parameter<bool>("outer_offset_enable", true);
    alpha_max_m_          = declare_parameter<double>("alpha_max_m", 3.0);
    beta_max_             = declare_parameter<double>("beta_max", 3.0);
    curv_window_m_        = declare_parameter<double>("curv_window_m", 2.0);
    outer_offset_max_m_   = declare_parameter<double>("outer_offset_max_m", 1.0);
    outer_offset_tau_max_ = declare_parameter<double>("outer_offset_tau_max", 0.7);
    outer_offset_kappa_gate_ = declare_parameter<double>("outer_offset_kappa_gate", 0.03);
    track_half_width_m_   = declare_parameter<double>("track_half_width_m", 0.0);
    track_margin_m_       = declare_parameter<double>("track_margin_m", 0.2);

    // 제어/명령 필터
    publish_rate_hz_      = declare_parameter<double>("publish_rate_hz", 50.0);
    steer_limit_deg_      = declare_parameter<double>("steer_limit_deg", 30.0);
    steer_rate_limit_deg_per_s_ = declare_parameter<double>("steer_rate_limit_deg_per_s", 360.0);
    ema_tau_cmd_          = declare_parameter<double>("ema_tau_cmd", 0.12);
    ema_tau_speed_        = declare_parameter<double>("ema_tau_speed", 0.20);

    // 노이즈 억제(추가)
    sticky_window_pts_        = declare_parameter<int>("sticky_window_pts", 15);          // ±포인트
    kappa_smooth_window_pts_  = declare_parameter<int>("kappa_smooth_window_pts", 3);     // 곡률 이동평균 절반폭
    target_ema_tau_           = declare_parameter<double>("target_ema_tau", 0.08);        // pl 좌표 EMA

    // 디버그 시각화
    show_lookahead_circle_ = declare_parameter<bool>("show_lookahead_circle", true);
    show_points_           = declare_parameter<bool>("show_points", true);     // pw/pd/pl
    show_tangent_          = declare_parameter<bool>("show_tangent", true);
    show_offset_vec_       = declare_parameter<bool>("show_offset_vec", true);
    show_selected_path_    = declare_parameter<bool>("show_selected_path", false);
    show_steer_arrow_      = declare_parameter<bool>("show_steer_arrow", true);
    show_steer_text_       = declare_parameter<bool>("show_steer_text", true);
    marker_alpha_          = declare_parameter<double>("marker_alpha", 1.0);
    circle_pts_            = declare_parameter<int>("circle_points", 60);

    // Colors (r,g,b)
    color_pw_  = { declare_parameter<double>("color_pw_r", 0.10), declare_parameter<double>("color_pw_g", 0.6),  declare_parameter<double>("color_pw_b", 1.0) };
    color_pd_  = { declare_parameter<double>("color_pd_r", 0.10), declare_parameter<double>("color_pd_g", 1.0),  declare_parameter<double>("color_pd_b", 0.10) };
    color_pl_  = { declare_parameter<double>("color_pl_r", 1.00), declare_parameter<double>("color_pl_g", 0.1),  declare_parameter<double>("color_pl_b", 0.80) };
    color_tan_ = { declare_parameter<double>("color_tan_r",1.00), declare_parameter<double>("color_tan_g",0.85), declare_parameter<double>("color_tan_b",0.10) };
    color_vec_ = { declare_parameter<double>("color_vec_r",0.00), declare_parameter<double>("color_vec_g",0.9),  declare_parameter<double>("color_vec_b",1.00) };
    color_cir_ = { declare_parameter<double>("color_cir_r",1.00), declare_parameter<double>("color_cir_g",1.00), declare_parameter<double>("color_cir_b",1.00) };

    // ----- Pub/Sub -----
    sub_path_ = create_subscription<nav_msgs::msg::Path>(
      path_topic_, rclcpp::QoS(10),
      std::bind(&PurePursuitAdaptive::onPath, this, std::placeholders::_1));

    sub_speed_ = create_subscription<std_msgs::msg::Float32>(
      speed_topic_, rclcpp::SensorDataQoS(),
      std::bind(&PurePursuitAdaptive::onSpeed, this, std::placeholders::_1));

    pub_steer_ = create_publisher<std_msgs::msg::Float32>(steer_topic_, 10);
    pub_debug_ = create_publisher<visualization_msgs::msg::MarkerArray>(debug_topic_, 10);

    // ----- Timer -----
    using namespace std::chrono_literals;
    const auto period = std::chrono::duration<double>(1.0 / std::max(1e-3, publish_rate_hz_));
    timer_ = create_wall_timer(std::chrono::duration_cast<std::chrono::milliseconds>(period),
              std::bind(&PurePursuitAdaptive::onTimer, this));

    RCLCPP_INFO(get_logger(), "PurePursuitAdaptive started. path='%s', speed='%s' -> steer='%s' (deg)",
                path_topic_.c_str(), speed_topic_.c_str(), steer_topic_.c_str());
  }

private:
  struct Pt { double x, y; };
  static constexpr double kPi = 3.14159265358979323846;

  // ---------- Callbacks ----------
  void onPath(const nav_msgs::msg::Path::SharedPtr msg) {
    last_path_ = *msg;
    frame_id_from_path_ = last_path_.header.frame_id.empty() ? base_frame_ : last_path_.header.frame_id;

    pts_.clear();
    pts_.reserve(last_path_.poses.size());
    for (const auto& ps : last_path_.poses) {
      pts_.push_back({static_cast<double>(ps.pose.position.x),
                      static_cast<double>(ps.pose.position.y)});
    }
    // 누적 호길이
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

  // ---------- Timer loop ----------
  void onTimer() {
    // 0) 안전검사
    if (pts_.size() < 2) {
      publishSteerDeg(smoothDeg(0.0));
      publishDebugMarkersEmpty();
      return;
    }

    // 1) LAD 계산
    const double Ld = computeLd();

    // 2) pw/pd 선택 (스티키 윈도우)
    int idx_pw = findNearestIndexSticky();
    if (idx_pw < 0) { publishSteerDeg(smoothDeg(0.0)); publishDebugMarkersEmpty(); return; }
    int idx_pd = selectTargetIndexSticky(idx_pw, Ld);
    if (idx_pd < 0) idx_pd = static_cast<int>(pts_.size()) - 1;

    Pt pw = pts_[static_cast<size_t>(idx_pw)];
    Pt pd = pts_[static_cast<size_t>(idx_pd)];
    Pt pl = pd;

    // 3) 바깥 이동(커팅 코너 억제 + 게이팅/상한)
    if (outer_offset_enable_) {
      const double k_pw = signedKappaSmoothedAt(idx_pw, kappa_smooth_window_pts_);
      const double k_pd = signedKappaSmoothedAt(idx_pd, kappa_smooth_window_pts_);

      // 근접/곡률비 기반 가중
      const double alpha = std::min(1.0, std::hypot(pw.x, pw.y) / std::max(1e-6, alpha_max_m_));
      double beta = 0.0;
      if (std::abs(k_pw) > 1e-12 && std::abs(k_pd) > std::abs(k_pw)) {
        beta = std::min(1.0, (std::abs(k_pd) / std::abs(k_pw)) / std::max(1e-6, beta_max_));
      }
      double tau = (1.0 - alpha) * beta;
      tau = std::clamp(tau, 0.0, outer_offset_tau_max_);

      // 거의 직선이면 오프셋 끔
      if (std::abs(k_pd) < outer_offset_kappa_gate_) tau = 0.0;

      // pd에서의 접선/법선, 바깥 방향
      Pt t = tangentAt(idx_pd);
      const double tnorm = std::hypot(t.x, t.y);
      if (tnorm > 1e-6) { t.x/=tnorm; t.y/=tnorm; } else { t = {1.0, 0.0}; }
      Pt n = { -t.y, t.x }; // 좌측 법선
      const double sign_k = (k_pd >= 0.0) ? +1.0 : -1.0;   // 좌회전(+): 안쪽은 +n
      Pt outward = { -sign_k * n.x, -sign_k * n.y };      // 바깥 = -sign(k)*n

      // 거리 상한
      const double d_pw_pd = std::hypot(pd.x - pw.x, pd.y - pw.y);
      double off = std::min(tau * d_pw_pd, outer_offset_max_m_);
      if (track_half_width_m_ > 0.0) {
        off = std::min(off, std::max(0.0, track_half_width_m_ - track_margin_m_));
      }

      pl = { pd.x + outward.x * off, pd.y + outward.y * off };
    }

    // 4) pl 좌표 EMA(타깃 스무딩)
    {
      const rclcpp::Time now = now_();
      const double dt = (last_target_time_.nanoseconds()==0) ? (1.0 / std::max(1e-3, publish_rate_hz_))
                        : (now - last_target_time_).seconds();
      last_target_time_ = now;
      const double alpha = std::exp(-std::max(0.0, dt) / std::max(1e-3, target_ema_tau_));
      if (!have_pl_prev_) { pl_prev_ = pl; have_pl_prev_ = true; }
      pl = { alpha*pl_prev_.x + (1.0 - alpha)*pl.x,
             alpha*pl_prev_.y + (1.0 - alpha)*pl.y };
      pl_prev_ = pl;
    }

    // 5) 조향 계산 (좌회전 +deg)
    const double Ld2 = std::max(1e-9, pl.x*pl.x + pl.y*pl.y);
    const double delta_rad = std::atan2(2.0 * wheelbase_m_ * pl.y, Ld2);
    double delta_deg = delta_rad * 180.0 / kPi;

    // 6) rate-limit + EMA + 제한
    delta_deg = clip(delta_deg, -steer_limit_deg_, steer_limit_deg_);
    const double cmd_deg = smoothDeg(delta_deg);

    // 7) Publish
    publishSteerDeg(cmd_deg);
    publishDebugMarkers(Ld, pw, pd, pl, idx_pw, idx_pd, cmd_deg);

    // 8) 스티키 인덱스 갱신
    idx_pw_prev_ = idx_pw;
    idx_pd_prev_ = idx_pd;
  }

  // ---------- Core helpers ----------
  double computeLd() const {
    double Ld = use_speed_term_ ? (L0_ + k_v_ * std::max(0.0, v_filt_)) : L0_;
    if (use_curv_term_ && k_curv_ != 0.0) {
      const double kappa = evalCurvatureAbsAt(curv_window_m_);
      if (std::isfinite(kappa)) {
        Ld += k_curv_ / (std::abs(kappa) + eps_kappa_);
      }
    }
    return clip(Ld, Ld_min_, Ld_max_);
  }

  int findNearestIndexSticky() const {
    if (pts_.empty()) return -1;
    // 최초는 전체 검색
    if (idx_pw_prev_ < 0) {
      double best = 1e100; int idx=-1;
      for (size_t i=0;i<pts_.size();++i){
        const double d = std::hypot(pts_[i].x, pts_[i].y);
        if (x_forward_only_ && pts_[i].x <= forward_margin_x_) continue;
        if (d < best){ best = d; idx = (int)i; }
      }
      if (idx < 0) { // 전방 없으면 전체에서
        for (size_t i=0;i<pts_.size();++i){
          const double d = std::hypot(pts_[i].x, pts_[i].y);
          if (d < best){ best = d; idx = (int)i; }
        }
      }
      return idx;
    }
    // 스티키 윈도우 내 검색
    int L = std::max(0, idx_pw_prev_ - sticky_window_pts_);
    int R = std::min((int)pts_.size()-1, idx_pw_prev_ + sticky_window_pts_);
    double best=1e100; int idx=-1;
    for (int i=L; i<=R; ++i){
      if (x_forward_only_ && pts_[(size_t)i].x <= forward_margin_x_) continue;
      const double d = std::hypot(pts_[(size_t)i].x, pts_[(size_t)i].y);
      if (d < best){ best = d; idx = i; }
    }
    if (idx < 0) { // 윈도우 내 전방 없으면 cand 유지
      for (int i=L; i<=R; ++i){
        const double d = std::hypot(pts_[(size_t)i].x, pts_[(size_t)i].y);
        if (d < best){ best = d; idx = i; }
      }
    }
    return idx;
  }

  int selectTargetIndexSticky(int idx_start, double Ld) const {
    const int N = static_cast<int>(pts_.size());
    if (N<=0) return -1;

    auto cand_from_arc = [&](){
      if (!use_arc_length_selection_ || cum_s_.empty()) return -1;
      const double s0 = cum_s_[static_cast<size_t>(idx_start)];
      const double s_target = s0 + Ld;
      auto it = std::lower_bound(cum_s_.begin()+idx_start, cum_s_.end(), s_target);
      if (it == cum_s_.end()) return N-1;
      return static_cast<int>(std::distance(cum_s_.begin(), it));
    };

    auto cand_from_euclid = [&](){
      for (int i=idx_start; i<N; ++i) {
        const auto& p = pts_[(size_t)i];
        if (x_forward_only_ && p.x <= forward_margin_x_) continue;
        if (std::hypot(p.x, p.y) >= Ld) return i;
      }
      for (int i=N-1; i>=0; --i) {
        if (!x_forward_only_ || pts_[(size_t)i].x > forward_margin_x_) return i;
      }
      return N-1;
    };

    int cand = (use_arc_length_selection_) ? cand_from_arc() : cand_from_euclid();
    // 전방 조건 보정
    if (x_forward_only_ && cand >= 0 && pts_[(size_t)cand].x <= forward_margin_x_) {
      for (int i=cand; i<N; ++i) {
        if (pts_[(size_t)i].x > forward_margin_x_) { cand = i; break; }
      }
    }

    // 스티키 범위 클램프
    if (idx_pd_prev_ >= 0) {
      const int L = std::max(0, idx_pd_prev_ - sticky_window_pts_);
      const int R = std::min(N-1, idx_pd_prev_ + sticky_window_pts_);
      cand = std::clamp(cand, L, R);
    }
    return cand;
  }

  // ----- 곡률 계산(스무딩) -----
  double kappaSignedRawAt(int j) const {
    // 3점법, 부호 유지
    size_t i = static_cast<size_t>(std::clamp(j, 1, (int)pts_.size()-2));
    const auto &A = pts_[i-1], &B = pts_[i], &C = pts_[i+1];
    const double ax = B.x - A.x, ay = B.y - A.y;
    const double cx = C.x - A.x, cy = C.y - A.y;
    const double a = std::hypot(ax, ay);
    const double b = std::hypot(C.x - B.x, C.y - B.y);
    const double c = std::hypot(cx, cy);
    const double area2_signed = (ax * cy - ay * cx);
    const double denom = std::max(1e-9, a*b*c);
    return area2_signed / denom;
  }

  double signedKappaSmoothedAt(int j, int W) const {
    if (pts_.size()<3) return 0.0;
    int L = std::max(1, j-W), R = std::min((int)pts_.size()-2, j+W);
    double sum=0.0; int cnt=0;
    for (int i=L;i<=R;++i) { sum += kappaSignedRawAt(i); ++cnt; }
    return (cnt>0)? (sum/cnt) : 0.0;
  }

  double absKappaSmoothedAt(int j, int W) const {
    if (pts_.size()<3) return 0.0;
    int L = std::max(1, j-W), R = std::min((int)pts_.size()-2, j+W);
    double sum=0.0; int cnt=0;
    for (int i=L;i<=R;++i) { sum += std::abs(kappaSignedRawAt(i)); ++cnt; }
    return (cnt>0)? (sum/cnt) : 0.0;
  }

  // s_eval (m) 부근의 |κ| (스무딩)
  double evalCurvatureAbsAt(double s_eval) const {
    if (pts_.size() < 3 || cum_s_.empty()) return 0.0;
    auto it = std::lower_bound(cum_s_.begin(), cum_s_.end(), s_eval);
    int j = (it == cum_s_.end()) ? ((int)cum_s_.size()-1) : (int)(it - cum_s_.begin());
    j = std::clamp(j, 1, (int)pts_.size()-2);
    return absKappaSmoothedAt(j, kappa_smooth_window_pts_);
  }

  Pt tangentAt(int idx) const {
    size_t j = static_cast<size_t>(std::clamp(idx, 1, (int)pts_.size()-2));
    const auto &A = pts_[j-1];
    const auto &C = pts_[j+1];
    return { C.x - A.x, C.y - A.y };
  }

  // ---------- Command shaping ----------
  double smoothDeg(double raw_deg) {
    const rclcpp::Time now = now_();
    const double dt = (last_cmd_time_.nanoseconds() == 0)
                      ? (1.0 / std::max(1e-3, publish_rate_hz_))
                      : (now - last_cmd_time_).seconds();
    last_cmd_time_ = now;

    // rate limit
    const double dmax = steer_rate_limit_deg_per_s_ * std::max(1e-3, dt);
    double limited = prev_cmd_deg_ + std::clamp(raw_deg - prev_cmd_deg_, -dmax, dmax);

    // EMA
    const double tau = std::max(1e-3, ema_tau_cmd_);
    const double alpha = std::exp(-std::max(0.0, dt) / tau);
    double smoothed = alpha * prev_cmd_deg_ + (1.0 - alpha) * limited;

    smoothed = clip(smoothed, -steer_limit_deg_, steer_limit_deg_);
    prev_cmd_deg_ = smoothed;
    return smoothed;
  }

  void publishSteerDeg(double steer_deg) {
    std_msgs::msg::Float32 out;
    out.data = static_cast<float>(steer_deg);
    pub_steer_->publish(out);
  }

  // ---------- Debug markers ----------
  void publishDebugMarkersEmpty() {
    visualization_msgs::msg::MarkerArray arr;
    pub_debug_->publish(arr);
  }

  void publishDebugMarkers(double Ld, const Pt& pw, const Pt& pd, const Pt& pl,
                           int idx_pw, int idx_pd, double cmd_deg) {
    visualization_msgs::msg::MarkerArray arr;
    const rclcpp::Time tnow = this->now_();
    auto make_color = [&](const std::array<double,3>& c, double a){
      std_msgs::msg::ColorRGBA col; col.r=c[0]; col.g=c[1]; col.b=c[2]; col.a=a; return col;
    };
    const std::string frame = frame_id_from_path_.empty()? base_frame_ : frame_id_from_path_;

    // 0) Look-ahead circle
    if (show_lookahead_circle_) {
      visualization_msgs::msg::Marker cir;
      cir.header.frame_id = frame; cir.header.stamp = tnow;
      cir.ns = "pp/lookahead_circle"; cir.id = 0;
      cir.type = visualization_msgs::msg::Marker::LINE_STRIP;
      cir.action = visualization_msgs::msg::Marker::ADD;
      cir.scale.x = 0.02;
      cir.color = make_color(color_cir_, 0.6);
      cir.pose.orientation.w = 1.0;
      const int N = std::max(16, circle_pts_);
      cir.points.resize(N+1);
      for (int i=0;i<=N;++i){
        const double th = (2.0*kPi*i)/N;
        geometry_msgs::msg::Point p; p.x = Ld*std::cos(th); p.y = Ld*std::sin(th); p.z = 0.02;
        cir.points[i] = p;
      }
      arr.markers.push_back(cir);
    }

    // 1) pw, pd, pl
    if (show_points_) {
      arr.markers.push_back(makeSphere("pp/pw", 0, pw, 0.25, make_color(color_pw_, marker_alpha_), "pw"));
      arr.markers.push_back(makeSphere("pp/pd", 0, pd, 0.30, make_color(color_pd_, marker_alpha_), "pd"));
      arr.markers.push_back(makeSphere("pp/pl", 0, pl, 0.40, make_color(color_pl_, marker_alpha_), "pl"));
    }

    // 2) tangent at pd
    if (show_tangent_) {
      Pt t = tangentAt(idx_pd);
      const double nrm = std::hypot(t.x, t.y);
      if (nrm > 1e-6) { t.x/=nrm; t.y/=nrm; }
      arr.markers.push_back(
        makeArrow("pp/tangent_at_pd", 0, pd, t, 0.9, 0.05, make_color(color_tan_, marker_alpha_))
      );
    }

    // 3) pd -> pl 오프셋 벡터
    if (show_offset_vec_) {
      visualization_msgs::msg::Marker line;
      line.header.frame_id = frame; line.header.stamp = tnow;
      line.ns = "pp/pd_to_pl"; line.id = 0;
      line.type = visualization_msgs::msg::Marker::LINE_LIST;
      line.action = visualization_msgs::msg::Marker::ADD;
      line.scale.x = 0.03;
      line.color = make_color(color_vec_, marker_alpha_);
      line.pose.orientation.w = 1.0;
      geometry_msgs::msg::Point p1, p2; 
      p1.x=pd.x; p1.y=pd.y; p1.z=0.05; 
      p2.x=pl.x; p2.y=pl.y; p2.z=0.05;
      line.points.push_back(p1); line.points.push_back(p2);
      arr.markers.push_back(line);
    }

    // 4) 선택 구간 하이라이트(옵션)
    if (show_selected_path_) {
      visualization_msgs::msg::Marker seg;
      seg.header.frame_id = frame; seg.header.stamp = tnow;
      seg.ns = "pp/selected_path"; seg.id = 0;
      seg.type = visualization_msgs::msg::Marker::LINE_STRIP;
      seg.action = visualization_msgs::msg::Marker::ADD;
      seg.scale.x = 0.02;
      seg.color = make_color({0.7,0.7,0.7}, 0.6);
      seg.pose.orientation.w = 1.0;
      int i0 = std::max(0, idx_pw-30), i1 = std::min((int)pts_.size()-1, idx_pd+30);
      seg.points.reserve(i1-i0+1);
      for (int i=i0;i<=i1;++i){
        geometry_msgs::msg::Point p; p.x = pts_[i].x; p.y = pts_[i].y; p.z = 0.02;
        seg.points.push_back(p);
      }
      arr.markers.push_back(seg);
    }

    // 5) 스티어링 화살표 + 텍스트
    if (show_steer_arrow_ || show_steer_text_) {
      const double yaw = cmd_deg * kPi / 180.0;
      Pt origin = { wheelbase_m_, 0.0 };
      Pt dir = { std::cos(yaw), std::sin(yaw) };
      if (show_steer_arrow_) {
        arr.markers.push_back(
          makeArrow("pp/steer_arrow", 0, origin, dir, 4.0, 0.06, make_color({1.0,0.5,0.0}, 1.0))
        );
      }
      if (show_steer_text_) {
        visualization_msgs::msg::Marker txt;
        txt.header.frame_id = frame; txt.header.stamp = tnow;
        txt.ns = "pp/steer_text"; txt.id = 0;
        txt.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
        txt.action = visualization_msgs::msg::Marker::ADD;
        txt.scale.z = 0.25;
        txt.color = make_color({1.0,1.0,1.0}, 1.0);
        txt.pose.position.x = origin.x;
        txt.pose.position.y = origin.y;
        txt.pose.position.z = 0.5;
        txt.pose.orientation.w = 1.0;
        char buf[64]; std::snprintf(buf, sizeof(buf), "%+.1f deg", cmd_deg);
        txt.text = std::string(buf);
        arr.markers.push_back(txt);
      }
    }

    pub_debug_->publish(arr);
  }

  visualization_msgs::msg::Marker makeSphere(const std::string& ns, int id, const Pt& p,
                                              double scale, const std_msgs::msg::ColorRGBA& col,
                                              const std::string& /*label*/) {
    visualization_msgs::msg::Marker m;
    m.header.frame_id = frame_id_from_path_.empty()? base_frame_ : frame_id_from_path_;
    m.header.stamp = now_();
    m.ns = ns; m.id = id; m.type = visualization_msgs::msg::Marker::SPHERE;
    m.action = visualization_msgs::msg::Marker::ADD;
    m.pose.position.x = p.x; m.pose.position.y = p.y; m.pose.position.z = 0.05;
    m.pose.orientation.w = 1.0;
    m.scale.x = scale; m.scale.y = scale; m.scale.z = scale;
    m.color = col;
    return m;
  }

  visualization_msgs::msg::Marker makeArrow(const std::string& ns, int id,
                                            const Pt& origin, const Pt& dir_unit,
                                            double length, double thickness,
                                            const std_msgs::msg::ColorRGBA& col) {
    visualization_msgs::msg::Marker a;
    a.header.frame_id = frame_id_from_path_.empty()? base_frame_ : frame_id_from_path_;
    a.header.stamp = now_();
    a.ns = ns; a.id = id; a.type = visualization_msgs::msg::Marker::ARROW;
    a.action = visualization_msgs::msg::Marker::ADD;

    geometry_msgs::msg::Point p0, p1;
    p0.x = origin.x; p0.y = origin.y; p0.z = 0.2;
    p1.x = origin.x + dir_unit.x * length; p1.y = origin.y + dir_unit.y * length; p1.z = 0.2;
    a.points = {p0, p1};

    a.scale.x = thickness;           // shaft diameter
    a.scale.y = thickness * 1.8;     // head diameter
    a.scale.z = thickness * 2.8;     // head length
    a.color = col;
    a.pose.orientation.w = 1.0;
    return a;
  }

  // ---------- Utils ----------
  inline double clip(double x, double lo, double hi) const { return std::max(lo, std::min(hi, x)); }
  rclcpp::Time now_() const { return this->now(); } // const에서 안전

  // ---------- Params ----------
  std::string path_topic_, speed_topic_, steer_topic_, debug_topic_, base_frame_;
  std::string frame_id_from_path_{};

  double wheelbase_m_{1.3};

  bool   use_speed_term_{true};
  double L0_{1.5}, k_v_{0.6};
  bool   use_curv_term_{true};
  double k_curv_{0.0}, eps_kappa_{1e-6};
  double Ld_min_{1.0}, Ld_max_{6.0};

  bool   use_arc_length_selection_{true};
  bool   x_forward_only_{true};
  double forward_margin_x_{-0.2};

  bool   outer_offset_enable_{true};
  double alpha_max_m_{3.0};
  double beta_max_{3.0};
  double curv_window_m_{2.0};
  double outer_offset_max_m_{1.0};
  double outer_offset_tau_max_{0.7};
  double outer_offset_kappa_gate_{0.03};
  double track_half_width_m_{0.0};
  double track_margin_m_{0.2};

  double publish_rate_hz_{50.0};
  double steer_limit_deg_{30.0};
  double steer_rate_limit_deg_per_s_{360.0};
  double ema_tau_cmd_{0.12};
  double ema_tau_speed_{0.20};

  // 노이즈 억제(추가)
  int    sticky_window_pts_{15};
  int    kappa_smooth_window_pts_{3};
  double target_ema_tau_{0.08};

  bool   show_lookahead_circle_{true};
  bool   show_points_{true};
  bool   show_tangent_{true};
  bool   show_offset_vec_{true};
  bool   show_selected_path_{false};
  bool   show_steer_arrow_{true};
  bool   show_steer_text_{true};
  double marker_alpha_{1.0};
  int    circle_pts_{60};

  std::array<double,3> color_pw_;
  std::array<double,3> color_pd_;
  std::array<double,3> color_pl_;
  std::array<double,3> color_tan_;
  std::array<double,3> color_vec_;
  std::array<double,3> color_cir_;

  // ---------- State ----------
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr sub_path_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr sub_speed_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr pub_steer_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_debug_;
  rclcpp::TimerBase::SharedPtr timer_;

  nav_msgs::msg::Path last_path_;
  std::vector<Pt> pts_;
  std::vector<double> cum_s_;

  bool have_speed_{false};
  double v_filt_{0.0};
  rclcpp::Time last_speed_time_{};

  rclcpp::Time last_cmd_time_{};
  double prev_cmd_deg_{0.0};

  int idx_pw_prev_{-1}, idx_pd_prev_{-1};

  bool have_pl_prev_{false};
  Pt pl_prev_{};
  rclcpp::Time last_target_time_{};
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PurePursuitAdaptive>());
  rclcpp::shutdown();
  return 0;
}
