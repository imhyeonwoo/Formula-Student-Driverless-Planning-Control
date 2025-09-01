#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32.hpp>
#include <nav_msgs/msg/path.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>

#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <vector>
#include <string>
#include <algorithm>
#include <cmath>
#include <chrono>
#include <array>
#include <limits>

class PurePursuitPredictive : public rclcpp::Node {
public:
  PurePursuitPredictive()
  : rclcpp::Node("pure_pursuit_predictive"),
    tf_buffer_(this->get_clock()),
    tf_listener_(tf_buffer_) {

    // ----- Parameters -----
    path_topic_  = declare_parameter<std::string>("path_topic", "/local_planned_path");
    speed_topic_ = declare_parameter<std::string>("speed_topic", "/current_speed");
    steer_topic_ = declare_parameter<std::string>("steer_topic", "/cmd/steer_predictive");
    debug_topic_ = declare_parameter<std::string>("debug_marker_topic", "/pure_pursuit/predictive_debug");
    base_frame_  = declare_parameter<std::string>("base_frame", "base_link");

    wheelbase_m_ = declare_parameter<double>("wheelbase_m", 1.3);

    // Prediction
    predict_enable_     = declare_parameter<bool>("predict_enable", true);
    T_pred_sec_         = declare_parameter<double>("T_pred_sec", 0.18);
    use_cmd_curv_       = declare_parameter<bool>("use_cmd_curvature", true);

    // Ld scheduling (time-based + curvature term)
    use_time_term_      = declare_parameter<bool>("use_time_term", true);
    T_LA_               = declare_parameter<double>("T_LA", 0.60);
    L_bias_             = declare_parameter<double>("L_bias", 0.9);

    use_speed_term_     = declare_parameter<bool>("use_speed_term", false);
    L0_                 = declare_parameter<double>("L0", 2.5);
    k_v_                = declare_parameter<double>("k_v", 0.6);

    use_curv_term_      = declare_parameter<bool>("use_curvature_term", true);
    k_curv_             = declare_parameter<double>("k_curv", 0.6);
    eps_kappa_          = declare_parameter<double>("epsilon_kappa", 1e-6);
    Ld_min_             = declare_parameter<double>("Ld_min", 1.0);
    Ld_max_             = declare_parameter<double>("Ld_max", 7.0);

    // Hysteresis / Sticky (meter-based)
    s_back_hys_m_       = declare_parameter<double>("s_back_hys_m", 0.8);
    allow_backtrack_heading_deg_ = declare_parameter<double>("allow_backtrack_heading_deg", 135.0);
    sticky_window_m_    = declare_parameter<double>("sticky_window_m", 1.2);

    // Outer-offset
    outer_offset_enable_    = declare_parameter<bool>("outer_offset_enable", true);
    alpha_max_m_            = declare_parameter<double>("alpha_max_m", 3.0);
    beta_max_               = declare_parameter<double>("beta_max", 3.0);
    curv_window_m_          = declare_parameter<double>("curv_window_m", 2.0);
    outer_offset_max_m_     = declare_parameter<double>("outer_offset_max_m", 1.0);
    outer_offset_tau_max_   = declare_parameter<double>("outer_offset_tau_max", 0.7);
    outer_offset_kappa_gate_= declare_parameter<double>("outer_offset_kappa_gate", 0.03);
    track_half_width_m_     = declare_parameter<double>("track_half_width_m", 2.5);
    track_margin_m_         = declare_parameter<double>("track_margin_m", 0.2);

    // Command shaping
    publish_rate_hz_    = declare_parameter<double>("publish_rate_hz", 50.0);
    steer_limit_deg_    = declare_parameter<double>("steer_limit_deg", 30.0);
    steer_rate_limit_deg_per_s_ = declare_parameter<double>("steer_rate_limit_deg_per_s", 360.0);
    ema_tau_cmd_        = declare_parameter<double>("ema_tau_cmd", 0.12);
    ema_tau_speed_      = declare_parameter<double>("ema_tau_speed", 0.20);
    target_ema_tau_     = declare_parameter<double>("target_ema_tau", 0.08);
    kappa_smooth_window_pts_ = declare_parameter<int>("kappa_smooth_window_pts", 3);

    // Debug
    show_lookahead_circle_ = declare_parameter<bool>("show_lookahead_circle", true);
    show_points_           = declare_parameter<bool>("show_points", true);
    show_tangent_          = declare_parameter<bool>("show_tangent", true);
    show_offset_vec_       = declare_parameter<bool>("show_offset_vec", true);
    show_selected_path_    = declare_parameter<bool>("show_selected_path", false);
    show_steer_arrow_      = declare_parameter<bool>("show_steer_arrow", true);
    show_steer_text_       = declare_parameter<bool>("show_steer_text", false);
    marker_alpha_          = declare_parameter<double>("marker_alpha", 1.0);
    circle_pts_            = declare_parameter<int>("circle_points", 60);

    // Colors
    color_pw_ = { param("color_pw_r",0.10), param("color_pw_g",0.60), param("color_pw_b",1.00) };
    color_pd_ = { param("color_pd_r",0.10), param("color_pd_g",1.00), param("color_pd_b",0.10) };
    color_pl_ = { param("color_pl_r",1.00), param("color_pl_g",0.10), param("color_pl_b",0.80) };
    color_tan_={ param("color_tan_r",1.00), param("color_tan_g",0.85), param("color_tan_b",0.10) };
    color_vec_={ param("color_vec_r",0.00), param("color_vec_g",0.90), param("color_vec_b",1.00) };
    color_cir_={ param("color_cir_r",1.00), param("color_cir_g",1.00), param("color_cir_b",1.00) };

    // Pub/Sub
    sub_path_ = create_subscription<nav_msgs::msg::Path>(
      path_topic_, rclcpp::QoS(10),
      std::bind(&PurePursuitPredictive::onPath, this, std::placeholders::_1));

    sub_speed_ = create_subscription<std_msgs::msg::Float32>(
      speed_topic_, rclcpp::SensorDataQoS(),
      std::bind(&PurePursuitPredictive::onSpeed, this, std::placeholders::_1));

    pub_steer_ = create_publisher<std_msgs::msg::Float32>(steer_topic_, 10);
    pub_debug_ = create_publisher<visualization_msgs::msg::MarkerArray>(debug_topic_, 10);

    // Timer
    using namespace std::chrono_literals;
    const auto period = std::chrono::duration<double>(1.0 / std::max(1e-3, publish_rate_hz_));
    timer_ = create_wall_timer(std::chrono::duration_cast<std::chrono::milliseconds>(period),
              std::bind(&PurePursuitPredictive::onTimer, this));

    RCLCPP_INFO(get_logger(), "PurePursuitPredictive ready. base_frame='%s', steer='%s'",
                base_frame_.c_str(), steer_topic_.c_str());
  }

private:
  struct Pt { double x, y; };
  static constexpr double kPi = 3.14159265358979323846;

  inline double param(const char* key, double def){ return declare_parameter<double>(key, def); }
  inline double clip(double x,double lo,double hi) const { return std::max(lo,std::min(hi,x)); }
  rclcpp::Time now_() const { return this->now(); }

  // --------- Subscriptions ---------
  void onPath(const nav_msgs::msg::Path::SharedPtr msg) {
    nav_msgs::msg::Path path_in_base;

    if (!msg->header.frame_id.empty() && msg->header.frame_id != base_frame_) {
      try {
        geometry_msgs::msg::TransformStamped tf =
          tf_buffer_.lookupTransform(base_frame_, msg->header.frame_id, tf2::TimePointZero);

        path_in_base.header.frame_id = base_frame_;
        path_in_base.header.stamp = msg->header.stamp;
        path_in_base.poses.reserve(msg->poses.size());

        for (const auto& ps : msg->poses) {
          geometry_msgs::msg::PoseStamped out;
          tf2::doTransform(ps, out, tf);
          path_in_base.poses.push_back(out);
        }
      } catch (const tf2::TransformException& ex) {
        RCLCPP_WARN(get_logger(), "TF transform failed: %s. Using raw path.", ex.what());
        path_in_base = *msg;
      }
    } else {
      path_in_base = *msg;
    }

    last_path_ = path_in_base;
    frame_id_from_path_ = base_frame_;

    // pts_/cum_s_ 갱신
    pts_.clear();
    pts_.reserve(last_path_.poses.size());
    for (const auto& ps : last_path_.poses) {
      pts_.push_back({ (double)ps.pose.position.x, (double)ps.pose.position.y });
    }

    cum_s_.assign(pts_.size(), 0.0);
    for (size_t i=1;i<pts_.size();++i){
      const double ds = std::hypot(pts_[i].x-pts_[i-1].x, pts_[i].y-pts_[i-1].y);
      cum_s_[i] = cum_s_[i-1] + ds;
    }
  }

  void onSpeed(const std_msgs::msg::Float32::SharedPtr m) {
    const rclcpp::Time t = now_();
    const double meas = (double)m->data;
    if (!have_speed_) {
      v_filt_ = meas; have_speed_ = true;
    } else {
      const double dt = (t - last_speed_time_).seconds();
      const double alpha = std::exp(-std::max(0.0, dt) / std::max(1e-3, ema_tau_speed_));
      v_filt_ = alpha * v_filt_ + (1.0 - alpha) * meas;
    }
    last_speed_time_ = t;
  }

  // --------- Timer loop ---------
  void onTimer() {
    if (pts_.size() < 2) { publishSteerDeg(smoothDeg(0.0)); publishDebugMarkersEmpty(); return; }

    // 1) 예측 기준점 계산 (base_link 좌표계)
    Pt P = {0.0, 0.0}; // base_link
    if (predict_enable_) {
      const double v = std::max(0.0, v_filt_);
      const double T = std::max(0.0, T_pred_sec_);
      double kappa = 0.0;
      if (use_cmd_curv_) {
        const double delta_rad = prev_cmd_deg_ * kPi / 180.0;
        kappa = std::tan(delta_rad) / std::max(1e-6, wheelbase_m_);
      }
      const double dtheta = v * kappa * T;
      if (std::abs(kappa) < 1e-6) {
        P.x = v * T; P.y = 0.0;
      } else {
        P.x = std::sin(dtheta) / kappa;
        P.y = (1.0 - std::cos(dtheta)) / kappa;
      }
    }

    // 2) 경로에 연속 투영 → s0
    Projection prj = projectPointToPath(P);
    if (prj.seg_idx < 0) { publishSteerDeg(smoothDeg(0.0)); publishDebugMarkersEmpty(); return; }
    const double s0 = prj.s_proj;

    // 3) Ld 계산 (s0 근방 곡률 사용)
    const double Ld = computeLd(s0);

    // 4) s_target = s0 + Ld  (히스테리시스 + 스티키 m 적용)
    double s_target = s0 + Ld;
    if (std::isfinite(s_pd_prev_)) {
      const double lo = s_pd_prev_ - sticky_window_m_;
      const double hi = s_pd_prev_ + sticky_window_m_;
      s_target = std::clamp(s_target, lo, hi);

      if (s_target < s_pd_prev_ - s_back_hys_m_) {
        Pt t = tangentAtS(s0);
        const double heading = std::atan2(t.y, t.x); // 차량 헤딩=0
        const double thresh = allow_backtrack_heading_deg_ * kPi / 180.0;
        if (std::abs(heading) < thresh) {
          s_target = s_pd_prev_ - s_back_hys_m_;
        }
      }
    }
    s_target = std::clamp(s_target, 0.0, cum_s_.empty()? 0.0 : cum_s_.back());
    s_pd_prev_ = s_target;

    // 5) pd / pw / pl
    Pt pd = pointAtS(s_target);
    Pt pw_proj = pointAtS(s0);   // 투영점(가중/α 계산용)
    Pt pw = {0.0, 0.0};          // 표시/개념상 pw는 base_link
    Pt pl = pd;

    // --- Outer-offset (커팅 억제) ---
    if (outer_offset_enable_) {
      const int idx_pw = indexAtS(s0);
      const int idx_pd = indexAtS(s_target);
      const double k_pw = signedKappaSmoothedAt(idx_pw, kappa_smooth_window_pts_);
      const double k_pd = signedKappaSmoothedAt(idx_pd, kappa_smooth_window_pts_);

      // α: 차량-경로 근접도(투영점 거리 기준) — base_link를 쓰면 α=0 고정 → 과도해짐을 방지
      const double alpha = std::min(1.0, std::hypot(pw_proj.x, pw_proj.y) / std::max(1e-6, alpha_max_m_));

      // β: 곡률 비
      double beta = 0.0;
      if (std::abs(k_pw) > 1e-12 && std::abs(k_pd) > std::abs(k_pw)) {
        beta = std::min(1.0, (std::abs(k_pd) / std::abs(k_pw)) / std::max(1e-6, beta_max_));
      }

      double tau = (1.0 - alpha) * beta;
      tau = std::clamp(tau, 0.0, outer_offset_tau_max_);
      if (std::abs(k_pd) < outer_offset_kappa_gate_) tau = 0.0;

      // pd에서의 접선/법선 방향(단위)
      Pt t = tangentAtS(s_target);
      const double nrm = std::hypot(t.x,t.y);
      if (nrm > 1e-6) { t.x/=nrm; t.y/=nrm; } else { t = {1.0,0.0}; }
      Pt n = { -t.y, t.x };                         // 좌측 법선
      const double sign_k = (k_pd >= 0.0) ? +1.0 : -1.0; // 좌회전(+): 안쪽은 +n
      Pt outward = { -sign_k * n.x, -sign_k * n.y };     // 바깥 = -sign(k)*n

      // ⬇️ 요청 사항: pl은 base_link 원점에서 'pd의 법선 방향'으로 이동시킨 점
      const double d_scale = std::hypot(pd.x - pw.x, pd.y - pw.y);  // |pd - base_link|
      double off = std::min(tau * d_scale, outer_offset_max_m_);
      if (track_half_width_m_ > 0.0) {
        off = std::min(off, std::max(0.0, track_half_width_m_ - track_margin_m_));
      }
      pl = { pw.x + outward.x * off, pw.y + outward.y * off };
    }

    // 6) 타깃 EMA
    {
      const rclcpp::Time t = now_();
      const double dt = (last_target_time_.nanoseconds()==0)
                        ? (1.0 / std::max(1e-3, publish_rate_hz_))
                        : (t - last_target_time_).seconds();
      last_target_time_ = t;
      const double a = std::exp(-std::max(0.0, dt) / std::max(1e-3, target_ema_tau_));
      if (!have_pl_prev_) { pl_prev_ = pl; have_pl_prev_ = true; }
      pl = { a*pl_prev_.x + (1.0 - a)*pl.x, a*pl_prev_.y + (1.0 - a)*pl.y };
      pl_prev_ = pl;
    }

    // 7) 조향 계산 (deg)
    const double Ld2 = std::max(1e-9, pl.x*pl.x + pl.y*pl.y);
    const double delta_rad = std::atan2(2.0 * wheelbase_m_ * pl.y, Ld2);
    double delta_deg = delta_rad * 180.0 / kPi;

    // 8) rate-limit + EMA + 제한
    delta_deg = clip(delta_deg, -steer_limit_deg_, steer_limit_deg_);
    const double cmd_deg = smoothDeg(delta_deg);

    // 9) Publish & Debug
    publishSteerDeg(cmd_deg);
    publishDebugMarkers(Ld, pw /* base_link */, pd, pl, indexAtS(s0), indexAtS(s_target), cmd_deg);
  }

  // --------- Geometry helpers ---------
  struct Projection { int seg_idx; double t; double s_proj; Pt p; };

  Projection projectPointToPath(const Pt& O) const {
    Projection best{ -1, 0.0, 0.0, {0,0} };
    if (pts_.size() < 2) return best;

    double best_d2 = 1e100;
    for (int i=0; i<(int)pts_.size()-1; ++i) {
      Pt A=pts_[i], B=pts_[i+1], AB{B.x-A.x, B.y-A.y};
      const double ab2 = AB.x*AB.x + AB.y*AB.y; if (ab2 < 1e-12) continue;
      Pt AO{ O.x - A.x, O.y - A.y };
      double t = (AO.x*AB.x + AO.y*AB.y) / ab2; t = std::clamp(t,0.0,1.0);
      Pt P{ A.x + t*AB.x, A.y + t*AB.y };
      const double d2 = (P.x-O.x)*(P.x-O.x) + (P.y-O.y)*(P.y-O.y);
      if (d2 < best_d2) {
        best_d2 = d2; best.seg_idx = i; best.t = t; best.p = P;
        const double seg_len = std::hypot(AB.x, AB.y);
        best.s_proj = cum_s_[i] + t * seg_len;
      }
    }
    return best;
  }

  int indexAtS(double s) const {
    if (cum_s_.empty()) return -1;
    if (s <= cum_s_.front()) return 0;
    if (s >= cum_s_.back())  return (int)cum_s_.size()-1;
    auto it = std::lower_bound(cum_s_.begin(), cum_s_.end(), s);
    return (int)std::distance(cum_s_.begin(), it);
  }

  Pt pointAtS(double s) const {
    Pt out{0.0,0.0};
    if (pts_.empty() || cum_s_.size()!=pts_.size()) return out;
    if (s <= cum_s_.front()) return pts_.front();
    if (s >= cum_s_.back())  return pts_.back();
    auto it = std::lower_bound(cum_s_.begin(), cum_s_.end(), s);
    int idx = (int)std::distance(cum_s_.begin(), it);
    int i0 = idx-1, i1 = idx;
    const double s0=cum_s_[i0], s1=cum_s_[i1];
    const double t = (s - s0) / std::max(1e-9, s1 - s0);
    Pt A=pts_[i0], B=pts_[i1];
    return { A.x + t*(B.x-A.x), A.y + t*(B.y-A.y) };
  }

  Pt tangentAtS(double s) const {
    if (pts_.size()<2) return {1.0,0.0};
    if (s <= cum_s_.front()) return { pts_[1].x-pts_[0].x, pts_[1].y-pts_[0].y };
    if (s >= cum_s_.back())  { size_t N=pts_.size(); return { pts_[N-1].x-pts_[N-2].x, pts_[N-1].y-pts_[N-2].y }; }
    auto it = std::lower_bound(cum_s_.begin(), cum_s_.end(), s);
    int idx = (int)std::distance(cum_s_.begin(), it);
    int i0 = std::max(0, idx-1), i1 = std::min((int)pts_.size()-1, idx);
    return { pts_[i1].x-pts_[i0].x, pts_[i1].y-pts_[i0].y };
  }

  // ---- Curvature ----
  double kappaSignedRawAt(int j) const {
    size_t i = (size_t)std::clamp(j, 1, (int)pts_.size()-2);
    const auto &A=pts_[i-1], &B=pts_[i], &C=pts_[i+1];
    const double ax=B.x-A.x, ay=B.y-A.y;
    const double cx=C.x-A.x, cy=C.y-A.y;
    const double a=std::hypot(ax,ay), b=std::hypot(C.x-B.x,C.y-B.y), c=std::hypot(cx,cy);
    const double area2 = (ax*cy - ay*cx);
    const double denom = std::max(1e-9, a*b*c);
    return area2 / denom; // 부호 포함
  }

  double signedKappaSmoothedAt(int j, int W) const {
    if (pts_.size()<3) return 0.0;
    int L=std::max(1, j-W), R=std::min((int)pts_.size()-2, j+W);
    double sum=0.0; int cnt=0;
    for (int i=L;i<=R;++i){ sum += kappaSignedRawAt(i); ++cnt; }
    return (cnt>0)? (sum/cnt) : 0.0;
  }

  double absKappaSmoothedAt(int j, int W) const {
    if (pts_.size()<3) return 0.0;
    int L=std::max(1, j-W), R=std::min((int)pts_.size()-2, j+W);
    double sum=0.0; int cnt=0;
    for (int i=L;i<=R;++i){ sum += std::abs(kappaSignedRawAt(i)); ++cnt; }
    return (cnt>0)? (sum/cnt) : 0.0;
  }

  double evalCurvatureAbsNearS(double s_ref) const {
    if (pts_.size()<3 || cum_s_.empty()) return 0.0;
    int j = indexAtS(s_ref);
    j = std::clamp(j, 1, (int)pts_.size()-2);
    return absKappaSmoothedAt(j, kappa_smooth_window_pts_);
  }

  double computeLd(double s_ref) const {
    double Ld = 0.0;
    if (use_time_term_)   Ld += std::max(0.0, v_filt_) * std::max(0.0, T_LA_) + L_bias_;
    if (use_speed_term_)  Ld += L0_ + k_v_ * std::max(0.0, v_filt_);
    if (use_curv_term_) {
      const double kappa = evalCurvatureAbsNearS(s_ref);
      Ld += k_curv_ / (std::abs(kappa) + eps_kappa_);
    }
    if (!use_time_term_ && !use_speed_term_) { Ld += L0_; }
    return clip(Ld, Ld_min_, Ld_max_);
  }

  // ---- Command shaping ----
  double smoothDeg(double raw_deg) {
    const rclcpp::Time t = now_();
    const double dt = (last_cmd_time_.nanoseconds()==0)
                      ? (1.0 / std::max(1e-3, publish_rate_hz_))
                      : (t - last_cmd_time_).seconds();
    last_cmd_time_ = t;

    // rate limit
    const double dmax = steer_rate_limit_deg_per_s_ * std::max(1e-3, dt);
    double limited = prev_cmd_deg_ + std::clamp(raw_deg - prev_cmd_deg_, -dmax, dmax);

    // EMA
    const double a = std::exp(-std::max(0.0, dt) / std::max(1e-3, ema_tau_cmd_));
    double smoothed = a * prev_cmd_deg_ + (1.0 - a) * limited;

    smoothed = clip(smoothed, -steer_limit_deg_, steer_limit_deg_);
    prev_cmd_deg_ = smoothed;
    return smoothed;
  }

  void publishSteerDeg(double steer_deg) {
    std_msgs::msg::Float32 out; out.data = (float)steer_deg; pub_steer_->publish(out);
  }

  // ---- Debug ----
  void publishDebugMarkersEmpty() {
    visualization_msgs::msg::MarkerArray arr; pub_debug_->publish(arr);
  }

  visualization_msgs::msg::Marker makeSphere(const std::string& ns, int id, const Pt& p,
      double scale, const std_msgs::msg::ColorRGBA& col) {
    visualization_msgs::msg::Marker m;
    m.header.frame_id = frame_id_from_path_.empty()? base_frame_ : frame_id_from_path_;
    m.header.stamp = now_();
    m.ns = ns; m.id = id; m.type = visualization_msgs::msg::Marker::SPHERE;
    m.action = visualization_msgs::msg::Marker::ADD;
    m.pose.position.x = p.x; m.pose.position.y = p.y; m.pose.position.z = 0.05;
    m.pose.orientation.w = 1.0;
    m.scale.x = scale; m.scale.y = scale; m.scale.z = scale;
    m.color = col; return m;
  }

  visualization_msgs::msg::Marker makeArrow(const std::string& ns, int id,
      const Pt& origin, const Pt& dir_unit, double length, double thickness,
      const std_msgs::msg::ColorRGBA& col) {
    visualization_msgs::msg::Marker a;
    a.header.frame_id = frame_id_from_path_.empty()? base_frame_ : frame_id_from_path_;
    a.header.stamp = now_();
    a.ns = ns; a.id = id; a.type = visualization_msgs::msg::Marker::ARROW;
    a.action = visualization_msgs::msg::Marker::ADD;
    geometry_msgs::msg::Point p0, p1;
    p0.x=origin.x; p0.y=origin.y; p0.z=0.2;
    p1.x=origin.x + dir_unit.x*length; p1.y=origin.y + dir_unit.y*length; p1.z=0.2;
    a.points = {p0,p1};
    a.scale.x = thickness; a.scale.y = thickness*1.8; a.scale.z = thickness*2.8;
    a.color = col; a.pose.orientation.w = 1.0; return a;
  }

  void publishDebugMarkers(double Ld, const Pt& pw, const Pt& pd, const Pt& pl,
                           int idx_pw, int idx_pd, double cmd_deg) {
    visualization_msgs::msg::MarkerArray arr;
    const rclcpp::Time tnow = now_();
    auto col = [&](const std::array<double,3>& c, double a){
      std_msgs::msg::ColorRGBA cc; cc.r=c[0];cc.g=c[1];cc.b=c[2];cc.a=a; return cc;
    };
    const std::string frame = frame_id_from_path_.empty()? base_frame_ : frame_id_from_path_;

    if (show_lookahead_circle_) {
      visualization_msgs::msg::Marker cir;
      cir.header.frame_id = frame; cir.header.stamp = tnow;
      cir.ns = "ppx/lookahead_circle"; cir.id = 0;
      cir.type = visualization_msgs::msg::Marker::LINE_STRIP;
      cir.action = visualization_msgs::msg::Marker::ADD;
      cir.scale.x = 0.02; cir.color = col(color_cir_, 0.6); cir.pose.orientation.w = 1.0;
      const int N = std::max(16, circle_pts_);
      cir.points.resize(N+1);
      for (int i=0;i<=N;++i){
        const double th = (2.0*kPi*i)/N;
        geometry_msgs::msg::Point p; p.x = Ld*std::cos(th); p.y = Ld*std::sin(th); p.z = 0.02;
        cir.points[i] = p;
      }
      arr.markers.push_back(cir);
    }

    if (show_points_) {
      arr.markers.push_back(makeSphere("ppx/pw", 0, pw, 0.25, col(color_pw_, marker_alpha_)));
      arr.markers.push_back(makeSphere("ppx/pd", 0, pd, 0.30, col(color_pd_, marker_alpha_)));
      arr.markers.push_back(makeSphere("ppx/pl", 0, pl, 0.40, col(color_pl_, marker_alpha_)));
    }

    if (show_tangent_) {
      Pt t = tangentAtS(cum_s_.empty()? 0.0 : cum_s_[std::clamp(idx_pd, 0, (int)cum_s_.size()-1)]);
      const double nrm = std::hypot(t.x,t.y); if (nrm>1e-6){ t.x/=nrm; t.y/=nrm; }
      arr.markers.push_back(makeArrow("ppx/tan_pd", 0, pd, t, 0.9, 0.05, col(color_tan_, marker_alpha_)));
    }

    if (show_offset_vec_) {
      visualization_msgs::msg::Marker line;
      line.header.frame_id = frame; line.header.stamp = tnow;
      line.ns = "ppx/pd_to_pl"; line.id = 0;
      line.type = visualization_msgs::msg::Marker::LINE_LIST;
      line.action = visualization_msgs::msg::Marker::ADD;
      line.scale.x = 0.03; line.color = col(color_vec_, marker_alpha_); line.pose.orientation.w = 1.0;
      geometry_msgs::msg::Point p1,p2; p1.x=pw.x; p1.y=pw.y; p1.z=0.05; p2.x=pl.x; p2.y=pl.y; p2.z=0.05;
      line.points.push_back(p1); line.points.push_back(p2);
      arr.markers.push_back(line);
    }

    if (show_selected_path_) {
      visualization_msgs::msg::Marker seg;
      seg.header.frame_id = frame; seg.header.stamp = tnow;
      seg.ns = "ppx/selected_path"; seg.id = 0;
      seg.type = visualization_msgs::msg::Marker::LINE_STRIP;
      seg.action = visualization_msgs::msg::Marker::ADD;
      seg.scale.x = 0.02; seg.color = col({0.7,0.7,0.7}, 0.6); seg.pose.orientation.w = 1.0;
      int i0 = std::max(0, idx_pw-30), i1 = std::min((int)pts_.size()-1, idx_pd+30);
      seg.points.reserve(i1-i0+1);
      for (int i=i0;i<=i1;++i){
        geometry_msgs::msg::Point p; p.x=pts_[i].x; p.y=pts_[i].y; p.z=0.02;
        seg.points.push_back(p);
      }
      arr.markers.push_back(seg);
    }

    if (show_steer_arrow_ || show_steer_text_) {
      const double yaw = cmd_deg * kPi / 180.0;
      Pt origin = { wheelbase_m_, 0.0 };
      Pt dir = { std::cos(yaw), std::sin(yaw) };
      if (show_steer_arrow_) {
        arr.markers.push_back(makeArrow("ppx/steer", 0, origin, dir, 4.0, 0.06, col({1.0,0.5,0.0}, 1.0)));
      }
      if (show_steer_text_) {
        visualization_msgs::msg::Marker txt;
        txt.header.frame_id = frame; txt.header.stamp = tnow;
        txt.ns = "ppx/steer_text"; txt.id = 0;
        txt.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
        txt.action = visualization_msgs::msg::Marker::ADD;
        txt.scale.z = 0.25; txt.color = col({1.0,1.0,1.0}, 1.0);
        txt.pose.position.x = origin.x; txt.pose.position.y = origin.y; txt.pose.position.z = 0.5;
        txt.pose.orientation.w = 1.0;
        char buf[64]; std::snprintf(buf, sizeof(buf), "%+.1f deg", cmd_deg);
        txt.text = std::string(buf);
        arr.markers.push_back(txt);
      }
    }

    pub_debug_->publish(arr);
  }

  // --------- Members ---------
  // Params / topics
  std::string path_topic_, speed_topic_, steer_topic_, debug_topic_, base_frame_;
  std::string frame_id_from_path_;

  // Vehicle
  double wheelbase_m_{1.3};

  // Prediction
  bool   predict_enable_{true};
  double T_pred_sec_{0.18};
  bool   use_cmd_curv_{true};

  // Ld
  bool   use_time_term_{true};
  double T_LA_{0.6}, L_bias_{0.9};
  bool   use_speed_term_{false};
  double L0_{2.5}, k_v_{0.6};
  bool   use_curv_term_{true};
  double k_curv_{0.6}, eps_kappa_{1e-6};
  double Ld_min_{1.0}, Ld_max_{7.0};

  // Hys / sticky
  double s_back_hys_m_{0.8};
  double allow_backtrack_heading_deg_{135.0};
  double sticky_window_m_{1.2};

  // Offset
  bool   outer_offset_enable_{true};
  double alpha_max_m_{3.0}, beta_max_{3.0};
  double curv_window_m_{2.0};
  double outer_offset_max_m_{1.0};
  double outer_offset_tau_max_{0.7};
  double outer_offset_kappa_gate_{0.03};
  double track_half_width_m_{2.5}, track_margin_m_{0.2};

  // Shaping
  double publish_rate_hz_{50.0};
  double steer_limit_deg_{30.0};
  double steer_rate_limit_deg_per_s_{360.0};
  double ema_tau_cmd_{0.12};
  double ema_tau_speed_{0.20};
  double target_ema_tau_{0.08};
  int    kappa_smooth_window_pts_{3};

  // Debug cfg
  bool show_lookahead_circle_{true}, show_points_{true}, show_tangent_{true},
       show_offset_vec_{true}, show_selected_path_{false},
       show_steer_arrow_{true}, show_steer_text_{false};
  double marker_alpha_{1.0}; int circle_pts_{60};

  std::array<double,3> color_pw_, color_pd_, color_pl_, color_tan_, color_vec_, color_cir_;

  // State
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr sub_path_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr sub_speed_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr   pub_steer_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_debug_;
  rclcpp::TimerBase::SharedPtr timer_;

  nav_msgs::msg::Path last_path_;
  std::vector<Pt> pts_;
  std::vector<double> cum_s_;

  bool have_speed_{false};
  double v_filt_{0.0};
  rclcpp::Time last_speed_time_{};

  rclcpp::Time last_cmd_time_{};
  double prev_cmd_deg_{0.0};   // smoothDeg 결과를 저장해 다음 step 예측에 사용

  bool have_pl_prev_{false};
  Pt pl_prev_{};
  rclcpp::Time last_target_time_{};
  double s_pd_prev_ = std::numeric_limits<double>::quiet_NaN();

  // TF
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PurePursuitPredictive>());
  rclcpp::shutdown();
  return 0;
}
