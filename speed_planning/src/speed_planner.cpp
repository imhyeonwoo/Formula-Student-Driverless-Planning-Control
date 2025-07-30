#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/path.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>

#include <vector>
#include <cmath>
#include <algorithm>

using std::placeholders::_1;

class SpeedPlanner : public rclcpp::Node
{
public:
  SpeedPlanner()
  : Node("speed_planner")
  {
    /* ───────── parameters ───────── */
    declare_parameter("v_max",         10.0);
    declare_parameter("a_lat_max",      3.0);
    declare_parameter("a_long_max",     2.0);
    declare_parameter("lookahead_pts",  8);
    declare_parameter("path_ds",        0.5);
    declare_parameter("min_kappa",   1e-4);          // 0 divide guard

    v_max_      = get_parameter("v_max").as_double();
    a_lat_max_  = get_parameter("a_lat_max").as_double();
    a_long_max_ = get_parameter("a_long_max").as_double();
    L_          = get_parameter("lookahead_pts").as_int();
    ds_         = get_parameter("path_ds").as_double();
    eps_kappa_  = get_parameter("min_kappa").as_double();

    sub_path_ = create_subscription<nav_msgs::msg::Path>(
        "/local_planned_path", 10,
        std::bind(&SpeedPlanner::cbPath, this, _1));

    pub_speed_ = create_publisher<std_msgs::msg::Float32MultiArray>(
        "/desired_speed_profile", 1);

    RCLCPP_INFO(get_logger(),
      "SpeedPlanner ready (v_max=%.2f a_lat=%.2f a_long=%.2f L=%d ds=%.2f)",
      v_max_, a_lat_max_, a_long_max_, L_, ds_);
  }

private:
  /* ---------- Path callback ---------- */
  void cbPath(const nav_msgs::msg::Path::SharedPtr msg)
  {
    const size_t N = msg->poses.size();
    if (N < 3) return;

    /* 1. curvature κ --------------------------------------- */
    std::vector<double> kappa(N, 0.0);

    for (size_t i = 1; i + 1 < N; ++i)
    {
      const auto &p0 = msg->poses[i-1].pose.position;
      const auto &p1 = msg->poses[i  ].pose.position;
      const auto &p2 = msg->poses[i+1].pose.position;

      double a = std::hypot(p1.x - p0.x, p1.y - p0.y);
      double b = std::hypot(p2.x - p1.x, p2.y - p1.y);
      double c = std::hypot(p2.x - p0.x, p2.y - p0.y);

      if (a < 1e-6 || b < 1e-6 || c < 1e-6) { kappa[i] = 0.0; continue; }

      double s = 0.5 * (a + b + c);
      double area2 = s * (s - a) * (s - b) * (s - c);
      if (area2 <= 0.0) { kappa[i] = 0.0; continue; }

      double A = std::sqrt(area2);
      kappa[i] = (4.0 * A) / (a * b * c);   // 1/R
    }

    /* 2. look‑ahead averaged curvature --------------------- */
    std::vector<double> kappa_avg(N, 0.0);
    for (size_t i = 0; i < N; ++i) {
      size_t j_end = std::min(N, i + static_cast<size_t>(L_));
      double sum = 0.0;
      for (size_t j = i; j < j_end; ++j) sum += kappa[j];
      kappa_avg[i] = sum / static_cast<double>(j_end - i);
    }

    /* 3. lateral limit speed ------------------------------- */
    std::vector<double> v(N, v_max_);
    for (size_t i = 0; i < N; ++i) {
      double kap = std::max(kappa_avg[i], eps_kappa_);
      v[i] = std::min(v[i], std::sqrt(a_lat_max_ / kap));
    }

    /* 4. longitudinal limit (forward / backward) ----------- */
    double a2ds = 2.0 * a_long_max_ * ds_;

    // forward (acceleration)
    for (size_t i = 1; i < N; ++i) {
      double v_lim = std::sqrt(v[i-1]*v[i-1] + a2ds);
      if (v[i] > v_lim) v[i] = v_lim;
    }
    // backward (deceleration)
    for (size_t i = N - 1; i-- > 0; ) {
      double v_lim = std::sqrt(v[i+1]*v[i+1] + a2ds);
      if (v[i] > v_lim) v[i] = v_lim;
    }

    /* 5. publish ------------------------------------------- */
    std_msgs::msg::Float32MultiArray out;
    out.data.assign(v.begin(), v.end());
    pub_speed_->publish(out);
  }

  /* ---------- members ---------- */
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr sub_path_;
  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr pub_speed_;

  double v_max_, a_lat_max_, a_long_max_, ds_, eps_kappa_;
  int    L_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SpeedPlanner>());
  rclcpp::shutdown();
  return 0;
}
