// global_speed_planner.cpp - curvature‑based speed planning for GLOBAL path
#include <fstream>
#include <sstream>
#include <string>
#include <vector>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "ament_index_cpp/get_package_share_directory.hpp"

using geometry_msgs::msg::Point;
using visualization_msgs::msg::Marker;

/* 지구 반경 [m] */
constexpr double R_EARTH = 6'378'137.0;

/* 위·경도 → 기준 평면(x,y) */
static inline std::pair<double,double> latlon_to_local(
    double lat, double lon, double ref_lat, double ref_lon)
{
  const double lat_r = lat      * M_PI/180.0;
  const double lon_r = lon      * M_PI/180.0;
  const double ref_lat_r = ref_lat * M_PI/180.0;
  const double ref_lon_r = ref_lon * M_PI/180.0;
  double x = (lon_r - ref_lon_r) * std::cos(ref_lat_r) * R_EARTH;
  double y = (lat_r - ref_lat_r) * R_EARTH;
  return {x, y};
}

/* CSV 한 행 */
struct Row { double x, y; };

class GlobalSpeedPlanner : public rclcpp::Node
{
public:
  GlobalSpeedPlanner() : Node("global_speed_planner")
  {
    /* ───────── 파라미터 ───────── */
    std::string def_csv =
      ament_index_cpp::get_package_share_directory("gps_global_planner") +
      "/data/administrator_250721.csv";               // 기본 CSV 경로

    declare_parameter("csv_filename",  def_csv);
    declare_parameter("min_distance",  0.3);
    declare_parameter("ref_lat",       37.54995);
    declare_parameter("ref_lon",       127.05485);

    declare_parameter("a_long_max",  3.0);            // 가속 한계 [m/s²]
    declare_parameter("a_brake_max", 3.5);            // 감속 한계 [m/s²]
    declare_parameter("a_lat_max",   3.0);            // 횡가속 한계 [m/s²]
    declare_parameter("smooth_win",  3);              // 이동평균 창(홀수)

    csv_file_      = get_parameter("csv_filename").as_string();
    min_dist_      = get_parameter("min_distance").as_double();
    ref_lat_       = get_parameter("ref_lat").as_double();
    ref_lon_       = get_parameter("ref_lon").as_double();
    a_long_max_    = get_parameter("a_long_max").as_double();
    a_brake_max_   = get_parameter("a_brake_max").as_double();
    a_lat_max_     = get_parameter("a_lat_max").as_double();
    smooth_win_    = std::max(1,
                      static_cast<int>(get_parameter("smooth_win").as_int())); // ← cast

    /* ── 퍼블리셔: latched QoS ── */
    auto qos_latched = rclcpp::QoS(rclcpp::KeepLast(1)).transient_local();
    speed_pub_ = create_publisher<std_msgs::msg::Float32MultiArray>(
                   "/global_speed_profile", qos_latched);
    bar_pub_   = create_publisher<Marker>(
                   "/global_speed_bars", qos_latched);

    /* ── 처리 파이프라인 ── */
    load_csv();
    filter_distance();
    compute_speed_profile();
    publish_once();                                   // 한 번만 발행

    RCLCPP_INFO(get_logger(), "Global speed profile ready (%zu pts)", pts_.size());
  }

private:
  /* CSV: index,Long,Lat,UTM_X,UTM_Y,Cov00 */
  void load_csv()
  {
    std::ifstream fin(csv_file_);
    if (!fin.is_open()) {
      RCLCPP_ERROR(get_logger(), "CSV not found: %s", csv_file_.c_str());
      return;
    }
    std::string line; std::getline(fin, line);        // header skip
    while (std::getline(fin, line))
    {
      std::stringstream ss(line);
      std::vector<std::string> c; std::string s;
      while (std::getline(ss, s, ',')) c.push_back(s);
      if (c.size() < 3) continue;
      try {
        double lon = std::stod(c[1]);
        double lat = std::stod(c[2]);
        auto [x,y] = latlon_to_local(lat, lon, ref_lat_, ref_lon_);
        pts_.push_back({x,y});
      } catch (...) { continue; }
    }
  }

  /* 최소 거리 다운샘플 */
  void filter_distance()
  {
    if (pts_.empty()) return;
    std::vector<Row> out; out.reserve(pts_.size());
    out.push_back(pts_.front());
    for (auto &p : pts_)
    {
      double dx = p.x - out.back().x;
      double dy = p.y - out.back().y;
      if (std::hypot(dx,dy) >= min_dist_) out.push_back(p);
    }
    pts_.swap(out);
  }

  /* 곡률 + 가·감속 한계 기반 속도 계산 */
  void compute_speed_profile()
  {
    const size_t N = pts_.size();
    speed_.assign(N, 0.0);
    if (N < 2) return;

    /* ─ 거리 ─ */
    std::vector<double> dist(N,0.0);
    for (size_t i = 0; i < N-1; ++i) {
      dist[i] = std::hypot(pts_[i+1].x - pts_[i].x,
                           pts_[i+1].y - pts_[i].y);
      if (dist[i] < 1e-6) dist[i] = 1e-6;
    }

    /* ─ 곡률 한계 ─ */
    std::vector<double> v_kappa(N, std::numeric_limits<double>::infinity());
    for (size_t i = 1; i < N-1; ++i) {
      auto &p0 = pts_[i-1]; auto &p1 = pts_[i]; auto &p2 = pts_[i+1];
      double dx1 = p1.x - p0.x, dy1 = p1.y - p0.y;
      double dx2 = p2.x - p0.x, dy2 = p2.y - p0.y;
      double cross = std::abs(dx1*dy2 - dy1*dx2);
      double d01 = std::hypot(dx1, dy1);
      double d02 = std::hypot(dx2, dy2);
      double d12 = std::hypot(p2.x - p1.x, p2.y - p1.y);
      if (d01 < 1e-6 || d02 < 1e-6 || d12 < 1e-6) continue;
      double kappa = (2.0 * cross) / (d01 * d02 * d12);
      if (kappa >= 1e-9)
        v_kappa[i] = std::sqrt(a_lat_max_ / kappa);
    }

    /* 초기 속도 = 곡률 한계 */
    for (size_t i = 0; i < N; ++i)
      speed_[i] = std::isfinite(v_kappa[i]) ? v_kappa[i] : 1e6;

    /* forward accel 한계 */
    speed_[0] = 0.0;
    for (size_t i = 1; i < N; ++i) {
      double v_lim = std::sqrt(speed_[i-1]*speed_[i-1] + 2*a_long_max_*dist[i-1]);
      if (speed_[i] > v_lim) speed_[i] = v_lim;
    }

    /* backward brake 한계 */
    for (int i = static_cast<int>(N)-2; i >= 0; --i) {
      double v_lim = std::sqrt(speed_[i+1]*speed_[i+1] + 2*a_brake_max_*dist[i]);
      if (speed_[i] > v_lim) speed_[i] = v_lim;
    }

    /* 이동평균 smoothing */
    if (smooth_win_ >= 3 && N >= static_cast<size_t>(smooth_win_)) {
      int w = smooth_win_;
      std::vector<double> tmp = speed_;
      for (size_t i = w/2; i + w/2 < N; ++i) {
        double sum = 0.0;
        for (int k = -w/2; k <= w/2; ++k) sum += tmp[i+k];
        speed_[i] = sum / w;
        if (speed_[i] > tmp[i]) speed_[i] = tmp[i];
      }
    }
  }

  /* 한번만 퍼블리시 (latched) */
  void publish_once()
  {
    /* 속도 배열 */
    std_msgs::msg::Float32MultiArray msg;
    msg.data.reserve(speed_.size());
    for (double v : speed_)
      msg.data.push_back(static_cast<float>(v));
    speed_pub_->publish(msg);

    /* 시각화 (CUBE_LIST) */
    Marker mk;
    mk.header.stamp    = now();
    mk.header.frame_id = "reference";
    mk.ns = "global_speed"; mk.id = 0;
    mk.type = Marker::CUBE_LIST; mk.action = Marker::ADD;
    mk.scale.x = mk.scale.y = 0.4;

    for (size_t i = 0; i < pts_.size(); ++i) {
      Point p; p.x = pts_[i].x; p.y = pts_[i].y; p.z = speed_[i] * 0.25;
      mk.points.push_back(p);
      std_msgs::msg::ColorRGBA c; c.r=0.0f; c.g=0.0f; c.b=1.0f; c.a=0.8f;
      mk.colors.push_back(c);
    }
    bar_pub_->publish(mk);
  }

  /* ── 멤버 ── */
  std::string csv_file_;
  double min_dist_, ref_lat_, ref_lon_;
  double a_long_max_, a_brake_max_, a_lat_max_;
  int smooth_win_;

  std::vector<Row>    pts_;
  std::vector<double> speed_;

  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr speed_pub_;
  rclcpp::Publisher<Marker>::SharedPtr                           bar_pub_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<GlobalSpeedPlanner>());
  rclcpp::shutdown();
  return 0;
}
