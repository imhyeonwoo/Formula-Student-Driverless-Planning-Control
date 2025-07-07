#include <fstream>
#include <sstream>
#include <string>
#include <vector>
#include <cmath>
#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "std_msgs/msg/color_rgba.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "ament_index_cpp/get_package_share_directory.hpp"

using geometry_msgs::msg::Point;
using std_msgs::msg::ColorRGBA;
using visualization_msgs::msg::Marker;

/* 지구 반경 [m] */
constexpr double R_EARTH = 6'378'137.0;

static inline std::pair<double,double> latlon_to_local(
    double lat, double lon, double ref_lat, double ref_lon)
{
  double lat_r = lat * M_PI/180.0,  lon_r = lon * M_PI/180.0;
  double ref_lat_r = ref_lat * M_PI/180.0, ref_lon_r = ref_lon * M_PI/180.0;
  double x = (lon_r - ref_lon_r) * std::cos(ref_lat_r) * R_EARTH;
  double y = (lat_r - ref_lat_r) * R_EARTH;
  return {x, y};
}

struct CSVRow { double x, y; int status; };

class StatusColoredPathPublisher : public rclcpp::Node
{
public:
  StatusColoredPathPublisher()
  : Node("status_colored_path_publisher")
  {
    /* ───────── 파라미터 ───────── */
    std::string def_csv =
      ament_index_cpp::get_package_share_directory("gps_global_planner") +
      "/data/ahrs3.csv";

    declare_parameter("csv_filename", def_csv);
    declare_parameter("min_distance", 0.3);
    declare_parameter("ref_lat",      37.54995);
    declare_parameter("ref_lon",      127.05485);

    csv_file_ = get_parameter("csv_filename").as_string();
    min_dist_ = get_parameter("min_distance").as_double();
    ref_lat_  = get_parameter("ref_lat").as_double();
    ref_lon_  = get_parameter("ref_lon").as_double();

    /* CSV 파싱 → pts_ */
    load_csv();
    filter_distance();

    marker_pub_ = create_publisher<Marker>("global_path_marker", 1);

    using namespace std::chrono_literals;
    timer_ = create_wall_timer(500ms,
            std::bind(&StatusColoredPathPublisher::timer_cb, this));
  }

private:
  /* CSV: index,Long,Lat,UTM_X,UTM_Y,Status */
  void load_csv()
  {
    std::ifstream fin(csv_file_);
    if (!fin.is_open()) {
      RCLCPP_ERROR(get_logger(), "CSV not found: %s", csv_file_.c_str());
      return;
    }
    std::string line; std::getline(fin, line); // header
    while (std::getline(fin, line))
    {
      std::stringstream ss(line);
      std::vector<std::string> c; std::string s;
      while (std::getline(ss, s, ',')) c.push_back(s);
      if (c.size() < 6) continue;
      try {
        double lon = std::stod(c[1]), lat = std::stod(c[2]);
        int    st  = std::stoi(c[5]);
        auto [x,y] = latlon_to_local(lat, lon, ref_lat_, ref_lon_);
        pts_.push_back({x,y,st});
      } catch (...) { continue; }
    }
    RCLCPP_INFO(get_logger(), "CSV rows loaded: %lu", pts_.size());
  }

  /* 최소 거리 필터 (모든 status 포함, 포인트 밀집도만 줄임) */
  void filter_distance()
  {
    if (pts_.empty()) return;
    std::vector<CSVRow> out; out.push_back(pts_.front());
    for (auto &p : pts_)
    {
      double dx = p.x - out.back().x;
      double dy = p.y - out.back().y;
      if (std::hypot(dx,dy) >= min_dist_) out.push_back(p);
    }
    pts_.swap(out);
  }

  void timer_cb()
  {
    Marker mk;
    mk.header.stamp    = now();
    mk.header.frame_id = "reference";
    mk.ns   = "status_path"; mk.id = 0;
    mk.type = Marker::LINE_STRIP; mk.action = Marker::ADD;
    mk.scale.x = 0.25;         // 선 굵기

    for (auto &p : pts_)
    {
      Point pt; pt.x = p.x; pt.y = p.y; pt.z = 0.0;
      mk.points.push_back(pt);

      ColorRGBA col;
      if (p.status == 2) {           // RTK Fix → green
        col.r = 0.0; col.g = 1.0; col.b = 0.0; col.a = 1.0;
      } else {                       // 기타 → red
        col.r = 1.0; col.g = 0.0; col.b = 0.0; col.a = 1.0;
      }
      mk.colors.push_back(col);
    }
    marker_pub_->publish(mk);
  }

  /* 멤버 */
  std::string csv_file_; double min_dist_, ref_lat_, ref_lon_;
  std::vector<CSVRow> pts_;

  rclcpp::Publisher<Marker>::SharedPtr marker_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<StatusColoredPathPublisher>());
  rclcpp::shutdown();
  return 0;
}