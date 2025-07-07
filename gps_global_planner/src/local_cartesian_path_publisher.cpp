#include <fstream>
#include <sstream>
#include <string>
#include <vector>
#include <cmath>
#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "ament_index_cpp/get_package_share_directory.hpp"

using nav_msgs::msg::Path;
using geometry_msgs::msg::PoseStamped;

/* 지구 반경 */
constexpr double R_EARTH = 6'378'137.0;

/* 위경도 → 기준점 기준 평면(m) */
static inline std::pair<double,double> latlon_to_local(
        double lat, double lon, double ref_lat, double ref_lon)
{
  const double lat_r = lat * M_PI/180.0;
  const double lon_r = lon * M_PI/180.0;
  const double ref_lat_r = ref_lat * M_PI/180.0;
  const double ref_lon_r = ref_lon * M_PI/180.0;

  double x = (lon_r - ref_lon_r) * std::cos(ref_lat_r) * R_EARTH;
  double y = (lat_r - ref_lat_r) * R_EARTH;
  return {x, y};
}

/* 2-D 점 구조체 */
struct Pt { double x, y; };

class LocalCartesianPathPublisher : public rclcpp::Node
{
public:
  LocalCartesianPathPublisher()
  : Node("local_cartesian_path_publisher")
  {
    /* ---------- 파라미터 ---------- */
    std::string def_csv =
      ament_index_cpp::get_package_share_directory("gps_global_planner") +
      "/data/ahrs3.csv";

    this->declare_parameter("csv_filename", def_csv);
    this->declare_parameter("target_spacing", 0.2);
    this->declare_parameter("min_distance", 0.3);
    this->declare_parameter("ref_lat", 37.54995);
    this->declare_parameter("ref_lon", 127.05485);

    csv_file_ = this->get_parameter("csv_filename").as_string();
    spacing_  = this->get_parameter("target_spacing").as_double();
    min_dist_ = this->get_parameter("min_distance").as_double();
    ref_lat_  = this->get_parameter("ref_lat").as_double();
    ref_lon_  = this->get_parameter("ref_lon").as_double();

    /* ---------- CSV 로드 ---------- */
    auto raw  = load_csv();
    auto filt = filter_points(raw);
    points_   = resample_linear(filt);
    RCLCPP_INFO(get_logger(), "Resampled path = %lu points", points_.size());

    /* ---------- 퍼블리셔 & 타이머 ---------- */
    path_pub_  = this->create_publisher<Path>("resampled_path", 1);
    local_pub_ = this->create_publisher<PoseStamped>("local_xy_goal", 10);

    using namespace std::chrono_literals;
    timer_ = this->create_wall_timer(
               100ms,
               std::bind(&LocalCartesianPathPublisher::timer_cb, this));
  }

private:
  /* ───── CSV 로드 ───── */
  std::vector<Pt> load_csv()
  {
    std::vector<Pt> v;
    std::ifstream fin(csv_file_);
    if (!fin.is_open()) {
      RCLCPP_ERROR(get_logger(), "CSV not found: %s", csv_file_.c_str());
      return v;
    }
    std::string line;
    std::getline(fin, line);              // header skip
    while (std::getline(fin, line))
    {
      std::stringstream ss(line);
      std::vector<std::string> col;
      std::string s;
      while (std::getline(ss, s, ',')) col.push_back(s);
      if (col.size() < 3) continue;
      try {
        double lon = std::stod(col[1]);
        double lat = std::stod(col[2]);
        auto [x,y] = latlon_to_local(lat, lon, ref_lat_, ref_lon_);
        v.push_back({x,y});
      } catch (...) { continue; }
    }
    return v;
  }

  /* ───── 최소 거리 필터 ───── */
  std::vector<Pt> filter_points(const std::vector<Pt>& in)
  {
    std::vector<Pt> out;
    if (in.empty()) return out;
    out.push_back(in.front());
    for (auto &p : in)
    {
      double dx = p.x - out.back().x,
             dy = p.y - out.back().y;
      if (std::hypot(dx,dy) >= min_dist_) out.push_back(p);
    }
    return out;
  }

  /* ───── 선형 리샘플 ───── */
  std::vector<Pt> resample_linear(const std::vector<Pt>& in)
  {
    if (in.size() < 2) return in;
    std::vector<Pt> out; out.push_back(in.front());
    double rest = spacing_;      // 거리 잔여분

    for (size_t i=1;i<in.size();++i)
    {
      Pt p0 = in[i-1], p1 = in[i];
      double seg = std::hypot(p1.x-p0.x, p1.y-p0.y);
      while (seg >= rest)
      {
        double t = rest / seg;
        Pt np{ p0.x + t*(p1.x-p0.x),
               p0.y + t*(p1.y-p0.y) };
        out.push_back(np);
        seg -= rest;
        p0  = np;
        rest = spacing_;
      }
      rest -= seg;
    }
    return out;
  }

  /* ───── 타이머 콜백 ───── */
  void timer_cb()
  {
    auto now = this->now();

    Path path;
    path.header.stamp = now;
    path.header.frame_id = "reference";

    for (auto &p : points_) {
      PoseStamped ps;
      ps.header = path.header;
      ps.pose.position.x = p.x;
      ps.pose.position.y = p.y;
      ps.pose.orientation.w = 1.0;
      path.poses.push_back(ps);
    }
    path_pub_->publish(path);

    if (!points_.empty()) {
      Pt back = points_.back();
      PoseStamped last;
      last.header = path.header;
      last.pose.position.x = back.x;
      last.pose.position.y = back.y;
      last.pose.orientation.w = 1.0;
      local_pub_->publish(last);
    }
  }

  /* 멤버 */
  std::string csv_file_;
  double spacing_, min_dist_, ref_lat_, ref_lon_;
  std::vector<Pt> points_;

  rclcpp::Publisher<Path>::SharedPtr path_pub_;
  rclcpp::Publisher<PoseStamped>::SharedPtr local_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LocalCartesianPathPublisher>());
  rclcpp::shutdown();
  return 0;
}
