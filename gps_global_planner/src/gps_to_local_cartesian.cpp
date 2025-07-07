#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"

constexpr double R_EARTH = 6378137.0;

inline std::pair<double,double>
latlon_to_local(double lat, double lon, double ref_lat, double ref_lon)
{
  const double lat_r = lat * M_PI/180.0;
  const double lon_r = lon * M_PI/180.0;
  const double ref_lat_r = ref_lat * M_PI/180.0;
  const double ref_lon_r = ref_lon * M_PI/180.0;
  double x = (lon_r - ref_lon_r) * std::cos(ref_lat_r) * R_EARTH;
  double y = (lat_r - ref_lat_r) * R_EARTH;
  return {x, y};
}

class GPSToLocalCartesian : public rclcpp::Node
{
public:
  GPSToLocalCartesian()
  : Node("gps_to_local_cartesian")
  {
    ref_lat_ = this->declare_parameter<double>("ref_lat", 37.54995);
    ref_lon_ = this->declare_parameter<double>("ref_lon", 127.05485);
    RCLCPP_INFO(get_logger(),
      "기준 좌표 = (%.6f, %.6f)", ref_lat_, ref_lon_);

    pub_ = this->create_publisher<geometry_msgs::msg::PointStamped>(
              "local_xy", 10);
    sub_ = this->create_subscription<sensor_msgs::msg::NavSatFix>(
              "ublox_gps_node/fix", 10,
              std::bind(&GPSToLocalCartesian::gps_cb, this,
                        std::placeholders::_1));
  }

private:
  void gps_cb(const sensor_msgs::msg::NavSatFix::SharedPtr msg)
  {
    auto [x, y] = latlon_to_local(
        msg->latitude, msg->longitude, ref_lat_, ref_lon_);
    geometry_msgs::msg::PointStamped out;
    out.header = msg->header;
    out.point.x = x;
    out.point.y = y;
    out.point.z = 0.0;
    pub_->publish(out);
    RCLCPP_DEBUG(this->get_logger(),
      "local_xy → (%.2f, %.2f)", x, y);
  }

  double ref_lat_, ref_lon_;
  rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr pub_;
  rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr sub_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<GPSToLocalCartesian>());
  rclcpp::shutdown();
  return 0;
}
