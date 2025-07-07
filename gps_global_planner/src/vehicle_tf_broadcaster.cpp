#include <chrono>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "std_msgs/msg/float32.hpp"

#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/static_transform_broadcaster.h"   // ★ 추가

using std::placeholders::_1;
using geometry_msgs::msg::PointStamped;
using std_msgs::msg::Float32;
using geometry_msgs::msg::TransformStamped;

class VehicleTFBroadcaster : public rclcpp::Node
{
public:
  VehicleTFBroadcaster()
  : Node("vehicle_tf_broadcaster"),
    tf_broadcaster_(this),
    static_broadcaster_(this)                     // ★ 추가
  {
    /* ---- 파라미터 ---- */
    declare_parameter("sensor_offset_z", -0.1);   // os_sensor가 10 cm 아래
    sensor_offset_z_ = get_parameter("sensor_offset_z").as_double();

    sub_xy_ = create_subscription<PointStamped>(
        "local_xy", 10, std::bind(&VehicleTFBroadcaster::cbXY, this, _1));
    sub_yaw_ = create_subscription<Float32>(
        "global_yaw", 10, std::bind(&VehicleTFBroadcaster::cbYaw, this, _1));

    using namespace std::chrono_literals;
    timer_ = create_wall_timer(50ms,
        std::bind(&VehicleTFBroadcaster::timerCB, this));

    broadcastStaticSensorTF();                    // ★ os_sensor 정적 TF 1회 송신
    RCLCPP_INFO(get_logger(),
      "VehicleTFBroadcaster started (20 Hz) – os_sensor offset z=%.2f m",
      sensor_offset_z_);
  }

private:
  /* ---- 콜백 ---- */
  void cbXY(const PointStamped::SharedPtr msg)
  {
    x_ = msg->point.x;
    y_ = msg->point.y;
    z_ = msg->point.z;
  }
  void cbYaw(const Float32::SharedPtr msg) { yaw_ = msg->data; }

  void timerCB()
  {
    TransformStamped t;
    t.header.stamp    = now();
    t.header.frame_id = "reference";
    t.child_frame_id  = "gps_antenna";

    t.transform.translation.x = x_;
    t.transform.translation.y = y_;
    t.transform.translation.z = z_;

    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, yaw_);
    t.transform.rotation.x = q.x();
    t.transform.rotation.y = q.y();
    t.transform.rotation.z = q.z();
    t.transform.rotation.w = q.w();

    tf_broadcaster_.sendTransform(t);
  }

  /* ---- 정적 TF: gps_antenna → os_sensor ---- */
  void broadcastStaticSensorTF()
  {
    TransformStamped t;
    t.header.stamp    = now();           // 정적 TF라도 time stamp 필요
    t.header.frame_id = "gps_antenna";
    t.child_frame_id  = "os_sensor";

    t.transform.translation.x = 0.0;
    t.transform.translation.y = 0.0;
    t.transform.translation.z = sensor_offset_z_; // 기본 −0.1 m
    t.transform.rotation.w = 1.0;                 // 회전 없음

    static_broadcaster_.sendTransform(t);
  }

  /* ---- 멤버 ---- */
  double x_{0.0}, y_{0.0}, z_{0.0}, yaw_{0.0};
  double sensor_offset_z_;

  rclcpp::Subscription<PointStamped>::SharedPtr sub_xy_;
  rclcpp::Subscription<Float32>::SharedPtr       sub_yaw_;

  tf2_ros::TransformBroadcaster       tf_broadcaster_;
  tf2_ros::StaticTransformBroadcaster static_broadcaster_;  // ★
  rclcpp::TimerBase::SharedPtr        timer_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<VehicleTFBroadcaster>());
  rclcpp::shutdown();
  return 0;
}
