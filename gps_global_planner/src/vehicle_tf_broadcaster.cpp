// file: src/Planning/gps_global_planner/src/vehicle_tf_broadcaster.cpp
#include <chrono>
#include <cmath>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "std_msgs/msg/float32.hpp"

#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/static_transform_broadcaster.h"

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
    static_broadcaster_(this)
  {
    /* ---- 파라미터 ---- */
    // base_link -> gps_antenna
    declare_parameter("antenna_offset_x", 0.0);
    declare_parameter("antenna_offset_y", 0.0);
    declare_parameter("antenna_offset_z", 3.0);

    // base_link -> os_sensor
    declare_parameter("sensor_offset_x", 0.0);
    declare_parameter("sensor_offset_y", 0.0);
    declare_parameter("sensor_offset_z", 2.0);

    // base_link의 z를 지면(0)으로 고정할지 여부
    declare_parameter("fix_base_z_to_zero", true);
    declare_parameter("base_z_level", 0.0);   // fix_base_z_to_zero=true일 때 적용

    antenna_offset_x_ = get_parameter("antenna_offset_x").as_double();
    antenna_offset_y_ = get_parameter("antenna_offset_y").as_double();
    antenna_offset_z_ = get_parameter("antenna_offset_z").as_double();

    sensor_offset_x_  = get_parameter("sensor_offset_x").as_double();
    sensor_offset_y_  = get_parameter("sensor_offset_y").as_double();
    sensor_offset_z_  = get_parameter("sensor_offset_z").as_double();

    fix_base_z_to_zero_ = get_parameter("fix_base_z_to_zero").as_bool();
    base_z_level_       = get_parameter("base_z_level").as_double();

    /* ---- I/O ---- */
    sub_xy_ = create_subscription<PointStamped>(
        "local_xy", 10, std::bind(&VehicleTFBroadcaster::cbXY, this, _1));
    sub_yaw_ = create_subscription<Float32>(
        "global_yaw", 10, std::bind(&VehicleTFBroadcaster::cbYaw, this, _1));

    using namespace std::chrono_literals;
    timer_ = create_wall_timer(50ms,
        std::bind(&VehicleTFBroadcaster::timerCB, this));

    // 정적 TF: base_link → {gps_antenna, os_sensor}
    broadcastStaticTFs();

    RCLCPP_INFO(get_logger(),
      "VehicleTFBroadcaster started (20 Hz)\n"
      "  base_link->gps_antenna offset = [%.3f, %.3f, %.3f] m\n"
      "  base_link->os_sensor   offset = [%.3f, %.3f, %.3f] m\n"
      "  fix_base_z_to_zero=%s (base_z_level=%.3f)",
      antenna_offset_x_, antenna_offset_y_, antenna_offset_z_,
      sensor_offset_x_,  sensor_offset_y_,  sensor_offset_z_,
      fix_base_z_to_zero_ ? "true" : "false", base_z_level_);
  }

private:
  /* ---- 콜백 ---- */
  void cbXY(const PointStamped::SharedPtr msg)
  {
    x_gps_or_base_ = msg->point.x;
    y_gps_or_base_ = msg->point.y;
    z_gps_or_base_ = msg->point.z;  // 현재 파이프라인에선 보통 0.0
  }

  void cbYaw(const Float32::SharedPtr msg)
  {
    yaw_ = msg->data; // 라디안 가정. (deg라면 외부에서 변환하거나 여기서 변환 처리)
  }

  void timerCB()
  {
    // 동적 TF: reference -> base_link
    // local_xy가 GPS 안테나 위치라면 base = gps - R(yaw)*offset(base->gps)
    const double cy = std::cos(yaw_);
    const double sy = std::sin(yaw_);

    const double dx_world = cy * antenna_offset_x_ - sy * antenna_offset_y_;
    const double dy_world = sy * antenna_offset_x_ + cy * antenna_offset_y_;
    const double dz_world = antenna_offset_z_;

    const double base_x = x_gps_or_base_ - dx_world;
    const double base_y = y_gps_or_base_ - dy_world;

    // ★ z는 지면 기준으로 고정 (base_link가 땅에 박히지 않도록)
    const double base_z = fix_base_z_to_zero_ ? base_z_level_
                                              : (z_gps_or_base_ - dz_world);

    TransformStamped t;
    t.header.stamp    = now();
    t.header.frame_id = "reference";
    t.child_frame_id  = "base_link";

    t.transform.translation.x = base_x;
    t.transform.translation.y = base_y;
    t.transform.translation.z = base_z;

    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, yaw_); // roll/pitch=0 가정
    t.transform.rotation.x = q.x();
    t.transform.rotation.y = q.y();
    t.transform.rotation.z = q.z();
    t.transform.rotation.w = q.w();

    tf_broadcaster_.sendTransform(t);
  }

  /* ---- 정적 TF: base_link → {gps_antenna, os_sensor} ---- */
  void broadcastStaticTFs()
  {
    std::vector<TransformStamped> vec;

    // base_link -> gps_antenna
    {
      TransformStamped t;
      t.header.stamp    = now();
      t.header.frame_id = "base_link";
      t.child_frame_id  = "gps_antenna";
      t.transform.translation.x = antenna_offset_x_;
      t.transform.translation.y = antenna_offset_y_;
      t.transform.translation.z = antenna_offset_z_;
      t.transform.rotation.w = 1.0; // 회전 없음
      vec.push_back(t);
    }

    // base_link -> os_sensor
    {
      TransformStamped t;
      t.header.stamp    = now();
      t.header.frame_id = "base_link";
      t.child_frame_id  = "os_sensor";
      t.transform.translation.x = sensor_offset_x_;
      t.transform.translation.y = sensor_offset_y_;
      t.transform.translation.z = sensor_offset_z_;
      t.transform.rotation.w = 1.0; // 회전 없음
      vec.push_back(t);
    }

    static_broadcaster_.sendTransform(vec);
  }

  /* ---- 멤버 ---- */
  double x_gps_or_base_{0.0}, y_gps_or_base_{0.0}, z_gps_or_base_{0.0}, yaw_{0.0};

  double antenna_offset_x_{0.0}, antenna_offset_y_{0.0}, antenna_offset_z_{0.0};
  double sensor_offset_x_{0.0},  sensor_offset_y_{0.0},  sensor_offset_z_{0.0};

  bool   fix_base_z_to_zero_{true};
  double base_z_level_{0.0};

  rclcpp::Subscription<PointStamped>::SharedPtr sub_xy_;
  rclcpp::Subscription<Float32>::SharedPtr      sub_yaw_;

  tf2_ros::TransformBroadcaster       tf_broadcaster_;
  tf2_ros::StaticTransformBroadcaster static_broadcaster_;
  rclcpp::TimerBase::SharedPtr        timer_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<VehicleTFBroadcaster>());
  rclcpp::shutdown();
  return 0;
}
