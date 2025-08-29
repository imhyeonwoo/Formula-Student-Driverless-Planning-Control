#include <cmath>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32.hpp>
#include <nav_msgs/msg/odometry.hpp>

class CurrentSpeedFromOdom : public rclcpp::Node
{
public:
  CurrentSpeedFromOdom()
  : Node("current_speed_from_odom"),
    have_last_pose_(false),
    last_speed_(0.0)
  {
    // ── Parameters ─────────────────────────────────────────
    odom_topic_ = declare_parameter<std::string>("odom_topic", "/odometry/filtered");
    include_z_  = declare_parameter<bool>("include_z", false);             // true면 3D 속도
    alpha_      = declare_parameter<double>("low_pass_alpha", 0.2);        // 0<α<1: 필터, 1.0: 미적용
    use_twist_first_ = declare_parameter<bool>("use_twist_first", true);   // false면 포즈 미분 우선

    // ── I/O ────────────────────────────────────────────────
    pub_speed_ = create_publisher<std_msgs::msg::Float32>("/current_speed/odom", 10);
    sub_odom_  = create_subscription<nav_msgs::msg::Odometry>(
      odom_topic_, rclcpp::SensorDataQoS(),
      std::bind(&CurrentSpeedFromOdom::odomCB, this, std::placeholders::_1));

    RCLCPP_INFO(get_logger(),
      "CurrentSpeedFromOdom started\n  odom_topic=%s\n  include_z=%s  alpha=%.3f  use_twist_first=%s",
      odom_topic_.c_str(), include_z_ ? "true" : "false", alpha_,
      use_twist_first_ ? "true" : "false");
  }

private:
  void odomCB(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    const auto now = rclcpp::Time(msg->header.stamp);

    // 1) 측정 속도 계산
    double meas_speed = 0.0;
    bool have_meas = false;

    if (use_twist_first_) {
      have_meas = computeFromTwist(*msg, meas_speed);
      if (!have_meas) {
        have_meas = computeFromPose(*msg, now, meas_speed);
      }
    } else {
      have_meas = computeFromPose(*msg, now, meas_speed);
      if (!have_meas) {
        have_meas = computeFromTwist(*msg, meas_speed);
      }
    }

    if (!have_meas) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                           "No valid speed measurement from odometry");
      return;
    }

    // 2) 1차 저역통과 필터 (alpha in (0,1))
    double speed_out;
    if (alpha_ > 0.0 && alpha_ < 1.0) {
      speed_out = alpha_ * meas_speed + (1.0 - alpha_) * last_speed_;
    } else {
      speed_out = meas_speed; // 필터 미적용
    }
    last_speed_ = speed_out;

    // 3) Publish
    std_msgs::msg::Float32 msg_out;
    msg_out.data = static_cast<float>(speed_out);
    pub_speed_->publish(msg_out);
  }

  // twist 기반 (기본) 속도 계산
  bool computeFromTwist(const nav_msgs::msg::Odometry &odom, double &speed_out) const
  {
    const auto &tw = odom.twist.twist.linear;
    if (!std::isfinite(tw.x) || !std::isfinite(tw.y) || !std::isfinite(tw.z)) {
      return false;
    }
    if (include_z_) speed_out = std::sqrt(tw.x*tw.x + tw.y*tw.y + tw.z*tw.z);
    else            speed_out = std::sqrt(tw.x*tw.x + tw.y*tw.y);
    return true;
  }

  // 포즈 미분 기반(플랜너용 평면 속도) 계산
  bool computeFromPose(const nav_msgs::msg::Odometry &odom,
                       const rclcpp::Time& now, double &speed_out)
  {
    const double x = odom.pose.pose.position.x;
    const double y = odom.pose.pose.position.y;
    const double z = odom.pose.pose.position.z;

    if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z)) {
      return false;
    }

    if (!have_last_pose_) {
      last_x_ = x; last_y_ = y; last_z_ = z; last_stamp_ = now;
      have_last_pose_ = true;
      return false; // 다음 콜백부터 계산 가능
    }

    const double dt = (now - last_stamp_).seconds();
    if (dt <= 1e-6) return false;

    const double dx = x - last_x_;
    const double dy = y - last_y_;
    const double dz = z - last_z_;

    speed_out = include_z_ ? std::sqrt(dx*dx + dy*dy + dz*dz)/dt
                           : std::sqrt(dx*dx + dy*dy)/dt;

    last_x_ = x; last_y_ = y; last_z_ = z; last_stamp_ = now;
    return true;
  }

  // Members
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr pub_speed_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_odom_;

  std::string odom_topic_;
  bool include_z_;
  double alpha_;             // low-pass alpha
  bool use_twist_first_;

  // Pose-diff state
  bool have_last_pose_;
  double last_x_, last_y_, last_z_;
  rclcpp::Time last_stamp_;

  // Filter state
  double last_speed_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CurrentSpeedFromOdom>());
  rclcpp::shutdown();
  return 0;
}
