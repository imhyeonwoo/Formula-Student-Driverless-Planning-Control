#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "std_msgs/msg/float32.hpp"
#include <cmath>

class GPSSpeedNode : public rclcpp::Node
{
public:
  GPSSpeedNode() : Node("gps_speed_node_float")
  {
    in_topic_   = this->declare_parameter<std::string>("in_topic", "local_xy");
    out_topic_  = this->declare_parameter<std::string>("out_topic", "current_speed");
    alpha_      = this->declare_parameter<double>("ema_alpha", 0.3);   // 0~1
    max_speed_  = this->declare_parameter<double>("max_speed", 60.0);  // m/s (필요에 맞게 조정)
    min_dt_     = this->declare_parameter<double>("min_dt", 0.02);     // s

    RCLCPP_INFO(get_logger(), "Subscribe: %s | Publish: %s (Float32) | alpha=%.2f",
                in_topic_.c_str(), out_topic_.c_str(), alpha_);

    pub_ = this->create_publisher<std_msgs::msg::Float32>(out_topic_, 10);
    sub_ = this->create_subscription<geometry_msgs::msg::PointStamped>(
      in_topic_, 10, std::bind(&GPSSpeedNode::xyCallback, this, std::placeholders::_1));
  }

private:
  void xyCallback(const geometry_msgs::msg::PointStamped::SharedPtr msg)
  {
    rclcpp::Time t(msg->header.stamp);
    if (!has_prev_) {
      prev_x_ = msg->point.x;
      prev_y_ = msg->point.y;
      prev_t_ = t;
      has_prev_ = true;
      return;
    }

    double dt = (t - prev_t_).seconds();
    if (dt <= min_dt_) return;

    double dx = msg->point.x - prev_x_;
    double dy = msg->point.y - prev_y_;
    double dist = std::hypot(dx, dy);
    double v_inst = dist / dt;

    if (std::isnan(v_inst) || std::isinf(v_inst) || v_inst > max_speed_) {
      RCLCPP_WARN_THROTTLE(get_logger(), *this->get_clock(), 1000,
                           "Abnormal speed %.2f m/s (dt=%.3f). Skip.", v_inst, dt);
      prev_x_ = msg->point.x; prev_y_ = msg->point.y; prev_t_ = t;
      return;
    }

    if (!has_ema_) { v_ema_ = v_inst; has_ema_ = true; }
    else { v_ema_ = alpha_ * v_inst + (1.0 - alpha_) * v_ema_; }

    std_msgs::msg::Float32 out;
    out.data = static_cast<float>(v_ema_);
    pub_->publish(out);

    RCLCPP_INFO_THROTTLE(get_logger(), *this->get_clock(), 1000,
                         "v_inst=%.2f m/s, v=%.2f m/s (dt=%.3f s)", v_inst, v_ema_, dt);

    prev_x_ = msg->point.x; prev_y_ = msg->point.y; prev_t_ = t;
  }

  // params
  std::string in_topic_, out_topic_;
  double alpha_, max_speed_, min_dt_;

  // state
  bool has_prev_{false}, has_ema_{false};
  double prev_x_{0.0}, prev_y_{0.0};
  rclcpp::Time prev_t_{0, 0, RCL_ROS_TIME};
  double v_ema_{0.0};

  // ros io
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr pub_;
  rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr sub_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<GPSSpeedNode>());
  rclcpp::shutdown();
  return 0;
}
