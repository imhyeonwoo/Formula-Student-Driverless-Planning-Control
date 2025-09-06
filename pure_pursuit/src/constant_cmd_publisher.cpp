// src/Planning/pure_pursuit/src/constant_cmd_publisher.cpp
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32.hpp>
#include <string>
#include <chrono>

class ConstantCmdPublisher : public rclcpp::Node {
public:
  ConstantCmdPublisher() : rclcpp::Node("constant_cmd_publisher") {
    speed_topic_       = declare_parameter<std::string>("speed_topic", "/cmd/speed");
    rpm_topic_         = declare_parameter<std::string>("rpm_topic", "/cmd/rpm");
    publish_rate_hz_   = declare_parameter<double>("publish_rate_hz", 50.0);
    speed_value_       = declare_parameter<double>("speed_value", 0.0);
    rpm_value_         = declare_parameter<double>("rpm_value", 0.0);
    // Optional: compute rpm from speed_value using a divisor (e.g., 0.00535)
    rpm_from_speed_    = declare_parameter<bool>("rpm_from_speed", false);
    rpm_divisor_       = declare_parameter<double>("rpm_divisor", 1.0); // rpm = speed_value / rpm_divisor
    publish_speed_     = declare_parameter<bool>("publish_speed", true);
    publish_rpm_       = declare_parameter<bool>("publish_rpm", true);

    if (publish_speed_) {
      pub_speed_ = create_publisher<std_msgs::msg::Float32>(speed_topic_, rclcpp::QoS(10));
    }
    if (publish_rpm_) {
      pub_rpm_ = create_publisher<std_msgs::msg::Float32>(rpm_topic_, rclcpp::QoS(10));
    }

    using namespace std::chrono_literals;
    const auto period = std::chrono::duration<double>(1.0 / std::max(1e-3, publish_rate_hz_));
    timer_ = create_wall_timer(std::chrono::duration_cast<std::chrono::milliseconds>(period),
              std::bind(&ConstantCmdPublisher::onTimer, this));

    RCLCPP_INFO(get_logger(),
      "ConstantCmdPublisher started: speed_topic='%s'(%.3f), rpm_topic='%s'(%s%.3f), rate=%.1f Hz",
      speed_topic_.c_str(), speed_value_, rpm_topic_.c_str(),
      rpm_from_speed_ ? "derived " : "",
      rpm_from_speed_ ? ( (rpm_divisor_!=0.0) ? (speed_value_/rpm_divisor_) : 0.0 ) : rpm_value_,
      publish_rate_hz_);
  }

private:
  void onTimer() {
    if (publish_speed_ && pub_speed_) {
      std_msgs::msg::Float32 m; m.data = static_cast<float>(speed_value_);
      pub_speed_->publish(m);
    }
    if (publish_rpm_ && pub_rpm_) {
      float rpm_out = static_cast<float>(rpm_value_);
      if (rpm_from_speed_) {
        rpm_out = static_cast<float>( (rpm_divisor_ != 0.0) ? (speed_value_ / rpm_divisor_) : 0.0f );
      }
      std_msgs::msg::Float32 m; m.data = rpm_out;
      pub_rpm_->publish(m);
    }
  }

  // Params
  std::string speed_topic_;
  std::string rpm_topic_;
  double publish_rate_hz_{};
  double speed_value_{};
  double rpm_value_{};
  bool   rpm_from_speed_{false};
  double rpm_divisor_{1.0};
  bool publish_speed_{true};
  bool publish_rpm_{true};

  // ROS
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr pub_speed_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr pub_rpm_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ConstantCmdPublisher>());
  rclcpp::shutdown();
  return 0;
}
