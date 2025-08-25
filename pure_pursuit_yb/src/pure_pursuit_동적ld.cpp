// src/pure_pursuit.cpp
// ──────────────────────────────────────────────────────────────
// • 입력 1: /local_planned_path (nav_msgs/Path)
// • 입력 2: /now_speed (std_msgs/Float32)  ★ 현재 속도
// • 입력 3: TF (reference -> base_link, base_link -> os_sensor)
//
// • 출력 1: /steering_angle (std_msgs/Float32, [deg])
// • 출력 2: /lookahead_point_marker (visualization_msgs/Marker)
//
// • 핵심 로직: 속도 비례 동적 Ld를 적용한 Pure Pursuit 알고리즘
// ──────────────────────────────────────────────────────────────

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/path.hpp>
#include <std_msgs/msg/float32.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <visualization_msgs/msg/marker.hpp>

#include <vector>
#include <cmath>
#include <string>
#include <memory>
#include <algorithm>  // std::clamp

using namespace std::chrono_literals;

class PurePursuit : public rclcpp::Node
{
public:
  PurePursuit()
  : Node("pure_pursuit")
  {
    // ===== 파라미터 =====
    this->declare_parameter<double>("wheelbase", 2.6);
    this->declare_parameter<int>("control_frequency", 30);
    // ★ 동적 Ld 파라미터
    this->declare_parameter<double>("ld_gain", 0.5);             // Ld = k * speed
    this->declare_parameter<double>("min_lookahead_distance", 2.0);
    this->declare_parameter<double>("max_lookahead_distance", 10.0);

    wheelbase_ = this->get_parameter("wheelbase").as_double();
    int control_freq = this->get_parameter("control_frequency").as_int();
    ld_gain_ = this->get_parameter("ld_gain").as_double();
    min_ld_ = this->get_parameter("min_lookahead_distance").as_double();
    max_ld_ = this->get_parameter("max_lookahead_distance").as_double();

    // TF 리스너 및 버퍼 초기화
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    // ===== 구독자 =====
    path_sub_ = this->create_subscription<nav_msgs::msg::Path>(
      "/local_planned_path", 10,
      std::bind(&PurePursuit::pathCallback, this, std::placeholders::_1)
    );
    // ★ 현재 속도 구독
    current_speed_sub_ = this->create_subscription<std_msgs::msg::Float32>(
      "/now_speed", 10,
      std::bind(&PurePursuit::currentSpeedCallback, this, std::placeholders::_1)
    );

    // ===== 퍼블리셔 =====
    steering_pub_ = this->create_publisher<std_msgs::msg::Float32>("/steering_angle", 10);
    lookahead_marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("/lookahead_point_marker", 10);

    // ===== 제어 타이머 =====
    auto control_period = std::chrono::duration<double>(1.0 / control_freq);
    timer_ = this->create_wall_timer(
      control_period,
      std::bind(&PurePursuit::controlLoop, this)
    );

    RCLCPP_INFO(this->get_logger(), "Pure Pursuit node started (Dynamic Ld).");
    RCLCPP_INFO(this->get_logger(), " - Wheelbase: %.2f m, Ld Gain: %.2f", wheelbase_, ld_gain_);
    RCLCPP_INFO(this->get_logger(), " - Lookahead Distance Range: [%.2f m, %.2f m]", min_ld_, max_ld_);
  }

private:
  void pathCallback(const nav_msgs::msg::Path::SharedPtr msg)
  {
    current_path_ = msg;
  }

  // ★ 현재 속도를 저장
  void currentSpeedCallback(const std_msgs::msg::Float32::SharedPtr msg)
  {
    current_speed_ = msg->data;
  }

  void publishLookaheadMarker(int target_idx)
  {
    visualization_msgs::msg::Marker clear_marker;
    clear_marker.header.frame_id = (current_path_) ? current_path_->header.frame_id : "os_sensor";
    clear_marker.header.stamp = this->now();
    clear_marker.ns = "pure_pursuit_lookahead";
    clear_marker.action = visualization_msgs::msg::Marker::DELETEALL;
    lookahead_marker_pub_->publish(clear_marker);

    if (target_idx < 0) return;

    visualization_msgs::msg::Marker lookahead_marker;
    lookahead_marker.header.frame_id = current_path_->header.frame_id;
    lookahead_marker.header.stamp = this->now();
    lookahead_marker.ns = "pure_pursuit_lookahead";
    lookahead_marker.id = 0;
    lookahead_marker.type = visualization_msgs::msg::Marker::SPHERE;
    lookahead_marker.action = visualization_msgs::msg::Marker::ADD;

    lookahead_marker.pose.position = current_path_->poses[target_idx].pose.position;
    lookahead_marker.pose.orientation.w = 1.0;

    lookahead_marker.scale.x = 0.5;
    lookahead_marker.scale.y = 0.5;
    lookahead_marker.scale.z = 0.5;

    // 색상: 마젠타
    lookahead_marker.color.r = 1.0f;
    lookahead_marker.color.g = 0.0f;
    lookahead_marker.color.b = 1.0f;
    lookahead_marker.color.a = 1.0f;

    lookahead_marker.lifetime = rclcpp::Duration::from_seconds(0.2);
    lookahead_marker_pub_->publish(lookahead_marker);
  }

  void controlLoop()
  {
    if (!current_path_ || current_path_->poses.empty()) {
      publishLookaheadMarker(-1);
      RCLCPP_WARN_ONCE(this->get_logger(), "Waiting for path...");
      return;
    }

    // ★ 1) 현재 속도로 동적 Ld 계산 (수신 전이면 current_speed_=0 → min_ld_ 사용)
    double dynamic_ld = std::clamp(ld_gain_ * current_speed_, min_ld_, max_ld_);

    // TF: path_frame → base_link
    geometry_msgs::msg::TransformStamped t;
    const std::string& path_frame = current_path_->header.frame_id;
    try {
      t = tf_buffer_->lookupTransform("base_link", path_frame, tf2::TimePointZero);
    } catch (const tf2::TransformException & ex) {
      RCLCPP_ERROR(this->get_logger(), "Could not transform %s to base_link: %s", path_frame.c_str(), ex.what());
      return;
    }

    // 경로를 base_link로 변환
    std::vector<geometry_msgs::msg::Point> transformed_path;
    transformed_path.reserve(current_path_->poses.size());
    for (const auto& pose_stamped : current_path_->poses) {
      geometry_msgs::msg::PoseStamped transformed_pose;
      tf2::doTransform(pose_stamped, transformed_pose, t);
      transformed_path.push_back(transformed_pose.pose.position);
    }

    // 2) 룩어헤드 타겟 인덱스 선택 (dynamic_ld 사용)
    int target_idx = -1;
    for (size_t i = 0; i < transformed_path.size(); ++i) {
      if (transformed_path[i].x > 0) {
        double dist = std::hypot(transformed_path[i].x, transformed_path[i].y);
        if (dist >= dynamic_ld) {
          target_idx = static_cast<int>(i);
          break;
        }
      }
    }
    if (target_idx == -1 && !transformed_path.empty()) {
      double max_dist = -1.0;
      for (size_t i = 0; i < transformed_path.size(); ++i) {
        if (transformed_path[i].x > 0) {
          double d = std::hypot(transformed_path[i].x, transformed_path[i].y);
          if (d > max_dist) {
            max_dist = d;
            target_idx = static_cast<int>(i);
          }
        }
      }
    }

    publishLookaheadMarker(target_idx);

    if (target_idx == -1) {
      RCLCPP_WARN(this->get_logger(), "No valid target point found in front of the vehicle.");
      return;
    }

    // 3) Pure Pursuit 조향각 계산 (dynamic_ld 사용)
    double target_x = transformed_path[target_idx].x;
    double target_y = transformed_path[target_idx].y;

    double alpha = std::atan2(target_y, target_x);
    double steering_rad = std::atan2(2.0 * wheelbase_ * std::sin(alpha), dynamic_ld);
    double steering_deg = steering_rad * 180.0 / M_PI;

    std_msgs::msg::Float32 steering_msg;
    steering_msg.data = static_cast<float>(steering_deg);
    steering_pub_->publish(steering_msg);
  }

  // 멤버 변수
  double wheelbase_;
  // 동적 Ld 파라미터
  double ld_gain_;
  double min_ld_;
  double max_ld_;
  // 현재 속도
  double current_speed_{0.0};

  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr current_speed_sub_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr steering_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr lookahead_marker_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  nav_msgs::msg::Path::ConstSharedPtr current_path_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PurePursuit>());
  rclcpp::shutdown();
  return 0;
}

