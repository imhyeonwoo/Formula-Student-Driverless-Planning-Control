// src/pure_pursuit.cpp
// ──────────────────────────────────────────────────────────────
// • 입력 1: /local_planned_path (nav_msgs/Path)
// • 입력 2: /desired_speed_profile (std_msgs/Float32MultiArray)
// • 입력 3: /now_speed (std_msgs/Float32)  ★ 현재 속도
// • 입력 4: TF (reference -> base_link, base_link -> os_sensor)
//
// • 출력 1: /steering_angle (std_msgs/Float32, [deg])
// • 출력 2: /speed (std_msgs/Float32, [m/s])
// • 출력 3: /lookahead_point_marker (visualization_msgs/Marker)
//
// • 핵심 로직: 속도 비례 동적 Ld를 적용한 Pure Pursuit 알고리즘
// ──────────────────────────────────────────────────────────────

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/path.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <vector>
#include <cmath>
#include <algorithm>
#include <string>
#include <memory>

#include <visualization_msgs/msg/marker.hpp>

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
    // ★ 동적 Ld 파라미터 추가
    this->declare_parameter<double>("ld_gain", 0.5); // Ld = k * speed 의 k(비례 상수)
    this->declare_parameter<double>("min_lookahead_distance", 2.0); // 최소 Ld
    this->declare_parameter<double>("max_lookahead_distance", 10.0); // 최대 Ld

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
    speed_profile_sub_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
      "/desired_speed_profile", 10,
      std::bind(&PurePursuit::speedProfileCallback, this, std::placeholders::_1)
    );
    // ★ 현재 속도 구독자 추가
    current_speed_sub_ = this->create_subscription<std_msgs::msg::Float32>(
      "/now_speed", 10,
      std::bind(&PurePursuit::currentSpeedCallback, this, std::placeholders::_1)
    );

    // ===== 퍼블리셔 =====
    steering_pub_ = this->create_publisher<std_msgs::msg::Float32>("/steering_angle", 10);
    speed_pub_ = this->create_publisher<std_msgs::msg::Float32>("/speed", 10);
    lookahead_marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("/lookahead_point_marker", 10);

    // ===== 제어 타이머 =====
    auto control_period = std::chrono::duration<double>(1.0 / control_freq);
    timer_ = this->create_wall_timer(
      control_period,
      std::bind(&PurePursuit::controlLoop, this)
    );

    RCLCPP_INFO(this->get_logger(), "Pure Pursuit node started with Dynamic Lookahead Distance.");
    RCLCPP_INFO(this->get_logger(), " - Wheelbase: %.2f m, Ld Gain: %.2f", wheelbase_, ld_gain_);
    RCLCPP_INFO(this->get_logger(), " - Lookahead Distance Range: [%.2f m, %.2f m]", min_ld_, max_ld_);
  }

private:
  void pathCallback(const nav_msgs::msg::Path::SharedPtr msg)
  {
    current_path_ = msg;
  }

  void speedProfileCallback(const std_msgs::msg::Float32MultiArray::SharedPtr msg)
  {
    current_speed_profile_ = msg->data;
  }
  
  // ★ 현재 속도를 수신하여 멤버 변수에 저장하는 콜백 함수
  void currentSpeedCallback(const std_msgs::msg::Float32::SharedPtr msg)
  {
    current_speed_ = msg->data;
  }
  
  void publishLookaheadMarker(int target_idx)
  {
    visualization_msgs::msg::Marker clear_marker;
    if(current_path_)
        clear_marker.header.frame_id = current_path_->header.frame_id;
    else
        clear_marker.header.frame_id = "os_sensor";
        
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

    lookahead_marker.color.r = 1.0f;
    lookahead_marker.color.g = 0.0f;
    lookahead_marker.color.b = 1.0f;
    lookahead_marker.color.a = 1.0f;

    lookahead_marker.lifetime = rclcpp::Duration::from_seconds(0.2);
    lookahead_marker_pub_->publish(lookahead_marker);
  }

  void controlLoop()
  {
    if (!current_path_ || current_speed_profile_.empty() || current_path_->poses.empty()) {
      publishLookaheadMarker(-1);
      RCLCPP_WARN_ONCE(this->get_logger(), "Waiting for path and speed profile...");
      return;
    }

    // ★ 1. 현재 속도 기반으로 동적 Ld 계산
    double dynamic_ld = std::clamp(ld_gain_ * current_speed_, min_ld_, max_ld_);

    geometry_msgs::msg::TransformStamped t;
    const std::string& path_frame = current_path_->header.frame_id;
    try {
      t = tf_buffer_->lookupTransform("base_link", path_frame, tf2::TimePointZero);
    } catch (const tf2::TransformException & ex) {
      RCLCPP_ERROR(this->get_logger(), "Could not transform %s to base_link: %s", path_frame.c_str(), ex.what());
      return;
    }

    std::vector<geometry_msgs::msg::Point> transformed_path;
    for (const auto& pose_stamped : current_path_->poses) {
      geometry_msgs::msg::PoseStamped transformed_pose;
      tf2::doTransform(pose_stamped, transformed_pose, t);
      transformed_path.push_back(transformed_pose.pose.position);
    }

    // ★ 2. 목표 지점 탐색 시 'dynamic_ld' 사용
    int target_idx = -1;
    for (size_t i = 0; i < transformed_path.size(); ++i) {
      if (transformed_path[i].x > 0) {
        double dist = std::hypot(transformed_path[i].x, transformed_path[i].y);
        if (dist >= dynamic_ld) {
          target_idx = i;
          break;
        }
      }
    }

    if (target_idx == -1 && !transformed_path.empty()) {
      double max_dist = -1.0;
      for (size_t i = 0; i < transformed_path.size(); ++i) {
        if (transformed_path[i].x > 0) {
          double dist_from_car = std::hypot(transformed_path[i].x, transformed_path[i].y);
          if (dist_from_car > max_dist) {
            max_dist = dist_from_car;
            target_idx = i;
          }
        }
      }
    }

    publishLookaheadMarker(target_idx);
    
    if (target_idx == -1) {
        RCLCPP_WARN(this->get_logger(), "No valid target point found in front of the vehicle.");
        return;
    }
    
    double target_x = transformed_path[target_idx].x;
    double target_y = transformed_path[target_idx].y;
    double alpha = std::atan2(target_y, target_x);
    // ★ 3. 조향각 계산 시 'dynamic_ld' 사용
    double steering_rad = std::atan2(2.0 * wheelbase_ * std::sin(alpha), dynamic_ld);
    double steering_deg = steering_rad * 180.0 / M_PI;

    int closest_idx = 0;
    double min_dist_sq = std::numeric_limits<double>::max();
    for (size_t i = 0; i < transformed_path.size(); ++i) {
      double dist_sq = transformed_path[i].x * transformed_path[i].x + transformed_path[i].y * transformed_path[i].y;
      if (dist_sq < min_dist_sq) {
        min_dist_sq = dist_sq;
        closest_idx = i;
      }
    }
    
    float target_speed = 0.0f;
    if (closest_idx < static_cast<int>(current_speed_profile_.size())) {
        target_speed = current_speed_profile_[closest_idx];
    }

    std_msgs::msg::Float32 steering_msg;
    steering_msg.data = static_cast<float>(steering_deg);
    steering_pub_->publish(steering_msg);

    std_msgs::msg::Float32 speed_msg;
    speed_msg.data = target_speed;
    speed_pub_->publish(speed_msg);
  }

  // 멤버 변수
  double wheelbase_;
  
  // ★ 동적 Ld 파라미터 멤버 변수
  double ld_gain_;
  double min_ld_;
  double max_ld_;
  
  // ★ 현재 속도를 저장할 멤버 변수
  double current_speed_{0.0};

  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
  rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr speed_profile_sub_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr current_speed_sub_; // ★ 구독자 멤버
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr steering_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr speed_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr lookahead_marker_pub_;

  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  nav_msgs::msg::Path::ConstSharedPtr current_path_;
  std::vector<float> current_speed_profile_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PurePursuit>());
  rclcpp::shutdown();
  return 0;
}
