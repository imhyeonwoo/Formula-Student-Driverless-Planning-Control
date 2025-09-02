#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32.hpp>
#include <nav_msgs/msg/path.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <std_srvs/srv/trigger.hpp>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>

#include <cmath>
#include <deque>
#include <string>
#include <chrono>

using namespace std::chrono_literals;

class WheelTrajectoryNode : public rclcpp::Node {
public:
  WheelTrajectoryNode() : Node("wheel_trajectory") {
    // ===== Parameters =====
    wheel_base_       = declare_parameter<double>("wheel_base", 1.3);
    mode_             = declare_parameter<std::string>("mode", "predict"); // predict | measure
    steer_limit_deg_  = declare_parameter<double>("steer_limit_deg", 30.0);
    steer_lpf_tau_    = declare_parameter<double>("steer_lpf_tau", 0.10);
    v_min_            = declare_parameter<double>("v_min", 0.03);
    rate_hz_          = declare_parameter<double>("rate_hz", 50.0);
    omega_eps_        = declare_parameter<double>("omega_eps", 1e-6);

    frame_map_        = declare_parameter<std::string>("frame_map", "map");
    frame_base_       = declare_parameter<std::string>("frame_base", "base_link");
    topic_speed_      = declare_parameter<std::string>("topic_speed", "/current_speed");
    topic_steer_      = declare_parameter<std::string>("topic_steer", "/cmd/steer");
    topic_global_yaw_ = declare_parameter<std::string>("topic_global_yaw", "/global_yaw/complementary");

    path_topic_       = declare_parameter<std::string>("path_topic", "/wheel_path");
    min_dist_         = declare_parameter<double>("min_dist", 0.05);
    min_yaw_deg_      = declare_parameter<double>("min_yaw_deg", 0.5);
    max_points_       = declare_parameter<int>("max_points", 3000);

    publish_vis_odom_ = declare_parameter<bool>("publish_vis_odom", true);
    vis_odom_topic_   = declare_parameter<std::string>("vis_odom_topic", "/wheel_odom_vis");

    // ===== TF Buffer/Listener =====
    tf_buffer_   = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    // ===== Subscribers =====
    sub_speed_ = create_subscription<std_msgs::msg::Float32>(
      topic_speed_, rclcpp::QoS(50),
      [this](const std_msgs::msg::Float32::SharedPtr m){
        v_mps_ = m->data;
        got_speed_ = true;
      });

    sub_steer_ = create_subscription<std_msgs::msg::Float32>(
      topic_steer_, rclcpp::QoS(50),
      [this](const std_msgs::msg::Float32::SharedPtr m){
        // deg(+좌, -우) -> rad
        double deg = static_cast<double>(m->data);
        // clamp
        if (deg >  steer_limit_deg_) deg =  steer_limit_deg_;
        if (deg < -steer_limit_deg_) deg = -steer_limit_deg_;
        delta_raw_rad_ = deg * M_PI / 180.0;
        got_steer_ = true;
      });

    // measure 모드에서 사용할 전역 yaw
    if (mode_ == "measure") {
      sub_global_yaw_ = create_subscription<std_msgs::msg::Float32>(
        topic_global_yaw_, rclcpp::QoS(100),
        [this](const std_msgs::msg::Float32::SharedPtr m){
          yaw_global_rad_ = m->data;
          got_global_yaw_ = true;
        });
    }

    // ===== Publishers =====
    pub_path_ = create_publisher<nav_msgs::msg::Path>(path_topic_, 1);
    if (publish_vis_odom_) {
      pub_odom_vis_ = create_publisher<nav_msgs::msg::Odometry>(vis_odom_topic_, 10);
    }

    // ===== Service: reset anchor =====
    srv_reset_anchor_ = create_service<std_srvs::srv::Trigger>(
      "/wheel_path/reset_anchor",
      std::bind(&WheelTrajectoryNode::onResetAnchor, this,
                std::placeholders::_1, std::placeholders::_2));

    // Init Path header
    path_msg_.header.frame_id = frame_map_;

    // Timer
    double dt_ms = std::max(1.0, 1000.0 / std::max(1.0, rate_hz_));
    timer_ = create_wall_timer(
      std::chrono::milliseconds(static_cast<int>(dt_ms)),
      std::bind(&WheelTrajectoryNode::onTimer, this));

    RCLCPP_INFO(get_logger(),
      "wheel_trajectory started. mode=%s, frames: map=%s, base=%s, speed=%s, steer=%s, yaw=%s",
      mode_.c_str(), frame_map_.c_str(), frame_base_.c_str(),
      topic_speed_.c_str(), topic_steer_.c_str(), topic_global_yaw_.c_str());
  }

private:
  // ===== Helpers =====
  static double wrapToPi(double a){
    while (a >  M_PI) a -= 2.0*M_PI;
    while (a < -M_PI) a += 2.0*M_PI;
    return a;
  }

  static double quatToYaw(const geometry_msgs::msg::Quaternion &q){
    tf2::Quaternion tq(q.x, q.y, q.z, q.w);
    double roll, pitch, yaw;
    tf2::Matrix3x3(tq).getRPY(roll, pitch, yaw);
    return yaw;
  }

  static geometry_msgs::msg::Quaternion yawToQuat(double yaw){
    tf2::Quaternion q; q.setRPY(0,0,yaw);
    geometry_msgs::msg::Quaternion qq;
    qq.x = q.x(); qq.y = q.y(); qq.z = q.z(); qq.w = q.w();
    return qq;
  }

  bool fetchAnchorFromTF(double &x0, double &y0, double &yaw0){
    // 최신 TF: map -> base_link
    try {
      // rclcpp::Time(0) == latest available transform
      auto ts = tf_buffer_->lookupTransform(frame_map_, frame_base_, tf2::TimePointZero);
      x0 = ts.transform.translation.x;
      y0 = ts.transform.translation.y;
      yaw0 = quatToYaw(ts.transform.rotation);
      return true;
    } catch (const tf2::TransformException &ex) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "TF lookup failed: %s", ex.what());
      return false;
    }
  }

  void addPathPointIfNeeded(const rclcpp::Time &stamp){
    if (path_msg_.poses.empty()){
      geometry_msgs::msg::PoseStamped p;
      p.header.stamp = stamp;
      p.header.frame_id = frame_map_;
      p.pose.position.x = x_world_;
      p.pose.position.y = y_world_;
      p.pose.position.z = 0.0;
      p.pose.orientation = yawToQuat(yaw_world_);
      path_msg_.poses.push_back(p);
      return;
    }
    const auto &last = path_msg_.poses.back().pose;
    double dx = x_world_ - last.position.x;
    double dy = y_world_ - last.position.y;
    double dist = std::hypot(dx, dy);

    double yaw_last = quatToYaw(last.orientation);
    double dyaw_deg = std::fabs((yaw_world_ - yaw_last) * 180.0 / M_PI);

    if (dist >= min_dist_ || dyaw_deg >= min_yaw_deg_){
      geometry_msgs::msg::PoseStamped p;
      p.header.stamp = stamp;
      p.header.frame_id = frame_map_;
      p.pose.position.x = x_world_;
      p.pose.position.y = y_world_;
      p.pose.position.z = 0.0;
      p.pose.orientation = yawToQuat(yaw_world_);
      path_msg_.poses.push_back(p);

      // size cap
      if (static_cast<int>(path_msg_.poses.size()) > max_points_){
        int remove_n = static_cast<int>(path_msg_.poses.size()) - max_points_;
        path_msg_.poses.erase(path_msg_.poses.begin(), path_msg_.poses.begin() + remove_n);
      }
    }
  }

  // ===== Service: reset anchor =====
  void onResetAnchor(const std::shared_ptr<std_srvs::srv::Trigger::Request>,
                     std::shared_ptr<std_srvs::srv::Trigger::Response> res){
    double x0, y0, yaw0;
    if (!fetchAnchorFromTF(x0, y0, yaw0)){
      res->success = false;
      res->message = "Failed to fetch anchor TF(map->base_link).";
      return;
    }
    x_world_ = x0; y_world_ = y0; yaw_world_ = yaw0;
    last_time_ = this->now();
    path_msg_.poses.clear(); // 경로 리셋
    addPathPointIfNeeded(last_time_);
    anchor_ready_ = true;
    res->success = true;
    res->message = "Anchor reset OK.";
    RCLCPP_INFO(get_logger(), "Anchor reset at (%.3f, %.3f, yaw=%.3f rad).",
                x_world_, y_world_, yaw_world_);
  }

  // ===== Main timer =====
  void onTimer(){
    const auto tnow = this->now();

    // 1) 입력 토픽 최소 1회 수신 확인
    if (!got_speed_ || !got_steer_){
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 3000,
        "Waiting inputs... speed=%d steer=%d", got_speed_, got_steer_);
      last_time_ = tnow;
      return;
    }
    if (mode_ == "measure" && !got_global_yaw_){
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 3000,
        "measure mode: waiting for global yaw...");
      last_time_ = tnow;
      return;
    }

    // 2) 앵커 설정 (1회)
    if (!anchor_ready_){
      double x0,y0,yaw0;
      if (!fetchAnchorFromTF(x0,y0,yaw0)){
        // 아직 TF 미준비
        return;
      }
      x_world_ = x0; y_world_ = y0; yaw_world_ = yaw0;
      last_time_ = tnow;
      path_msg_.poses.clear();
      addPathPointIfNeeded(tnow);
      anchor_ready_ = true;
      RCLCPP_INFO(get_logger(),
        "Anchor set at start: (%.3f, %.3f, yaw=%.3f rad).", x_world_, y_world_, yaw_world_);
      return; // 다음 틱부터 적분
    }

    // 3) dt
    double dt = (tnow - last_time_).seconds();
    if (dt <= 0.0 || dt > 0.25) { // 너무 큰 간격은 스킵(로그만)
      last_time_ = tnow;
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
        "Abnormal dt=%.3f s, skipping this tick.", dt);
      return;
    }
    last_time_ = tnow;

    // 4) 입력 정리
    double v = v_mps_;
    if (std::fabs(v) < v_min_) v = 0.0;

    // 조향 LPF
    double delta = delta_raw_rad_;
    if (steer_lpf_tau_ > 1e-6){
      double alpha = std::exp(-dt / steer_lpf_tau_);
      delta_filt_rad_ = alpha * delta_filt_rad_ + (1.0 - alpha) * delta;
    } else {
      delta_filt_rad_ = delta;
    }

    // 5) yaw/위치 적분
    if (mode_ == "predict"){
      // Ackermann bicycle model
      double L = std::max(1e-6, wheel_base_);
      double omega = (v / L) * std::tan(delta_filt_rad_); // yaw rate
      if (std::fabs(omega) < omega_eps_){
        // straight
        x_world_ += v * std::cos(yaw_world_) * dt;
        y_world_ += v * std::sin(yaw_world_) * dt;
        // yaw unchanged (or small update)
        yaw_world_ = wrapToPi(yaw_world_ + omega * dt);
      } else {
        // arc
        double dtheta = omega * dt;
        double R = v / omega; // = L / tan(delta)
        double x_new = x_world_ + R * (std::sin(yaw_world_ + dtheta) - std::sin(yaw_world_));
        double y_new = y_world_ - R * (std::cos(yaw_world_ + dtheta) - std::cos(yaw_world_));
        x_world_ = x_new;
        y_world_ = y_new;
        yaw_world_ = wrapToPi(yaw_world_ + dtheta);
      }
    } else { // measure
      // yaw는 글로벌 yaw를 직접 사용(없으면 이전값 유지)
      double yaw_meas = got_global_yaw_ ? yaw_global_rad_ : yaw_world_;
      yaw_world_ = wrapToPi(yaw_meas);
      x_world_ += v * std::cos(yaw_world_) * dt;
      y_world_ += v * std::sin(yaw_world_) * dt;
    }

    // 6) Path 관리 & 퍼블리시
    path_msg_.header.stamp = tnow;
    addPathPointIfNeeded(tnow);
    pub_path_->publish(path_msg_);

    // 7) (옵션) Odometry 시각화
    if (publish_vis_odom_) {
      nav_msgs::msg::Odometry od;
      od.header.stamp = tnow;
      od.header.frame_id = frame_map_;
      od.child_frame_id = "base_link"; // TF는 브로드캐스트하지 않음 (시각화만)
      od.pose.pose.position.x = x_world_;
      od.pose.pose.position.y = y_world_;
      od.pose.pose.position.z = 0.0;
      od.pose.pose.orientation = yawToQuat(yaw_world_);
      od.twist.twist.linear.x  = v_mps_;
      // 추정 yaw rate (predict 기준으로 계산 가능)
      double L = std::max(1e-6, wheel_base_);
      double omega_est = (v_mps_ / L) * std::tan(delta_filt_rad_);
      od.twist.twist.angular.z = omega_est;
      pub_odom_vis_->publish(od);
    }
  }

private:
  // Params
  double wheel_base_, steer_limit_deg_, steer_lpf_tau_, v_min_, rate_hz_, omega_eps_;
  std::string mode_, frame_map_, frame_base_, topic_speed_, topic_steer_, topic_global_yaw_;
  std::string path_topic_, vis_odom_topic_;
  double min_dist_, min_yaw_deg_;
  int max_points_;
  bool publish_vis_odom_;

  // TF
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  // Subs/Pubs
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr sub_speed_, sub_steer_, sub_global_yaw_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pub_path_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_odom_vis_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr srv_reset_anchor_;

  // Timer
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Time last_time_;

  // State
  bool anchor_ready_{false};
  bool got_speed_{false}, got_steer_{false}, got_global_yaw_{false};

  double v_mps_{0.0};
  double delta_raw_rad_{0.0};
  double delta_filt_rad_{0.0};
  double yaw_global_rad_{0.0};

  double x_world_{0.0}, y_world_{0.0}, yaw_world_{0.0};

  nav_msgs::msg::Path path_msg_;
};

int main(int argc, char** argv){
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<WheelTrajectoryNode>());
  rclcpp::shutdown();
  return 0;
}
