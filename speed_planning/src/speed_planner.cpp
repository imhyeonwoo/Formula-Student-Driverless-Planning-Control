// speed_planner.cpp - ROS 2 Node for curvature-based local speed planning
#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <vector>
#include <algorithm>
#include <cmath>
#include <limits>

class SpeedPlanner : public rclcpp::Node {
public:
    SpeedPlanner() : rclcpp::Node("speed_planner") {
        // Declare ROS 2 parameters with default values
        this->declare_parameter<double>("a_long_max", 3.0);   // max forward acceleration (m/s^2)
        this->declare_parameter<double>("a_brake_max", 3.5);  // max braking deceleration (m/s^2)
        this->declare_parameter<double>("a_lat_max", 3.0);    // max lateral acceleration for turns (m/s^2)
        this->declare_parameter<double>("diff_eps",  0.05);   // min change to republish (m/s)

        // Get parameter values (or defaults if not set)
        this->get_parameter("a_long_max",  a_long_max_);
        this->get_parameter("a_brake_max", a_brake_max_);
        this->get_parameter("a_lat_max",   a_lat_max_);
        this->get_parameter("diff_eps",    diff_eps_);

        // Set up publisher for the desired speed profile (list of speeds)
        speed_pub_ = this->create_publisher<std_msgs::msg::Float32MultiArray>(
            "/desired_speed_profile", 10);

        // Set up subscriber for the local planned path (sequence of PoseStamped)
        path_sub_ = this->create_subscription<nav_msgs::msg::Path>(
            "/local_planned_path", 10,
            std::bind(&SpeedPlanner::pathCallback, this, std::placeholders::_1)
        );
    }

private:
    void pathCallback(const nav_msgs::msg::Path::SharedPtr msg) {
        const auto & poses = msg->poses;
        const size_t n = poses.size();
        if (n == 0) {                         // No path
            return;
        }

        if (n == 1) {                         // Single‑point path
            std_msgs::msg::Float32MultiArray speed_msg;
            speed_msg.data = {0.0f};
            speed_pub_->publish(speed_msg);
            prev_speed_  = {0.0};
            return;
        }

        /* --------- 준비 --------- */
        std::vector<double> curvature_speed(n);
        std::vector<double> speed(n);
        std::vector<double> dist(n);

        /* 거리 계산 */
        for (size_t i = 0; i < n - 1; ++i) {
            double dx = poses[i+1].pose.position.x - poses[i].pose.position.x;
            double dy = poses[i+1].pose.position.y - poses[i].pose.position.y;
            double dz = poses[i+1].pose.position.z - poses[i].pose.position.z;
            dist[i] = std::sqrt(dx*dx + dy*dy + dz*dz);
            if (dist[i] < 1e-6) dist[i] = 1e-6;            // avoid zero
        }
        dist[n-1] = 0.0;

        /* 곡률 기반 속도 한계 */
        curvature_speed.front()  = std::numeric_limits<double>::infinity();
        curvature_speed.back()   = std::numeric_limits<double>::infinity();
        for (size_t i = 1; i < n - 1; ++i) {
            double x0 = poses[i-1].pose.position.x, y0 = poses[i-1].pose.position.y;
            double x1 = poses[i  ].pose.position.x, y1 = poses[i  ].pose.position.y;
            double x2 = poses[i+1].pose.position.x, y2 = poses[i+1].pose.position.y;

            double dx1 = x1 - x0, dy1 = y1 - y0;
            double dx2 = x2 - x0, dy2 = y2 - y0;
            double cross = std::abs(dx1 * (y2 - y0) - dy1 * (x2 - x0));
            double d01 = std::hypot(dx1, dy1);
            double d02 = std::hypot(dx2, dy2);
            double d12 = std::hypot(x2 - x1, y2 - y1);

            if (d01 < 1e-6 || d02 < 1e-6 || d12 < 1e-6) {
                curvature_speed[i] = std::numeric_limits<double>::infinity();
            } else {
                double curvature = (2.0 * cross) / (d01 * d02 * d12);
                curvature_speed[i] =
                    (curvature < 1e-9) ?
                    std::numeric_limits<double>::infinity() :
                    std::sqrt(a_lat_max_ / curvature);
            }
        }

        /* 초기 속도 프로파일 = 곡률 한계 */
        for (size_t i = 0; i < n; ++i)
            speed[i] = std::isfinite(curvature_speed[i]) ? curvature_speed[i] : 1e6;

        /* 가속 한계 (forward pass) */
        speed[0] = 0.0;                       // assume start at 0
        for (size_t i = 1; i < n; ++i) {
            double v_limit = std::sqrt(speed[i-1]*speed[i-1]
                                       + 2.0 * a_long_max_ * dist[i-1]);
            if (speed[i] > v_limit) speed[i] = v_limit;
        }

        /* 감속 한계 (backward pass) */
        for (int i = static_cast<int>(n) - 2; i >= 0; --i) {
            double v_limit = std::sqrt(speed[i+1]*speed[i+1]
                                       + 2.0 * a_brake_max_ * dist[i]);
            if (speed[i] > v_limit) speed[i] = v_limit;
        }

        /* 3‑point moving average */
        std::vector<double> smooth_speed(n);
        if (n >= 3) {
            smooth_speed[0]   = speed[0];
            smooth_speed[n-1] = speed[n-1];
            for (size_t i = 1; i < n-1; ++i) {
                smooth_speed[i] = 0.25*speed[i-1] + 0.5*speed[i] + 0.25*speed[i+1];
                if (smooth_speed[i] > speed[i]) smooth_speed[i] = speed[i];
            }
        } else {
            smooth_speed = speed;
        }

        /* ---------- 변화량 검사 ---------- */
        bool changed = (prev_speed_.size() != n);
        if (!changed) {
            for (size_t i = 0; i < n; ++i) {
                if (std::fabs(smooth_speed[i] - prev_speed_[i]) > diff_eps_) {
                    changed = true; break;
                }
            }
        }
        if (!changed) return;                 // skip publish

        /* 퍼블리시 */
        std_msgs::msg::Float32MultiArray speed_msg;
        speed_msg.data.reserve(n);
        for (double v : smooth_speed)
            speed_msg.data.push_back(static_cast<float>(v));
        speed_pub_->publish(speed_msg);

        /* 캐시 갱신 */
        prev_speed_.swap(smooth_speed);
    }

    /* ---------- ROS 2 I/O ---------- */
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr speed_pub_;

    /* ---------- 파라미터 ---------- */
    double a_long_max_;
    double a_brake_max_;
    double a_lat_max_;
    double diff_eps_;

    /* ---------- 상태 ---------- */
    std::vector<double> prev_speed_;
};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SpeedPlanner>());
    rclcpp::shutdown();
    return 0;
}
