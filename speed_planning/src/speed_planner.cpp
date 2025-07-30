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

        // Get parameter values (or defaults if not set)
        this->get_parameter("a_long_max", a_long_max_);
        this->get_parameter("a_brake_max", a_brake_max_);
        this->get_parameter("a_lat_max", a_lat_max_);

        // Set up publisher for the desired speed profile (list of speeds)
        speed_pub_ = this->create_publisher<std_msgs::msg::Float32MultiArray>("/desired_speed_profile", 10);

        // Set up subscriber for the local planned path (sequence of PoseStamped)
        path_sub_ = this->create_subscription<nav_msgs::msg::Path>(
            "/local_planned_path", 10,
            std::bind(&SpeedPlanner::pathCallback, this, std::placeholders::_1)
        );
    }

private:
    void pathCallback(const nav_msgs::msg::Path::SharedPtr msg) {
        const auto & poses = msg->poses;
        size_t n = poses.size();
        if (n == 0) {
            // No path received, nothing to do
            return;
        }

        // If path has only one point, speed is zero (we're at a goal or no movement)
        if (n == 1) {
            std_msgs::msg::Float32MultiArray speed_msg;
            speed_msg.data.resize(1);
            speed_msg.data[0] = 0.0f;
            speed_pub_->publish(speed_msg);
            return;
        }

        // Prepare vectors for computed speeds
        std::vector<double> curvature_speed(n);  // speed limited by curvature (lateral accel)
        std::vector<double> speed(n);            // final speed profile (will be updated)
        std::vector<double> dist(n);             // distances between consecutive points

        // Compute distances between consecutive path points
        for (size_t i = 0; i < n - 1; ++i) {
            double dx = poses[i+1].pose.position.x - poses[i].pose.position.x;
            double dy = poses[i+1].pose.position.y - poses[i].pose.position.y;
            double dz = poses[i+1].pose.position.z - poses[i].pose.position.z;
            dist[i] = std::sqrt(dx*dx + dy*dy + dz*dz);
            if (dist[i] < 1e-6) {
                dist[i] = 1e-6;  // avoid zero distance (to prevent division by zero later)
            }
        }
        dist[n-1] = 0.0;  // no segment after the last point

        // Compute curvature-based speed limits for each point (except endpoints)
        // Using the formula v_max = sqrt(a_lat_max / curvature).
        // We estimate curvature using three consecutive points (i-1, i, i+1).
        curvature_speed[0] = std::numeric_limits<double>::infinity();
        curvature_speed[n-1] = std::numeric_limits<double>::infinity();
        for (size_t i = 1; i < n - 1; ++i) {
            // Coordinates of three consecutive points
            double x0 = poses[i-1].pose.position.x;
            double y0 = poses[i-1].pose.position.y;
            double x1 = poses[i].pose.position.x;
            double y1 = poses[i].pose.position.y;
            double x2 = poses[i+1].pose.position.x;
            double y2 = poses[i+1].pose.position.y;

            // Compute the curvature (kappa) via circle through the three points
            double dx1 = x1 - x0;
            double dy1 = y1 - y0;
            double dx2 = x2 - x0;
            double dy2 = y2 - y0;
            // Cross product magnitude (twice the area of the triangle)
            double cross = std::abs(dx1 * (y2 - y0) - dy1 * (x2 - x0));
            // Distances between the points
            double d01 = std::sqrt(dx1*dx1 + dy1*dy1);
            double d02 = std::sqrt(dx2*dx2 + dy2*dy2);
            double d12 = std::sqrt((x2 - x1)*(x2 - x1) + (y2 - y1)*(y2 - y1));

            // Ensure no division by zero in curvature calculation
            if (d01 < 1e-6 || d02 < 1e-6 || d12 < 1e-6) {
                curvature_speed[i] = std::numeric_limits<double>::infinity();
            } else {
                // Curvature k = 2 * area / (d01 * d02 * d12)
                double curvature = (2 * cross) / (d01 * d02 * d12);
                // Limit speed by lateral acceleration: v_max = sqrt(a_lat_max / kappa)
                // If curvature is very small (near zero), v_max becomes very large (treat as essentially infinite or capped by other limits)
                if (curvature < 1e-9) {
                    curvature_speed[i] = std::numeric_limits<double>::infinity();
                } else {
                    curvature_speed[i] = std::sqrt(a_lat_max_ / curvature);
                }
            }
        }

        // Initialize speed profile with curvature limits (start with curvature-based limits or 0 if infinite)
        for (size_t i = 0; i < n; ++i) {
            if (std::isfinite(curvature_speed[i])) {
                speed[i] = curvature_speed[i];
            } else {
                // If curvature_speed is infinite (straight path), set to a very high value (to be constrained by accel/decel later)
                speed[i] = 1e6;  // effectively "no limit" (could also use a global max speed if defined)
            }
        }
        // Optionally, we could define a global max speed here to clamp speed[i] if needed.

        // Forward pass: enforce acceleration (a_long_max_) limits
        // Start at the first point. Assume starting speed is current speed (or 0 if starting from stop).
        // Here we assume start from 0 for simplicity. If current speed is known, it can be used instead.
        speed[0] = 0.0;
        for (size_t i = 1; i < n; ++i) {
            // Maximum speed reachable from speed[i-1] with acceleration a_long_max over distance dist[i-1]
            double v_accel_limit = std::sqrt(speed[i-1]*speed[i-1] + 2 * a_long_max_ * dist[i-1]);
            // Constrain by curvature limit at i
            if (speed[i] > v_accel_limit) {
                speed[i] = v_accel_limit;
            }
        }

        // Backward pass: enforce deceleration (a_brake_max_) limits
        for (int i = static_cast<int>(n) - 2; i >= 0; --i) {
            // Maximum speed at i that allows deceleration to speed[i+1] over distance dist[i]
            double v_decel_limit = std::sqrt(speed[i+1]*speed[i+1] + 2 * a_brake_max_ * dist[i]);
            if (speed[i] > v_decel_limit) {
                speed[i] = v_decel_limit;
            }
        }

        // Smooth the speed profile using a 3-point moving average filter [weights 0.25, 0.5, 0.25]
        std::vector<double> smooth_speed(n);
        if (n >= 3) {
            // Leave the endpoints unchanged (cannot apply 3-point average at the ends)
            smooth_speed[0] = speed[0];
            smooth_speed[n-1] = speed[n-1];
            for (size_t i = 1; i < n-1; ++i) {
                smooth_speed[i] = 0.25 * speed[i-1] + 0.5 * speed[i] + 0.25 * speed[i+1];
                // Ensure the smoothed speed does not exceed original speed limit at i
                if (smooth_speed[i] > speed[i]) {
                    smooth_speed[i] = speed[i];
                }
            }
        } else {
            // Path too short for smoothing, copy original speeds
            smooth_speed = speed;
        }

        // Publish the final desired speed profile
        std_msgs::msg::Float32MultiArray speed_msg;
        speed_msg.data.reserve(n);
        for (size_t i = 0; i < n; ++i) {
            // Cast to float for publishing
            speed_msg.data.push_back(static_cast<float>(smooth_speed[i]));
        }
        speed_pub_->publish(speed_msg);
    }

    // ROS 2 publisher and subscriber
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr speed_pub_;

    // Parameters for acceleration limits
    double a_long_max_;
    double a_brake_max_;
    double a_lat_max_;
};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<SpeedPlanner>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
