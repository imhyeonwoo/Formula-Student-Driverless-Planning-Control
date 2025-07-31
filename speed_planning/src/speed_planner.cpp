#include <cmath>
#include <functional>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

/**
 * @brief Speed Planner Node
 *
 * This node subscribes to a planned path and the current vehicle speed, 
 * then computes a desired speed profile along that path. The speed profile 
 * is published on the /desired_speed_profile topic, where each element 
 * corresponds to a target speed at the respective point along the path.
 * 
 * The initial speed of the profile is set to the current vehicle speed (if available),
 * which improves planning accuracy when the vehicle is already moving. If the current
 * speed is not yet available, it defaults to 0.0 and a warning is logged.
 * This ensures that the forward acceleration planning accounts for the actual 
 * vehicle speed, allowing safe acceleration planning even when the vehicle is not starting from a stop.
 */
class SpeedPlanner : public rclcpp::Node
{
public:
    SpeedPlanner();

private:
    void pathCallback(const nav_msgs::msg::Path::SharedPtr path);
    void speedCallback(const std_msgs::msg::Float32::SharedPtr msg);

    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr speed_profile_pub_;
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr current_speed_sub_;

    float current_speed_;
    bool current_speed_received_;

    double max_speed_;
    double max_accel_;
    double max_decel_;
};

SpeedPlanner::SpeedPlanner()
: Node("speed_planner"), current_speed_(0.0f), current_speed_received_(false)
{
    // Declare and get parameters for speed limits
    this->declare_parameter<double>("max_speed", 20.0);  // Maximum speed in m/s(5 m/s = 18 km/h)
    this->declare_parameter<double>("max_accel", 2.0);  // Maximum acceleration in m/s^2(2 m/s^2 = 0.2 g)
    this->declare_parameter<double>("max_decel", 2.0);  // Maximum deceleration in m/s^2(2 m/s^2 = 0.2 g)
    max_speed_ = this->get_parameter("max_speed").as_double();
    max_accel_ = this->get_parameter("max_accel").as_double();
    max_decel_ = this->get_parameter("max_decel").as_double();

    // Publisher: desired speed profile
    speed_profile_pub_ = this->create_publisher<std_msgs::msg::Float32MultiArray>(
        "/desired_speed_profile", 10);

    // Subscription: planned path (Path message)
    path_sub_ = this->create_subscription<nav_msgs::msg::Path>(
        "/local_planned_path", 10,
        std::bind(&SpeedPlanner::pathCallback, this, std::placeholders::_1));

    // Subscription: current speed (std_msgs/Float32), using SensorDataQoS for sensor data
    current_speed_sub_ = this->create_subscription<std_msgs::msg::Float32>(
        "/current_speed", rclcpp::SensorDataQoS(),
        std::bind(&SpeedPlanner::speedCallback, this, std::placeholders::_1));
}

void SpeedPlanner::speedCallback(const std_msgs::msg::Float32::SharedPtr msg)
{
    // Very lightweight: store the latest speed value
    current_speed_ = msg->data;
    current_speed_received_ = true;
    // (Single-threaded execution assumed, so no mutex needed for this variable)
}

void SpeedPlanner::pathCallback(const nav_msgs::msg::Path::SharedPtr path)
{
    // If no path points, do nothing
    if (path->poses.empty()) {
        return;
    }

    size_t N = path->poses.size();
    std::vector<float> speed(N);

    // Set initial speed to current speed if available, otherwise 0.0
    float initial_speed = 0.0f;
    if (current_speed_received_) {
        initial_speed = current_speed_;
    } else {
        RCLCPP_WARN(this->get_logger(),
            "Current speed not received yet. Using 0.0 as initial speed.");
        initial_speed = 0.0f;
    }
    // Cap initial speed to the maximum allowed speed
    if (initial_speed > static_cast<float>(max_speed_)) {
        initial_speed = static_cast<float>(max_speed_);
    }
    speed[0] = initial_speed;

    // Initialize remaining points of the speed profile to the max allowed speed
    for (size_t i = 1; i < N; ++i) {
        speed[i] = static_cast<float>(max_speed_);
    }
    // Set target speed at the final point (e.g., stop at end of path)
    speed[N - 1] = 0.0f;

    // Backward pass: enforce deceleration limits
    // Ensure the vehicle can decelerate from each point to the final speed within the distance to the next point
    for (int j = static_cast<int>(N) - 2; j >= 0; --j) {
        // Calculate distance between point j and j+1
        double dx = path->poses[j+1].pose.position.x - path->poses[j].pose.position.x;
        double dy = path->poses[j+1].pose.position.y - path->poses[j].pose.position.y;
        double dist = std::sqrt(dx * dx + dy * dy);
        // Compute allowed speed at point j to decelerate to speed[j+1] over distance
        double v_next = speed[j+1];
        double v_allow = std::sqrt(v_next * v_next + 2 * max_decel_ * dist);
        if (speed[j] > v_allow) {
            speed[j] = static_cast<float>(v_allow);
        }
    }

    // Forward pass: enforce acceleration limits
    // Ensure the vehicle does not accelerate faster than max_accel_ between consecutive points
    for (size_t j = 0; j < N - 1; ++j) {
        // Calculate distance between point j and j+1
        double dx = path->poses[j+1].pose.position.x - path->poses[j].pose.position.x;
        double dy = path->poses[j+1].pose.position.y - path->poses[j].pose.position.y;
        double dist = std::sqrt(dx * dx + dy * dy);
        // Compute allowed speed at point j+1 given acceleration from speed[j] over distance
        double v_curr = speed[j];
        double v_allow = std::sqrt(v_curr * v_curr + 2 * max_accel_ * dist);
        if (speed[j+1] > v_allow) {
            speed[j+1] = static_cast<float>(v_allow);
        }
    }

    // Publish the desired speed profile as a Float32MultiArray
    std_msgs::msg::Float32MultiArray speed_profile_msg;
    speed_profile_msg.data.resize(N);
    for (size_t i = 0; i < N; ++i) {
        speed_profile_msg.data[i] = speed[i];
    }
    speed_profile_msg.layout.data_offset = 0;  // (Optional: offset of the data array)
    // Note: layout.dim is left empty since this is a 1D array of length N

    speed_profile_pub_->publish(speed_profile_msg);
}

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<SpeedPlanner>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
