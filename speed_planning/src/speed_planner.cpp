#include <cmath>
#include <limits>
#include <vector>
#include <algorithm>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

/**
 * @brief Speed Planner Node (curvature-constrained version)
 *
 * ① 곡률 κ 로부터 횡가속 한계 a_lat_max 를 적용해 각 지점의
 *    최대 허용 속도 v_curve = sqrt(a_lat_max / |κ|)
 * ② 최대 직선 속도 max_speed 와 min 처리
 * ③ 현재 속도로 첫 지점을 다시 min 처리
 * ④ Backward (감속 한계), Forward (가속 한계) 패스 적용
 * ※ 더 이상 speed[N-1] = 0 강제하지 않는다.
 */
class SpeedPlanner : public rclcpp::Node
{
public:
    SpeedPlanner();

private:
    // 콜백
    void pathCallback(const nav_msgs::msg::Path::SharedPtr path);
    void speedCallback(const std_msgs::msg::Float32::SharedPtr msg);

    // 유틸
    static double curvature2D(const geometry_msgs::msg::Pose &p0,
                              const geometry_msgs::msg::Pose &p1,
                              const geometry_msgs::msg::Pose &p2);

    // ROS 통신
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr speed_profile_pub_;
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr           path_sub_;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr        current_speed_sub_;

    // 상태
    float current_speed_;
    bool  current_speed_received_;

    // 파라미터
    double max_speed_;
    double max_accel_;
    double max_decel_;
    double max_lat_accel_;   ///< 최대 허용 횡가속 (m/s²)
};

// ─────────────────────────────────────────────────────────────
SpeedPlanner::SpeedPlanner()
: Node("speed_planner")
, current_speed_(0.0f)
, current_speed_received_(false)
{
    // 매개변수 선언
    this->declare_parameter<double>("max_speed",      20.0); // m/s
    this->declare_parameter<double>("max_accel",       2.0); // m/s²
    this->declare_parameter<double>("max_decel",       2.0); // m/s²
    this->declare_parameter<double>("max_lat_accel",   4.0); // m/s² (횡가속 한계)

    max_speed_      = this->get_parameter("max_speed").as_double();
    max_accel_      = this->get_parameter("max_accel").as_double();
    max_decel_      = this->get_parameter("max_decel").as_double();
    max_lat_accel_  = this->get_parameter("max_lat_accel").as_double();

    // 퍼블리셔
    speed_profile_pub_ = this->create_publisher<std_msgs::msg::Float32MultiArray>(
        "/desired_speed_profile", 10);

    // 서브스크립션
    path_sub_ = this->create_subscription<nav_msgs::msg::Path>(
        "/local_planned_path", 10,
        std::bind(&SpeedPlanner::pathCallback, this, std::placeholders::_1));

    current_speed_sub_ = this->create_subscription<std_msgs::msg::Float32>(
        "/current_speed", rclcpp::SensorDataQoS(),
        std::bind(&SpeedPlanner::speedCallback, this, std::placeholders::_1));
}

// ─────────────────────────────────────────────────────────────
void SpeedPlanner::speedCallback(const std_msgs::msg::Float32::SharedPtr msg)
{
    current_speed_           = msg->data;
    current_speed_received_  = true;
}

// ─────────────────────────────────────────────────────────────
double SpeedPlanner::curvature2D(const geometry_msgs::msg::Pose &p0,
                                 const geometry_msgs::msg::Pose &p1,
                                 const geometry_msgs::msg::Pose &p2)
{
    // 2D 상에서 세 점의 곡률 계산 (원의 반지름 역수)
    const double x0 = p0.position.x, y0 = p0.position.y;
    const double x1 = p1.position.x, y1 = p1.position.y;
    const double x2 = p2.position.x, y2 = p2.position.y;

    const double a   = std::hypot(x1 - x0, y1 - y0);
    const double b   = std::hypot(x2 - x1, y2 - y1);
    const double c   = std::hypot(x2 - x0, y2 - y0);
    const double det = (x1 - x0)*(y2 - y0) - (x2 - x0)*(y1 - y0); // 2*삼각형 면적 부호

    const double area2 = std::abs(det);          // 2*면적 (절대값)
    if (area2 < 1e-9 || a*b*c < 1e-9) {
        return 0.0;                              // 직선 또는 너무 가까운 점
    }
    return 2.0 * area2 / (a * b * c);            // κ = 4A / (abc)인데 det=2A, 2*area / (abc)
}

// ─────────────────────────────────────────────────────────────
void SpeedPlanner::pathCallback(const nav_msgs::msg::Path::SharedPtr path)
{
    const size_t N = path->poses.size();
    if (N < 2) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                             "Path too short (%zu pts) – skipping speed plan", N);
        return;
    }

    // ── 1. 곡률-기반 초기 속도 한계 ─────────────────────────
    std::vector<float> speed(N, static_cast<float>(max_speed_)); // 우선 max_speed로 채움

    constexpr double eps = 1e-6;
    for (size_t i = 1; i + 1 < N; ++i) {       // 끝점 제외(곡률=0 취급)
        double kappa = curvature2D(path->poses[i-1].pose,
                                   path->poses[i  ].pose,
                                   path->poses[i+1].pose);
        double v_curve = std::sqrt(max_lat_accel_ / (std::abs(kappa) + eps));
        speed[i] = static_cast<float>(std::min(max_speed_, v_curve));
    }
    // 양 끝점(0, N-1)은 인접 점과 동일한 곡률 한계를 적용하거나 max_speed 유지
    speed[0]     = speed[1];
    speed[N - 1] = speed[N - 2];

    // ── 2. 출발점: 현 차량 속도로 상한 설정 ────────────────
    float initial_speed = current_speed_received_ ? current_speed_ : 0.0f;
    speed[0] = std::min(speed[0], initial_speed);

    // ── 3. Backward pass – 감속 한계 ───────────────────────
    for (int j = static_cast<int>(N) - 2; j >= 0; --j) {
        const auto &p0 = path->poses[j    ].pose.position;
        const auto &p1 = path->poses[j + 1].pose.position;
        double dist = std::hypot(p1.x - p0.x, p1.y - p0.y);
        double v_next = speed[j + 1];
        double v_allow = std::sqrt(v_next * v_next + 2.0 * max_decel_ * dist);
        if (speed[j] > v_allow) speed[j] = static_cast<float>(v_allow);
    }

    // ── 4. Forward pass – 가속 한계 ────────────────────────
    for (size_t j = 0; j + 1 < N; ++j) {
        const auto &p0 = path->poses[j    ].pose.position;
        const auto &p1 = path->poses[j + 1].pose.position;
        double dist = std::hypot(p1.x - p0.x, p1.y - p0.y);
        double v_curr  = speed[j];
        double v_allow = std::sqrt(v_curr * v_curr + 2.0 * max_accel_ * dist);
        if (speed[j + 1] > v_allow) speed[j + 1] = static_cast<float>(v_allow);
    }

    // ── 5. 퍼블리시 ─────────────────────────────────────────
    std_msgs::msg::Float32MultiArray msg;
    msg.data.assign(speed.begin(), speed.end());
    speed_profile_pub_->publish(msg);
}

// ─────────────────────────────────────────────────────────────
int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SpeedPlanner>());
    rclcpp::shutdown();
    return 0;
}
