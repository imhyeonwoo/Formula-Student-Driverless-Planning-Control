#include <chrono>
#include <cmath>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <std_msgs/msg/float32.hpp>

using std::placeholders::_1;

namespace {
constexpr double kEarthRadiusM = 6371000.0; // 지구 반경 [m]

// deg → rad
inline double deg2rad(double deg) { return deg * M_PI / 180.0; }

// 하버사인 거리 [m]
double haversine(double lat1_deg, double lon1_deg, double lat2_deg, double lon2_deg) {
  const double lat1 = deg2rad(lat1_deg);
  const double lon1 = deg2rad(lon1_deg);
  const double lat2 = deg2rad(lat2_deg);
  const double lon2 = deg2rad(lon2_deg);

  const double dlat = lat2 - lat1;
  const double dlon = lon2 - lon1;

  const double a = std::sin(dlat/2.0) * std::sin(dlat/2.0) +
                   std::cos(lat1) * std::cos(lat2) *
                   std::sin(dlon/2.0) * std::sin(dlon/2.0);
  const double c = 2.0 * std::atan2(std::sqrt(a), std::sqrt(1.0 - a));
  return kEarthRadiusM * c;
}
} // namespace

class GpsSpeedNode final : public rclcpp::Node {
public:
  GpsSpeedNode() : Node("gps_speed_node") {
    // 파라미터 (필요 시 런치/CLI로 조정)
    min_dt_sec_      = this->declare_parameter<double>("min_dt_sec", 0.20);  // 너무 촘촘한 샘플 무시
    max_dt_sec_      = this->declare_parameter<double>("max_dt_sec", 5.0);   // 너무 뜬 샘플은 prev 리셋
    max_speed_mps_   = this->declare_parameter<double>("max_speed_mps", 100.0); // 비현실적 속도 컷
    ema_alpha_       = this->declare_parameter<double>("ema_alpha", 0.3);    // 0(무한평균)~1(필터X)
    require_good_fix_= this->declare_parameter<bool>("require_good_fix", false); // status 체크 여부

    pub_ = this->create_publisher<std_msgs::msg::Float32>("/current_speed", 10);

    sub_ = this->create_subscription<sensor_msgs::msg::NavSatFix>(
      "/ublox_gps_node/fix", rclcpp::SensorDataQoS(),
      std::bind(&GpsSpeedNode::onFix, this, _1));

    RCLCPP_INFO(get_logger(),
      "gps_speed_node started. Publishing /current_speed (m/s)."
      " params: min_dt=%.2f, max_dt=%.2f, ema_alpha=%.2f",
      min_dt_sec_, max_dt_sec_, ema_alpha_);
  }

private:
  void onFix(const sensor_msgs::msg::NavSatFix::SharedPtr msg) {
    // (옵션) Fix 상태가 너무 나쁘면 패스
    // NavSatStatus::STATUS_NO_FIX(-1)~RTK_FIXED(2) 등. 임시 용도면 꺼도 됨.
    if (require_good_fix_ && msg->status.status < 0) {
      return;
    }

    const rclcpp::Time t = msg->header.stamp;
    if (t.nanoseconds() == 0) {
      // 타임스탬프가 0이면 노드 시간 사용(임시)
      last_msg_time_ = this->now();
    }

    if (!has_prev_) {
      prev_lat_ = msg->latitude;
      prev_lon_ = msg->longitude;
      prev_time_ = t;
      has_prev_ = true;
      return;
    }

    const double dt = (t - prev_time_).seconds();
    if (dt <= 0.0 || dt < min_dt_sec_) {
      // 시간 흐름이 없거나 너무 촘촘하면 보류
      return;
    }
    if (dt > max_dt_sec_) {
      // 너무 오래 끊겼으면 기준점 리셋
      prev_lat_ = msg->latitude;
      prev_lon_ = msg->longitude;
      prev_time_ = t;
      return;
    }

    const double dist_m = haversine(prev_lat_, prev_lon_, msg->latitude, msg->longitude);
    double speed_mps = dist_m / dt;

    // 비현실적 점프 컷
    if (speed_mps > max_speed_mps_) {
      // 점프라고 보고 위치만 갱신하고 퍼블리시는 생략
      prev_lat_ = msg->latitude;
      prev_lon_ = msg->longitude;
      prev_time_ = t;
      return;
    }

    // EMA로 간단 스무딩 (alpha=1.0이면 필터 없이 즉시)
    if (has_ema_) {
      speed_mps = ema_alpha_ * speed_mps + (1.0 - ema_alpha_) * ema_speed_mps_;
    }
    ema_speed_mps_ = speed_mps;
    has_ema_ = true;

    std_msgs::msg::Float32 out;
    out.data = static_cast<float>(speed_mps);
    pub_->publish(out);

    // 상태 업데이트
    prev_lat_ = msg->latitude;
    prev_lon_ = msg->longitude;
    prev_time_ = t;
  }

  // Pub/Sub
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr pub_;
  rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr sub_;

  // 상태
  bool has_prev_{false};
  double prev_lat_{0.0}, prev_lon_{0.0};
  rclcpp::Time prev_time_{0, 0, RCL_ROS_TIME};

  // EMA
  bool has_ema_{false};
  double ema_speed_mps_{0.0};

  // 파라미터
  double min_dt_sec_{0.2};
  double max_dt_sec_{5.0};
  double max_speed_mps_{100.0};
  double ema_alpha_{0.3};
  bool require_good_fix_{false};

  rclcpp::Time last_msg_time_{0,0,RCL_ROS_TIME};
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<GpsSpeedNode>());
  rclcpp::shutdown();
  return 0;
}
