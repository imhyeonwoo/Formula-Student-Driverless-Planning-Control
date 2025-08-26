// src/cone_classifier.cpp
// ──────────────────────────────────────────────────────────────
// • 입력 1) /cone/lidar/ukf   (custom_interface/msg/TrackedConeArray)
// • 입력 2) /cone/fused      (custom_interface/msg/ModifiedFloat32MultiArray)
//          └ 전방 10 m 빨간 콘 개수 집계 (6개 이상이면 stop_node=1)
// • 출력   : /clustered_cones        (전체 색상)
//            /left_cone_marker      (왼쪽 클러스터 전체)
//            /right_cone_marker     (오른쪽 클러스터 전체)
//            /stop_node             (0 또는 1, 100 ms 간격 퍼블리시)
// • 프레임 : os_sensor
// ──────────────────────────────────────────────────────────────

#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <std_msgs/msg/int32.hpp>
#include <chrono>

#include "custom_interface/msg/modified_float32_multi_array.hpp"
#include "custom_interface/msg/tracked_cone_array.hpp"

#include <unordered_map>
#include <vector>
#include <queue>
#include <cmath>
#include <algorithm>
#include <atomic>
#include <cctype>

using namespace std::chrono_literals;
using visualization_msgs::msg::Marker;
using visualization_msgs::msg::MarkerArray;
using geometry_msgs::msg::Point;
using std_msgs::msg::Int32;
using custom_interface::msg::ModifiedFloat32MultiArray;
using custom_interface::msg::TrackedConeArray;

static double rad2deg(double r){ return r * 180.0 / M_PI; }

class ConeClassifier : public rclcpp::Node {
public:
  ConeClassifier() : Node("cone_classifier")
  {
    // 파라미터
    declare_parameter<double>("eps", 4.0);
    eps_ = get_parameter("eps").as_double();

    // 구독자
    sub_sorted_ = create_subscription<TrackedConeArray>(
      "/cone/lidar/ukf", rclcpp::SystemDefaultsQoS(),
      std::bind(&ConeClassifier::sortedCallback, this, std::placeholders::_1));

    sub_fused_ = create_subscription<ModifiedFloat32MultiArray>(
      "/cone/fused", rclcpp::SystemDefaultsQoS(),
      std::bind(&ConeClassifier::fusedCallback, this, std::placeholders::_1));

    // 퍼블리셔
    pub_clustered_    = create_publisher<MarkerArray>("/clustered_cones",   10);
    pub_left_cluster_ = create_publisher<MarkerArray>("/left_cone_marker",  10);
    pub_right_cluster_= create_publisher<MarkerArray>("/right_cone_marker", 10);
    pub_stop_         = create_publisher<Int32>("/stop_node", 10);

    // 100ms 주기로 stop_node(0/1) 퍼블리시
    timer_ = this->create_wall_timer(
      100ms,
      [this]() {
        Int32 stop_msg;
        stop_msg.data = (red_cone_count_.load() >= kSkipRedThreshold) ? 1 : 0;
        pub_stop_->publish(stop_msg);
      }
    );

    RCLCPP_INFO(get_logger(),
      "ConeClassifier started (eps=%.2f m). Publishing /stop_node at 100ms intervals.",
      eps_);
  }

private:
  // 임계치 상수
  static constexpr int  kSkipRedThreshold = 6;
  static constexpr char kFrameId[]        = "os_sensor";

  // fused_sorted_cones 콜백: 빨간 콘 개수 집계
  void fusedCallback(const ModifiedFloat32MultiArray::SharedPtr msg)
  {
    int red_cnt = 0;
    const auto &cls = msg->class_names;
    const auto &d   = msg->data;
    size_t n = cls.size();
    if (d.size() < n * 3) return;

    for (size_t i = 0; i < n; ++i) {
      std::string name = cls[i];
      std::transform(name.begin(), name.end(), name.begin(),
                     [](unsigned char c){ return std::tolower(c); });
      if (name.find("red") == std::string::npos) continue;

      double x = d[3*i], y = d[3*i+1];
      if (x > 0.0 && std::hypot(x, y) <= 10.0) {
        red_cnt++;
      }
    }
    red_cone_count_.store(red_cnt, std::memory_order_relaxed);
  }

  // cone/lidar/ukf 콜백: 매 프레임 클러스터링 & 마커 퍼블리시
  void sortedCallback(const TrackedConeArray::SharedPtr msg)
  {
    // 이전 마커 삭제
    Marker clear;
    clear.header = msg->header;
    clear.action = Marker::DELETEALL;
    MarkerArray del; del.markers.push_back(clear);
    pub_clustered_->publish(del);
    pub_left_cluster_->publish(del);
    pub_right_cluster_->publish(del);

    // 좌표 수집
    positions_.clear();
    for (size_t i = 0; i < msg->cones.size(); ++i) {
      const auto &c = msg->cones[i];
      positions_[static_cast<int>(i)] = {c.position.x, c.position.y};
    }
    if (positions_.empty()) return;

    // 가장 앞쪽 콘(시드) 찾기
    left_seed_ = right_seed_ = -1;
    double bestL = 1e9, bestR = 1e9;
    for (auto &kv : positions_) {
      double x = kv.second.first, y = kv.second.second;
      double ang = rad2deg(std::atan2(y, x));
      if (ang < 0) ang += 360.0;
      double d = std::hypot(x, y);
      if (ang >= 0 && ang < 90 && d < bestL)      { bestL = d; left_seed_  = kv.first; }
      if (ang >= 270 && ang < 360 && d < bestR)   { bestR = d; right_seed_ = kv.first; }
    }

    // BFS로 클러스터 확장
    auto grow = [&](int seed){
      std::vector<int> cl;
      if (seed < 0) return cl;
      std::queue<int> q;
      std::unordered_map<int,bool> vis;
      vis[seed] = true;
      q.push(seed);
      cl.push_back(seed);
      while (!q.empty()) {
        int id = q.front(); q.pop();
        auto p = positions_[id];
        for (auto &kv : positions_) {
          int nid = kv.first;
          if (vis[nid]) continue;
          double dx = p.first - kv.second.first;
          double dy = p.second - kv.second.second;
          if (std::hypot(dx, dy) <= eps_) {
            vis[nid] = true;
            q.push(nid);
            cl.push_back(nid);
          }
        }
      }
      return cl;
    };
    auto L = grow(left_seed_), R = grow(right_seed_);

    // 마커 생성 헬퍼
    auto make_marker = [&](int id, float r, float g, float b){
      Marker m;
      m.header.stamp    = now();
      m.header.frame_id = kFrameId;
      m.ns   = "cones";
      m.id   = id;
      m.type = Marker::SPHERE;
      m.action = Marker::ADD;
      m.pose.position.x = positions_[id].first;
      m.pose.position.y = positions_[id].second;
      m.pose.position.z = 0.0;
      m.scale.x = m.scale.y = m.scale.z = 0.3f;
      m.color.r = r; m.color.g = g; m.color.b = b; m.color.a = 1.0f;
      return m;
    };

    // 각 클러스터에 색상 부여
    MarkerArray all, outL, outR;
    for (auto &kv : positions_) {
      int id = kv.first;
      if      (id == left_seed_)  { auto m = make_marker(id,0,1,0); all.markers.push_back(m); outL.markers.push_back(m); }
      else if (id == right_seed_) { auto m = make_marker(id,1,0,0); all.markers.push_back(m); outR.markers.push_back(m); }
      else if (std::count(L.begin(), L.end(), id)) {
                                  auto m = make_marker(id,0,0,1); all.markers.push_back(m); outL.markers.push_back(m); }
      else if (std::count(R.begin(), R.end(), id)) {
                                  auto m = make_marker(id,1,1,0); all.markers.push_back(m); outR.markers.push_back(m); }
      else                          all.markers.push_back(make_marker(id,0.5f,0.5f,0.5f));
    }

    // 퍼블리시
    pub_clustered_->publish(all);
    pub_left_cluster_->publish(outL);
    pub_right_cluster_->publish(outR);
  }

  /* 멤버 변수 */
  rclcpp::Subscription<TrackedConeArray>::SharedPtr          sub_sorted_;
  rclcpp::Subscription<ModifiedFloat32MultiArray>::SharedPtr sub_fused_;
  rclcpp::Publisher<MarkerArray>::SharedPtr                 pub_clustered_, pub_left_cluster_, pub_right_cluster_;
  rclcpp::Publisher<Int32>::SharedPtr                        pub_stop_;
  rclcpp::TimerBase::SharedPtr                              timer_;

  std::unordered_map<int,std::pair<double,double>> positions_;
  double eps_;
  int left_seed_{-1}, right_seed_{-1};
  std::atomic<int> red_cone_count_{0};
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ConeClassifier>());
  rclcpp::shutdown();
  return 0;
}
