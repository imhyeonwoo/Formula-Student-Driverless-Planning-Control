// src/cone_classifier.cpp
// ──────────────────────────────────────────────────────────────
// • 입력 1) /sorted_cones_time_ukf   (custom_interface/msg/TrackedConeArray)
// • 입력 2) /fused_sorted_cones      (custom_interface/msg/ModifiedFloat32MultiArray)
//          └ 전방 10 m 빨간 콘 개수 집계 (6개 이상이면 stop_node=1)
// • 출력   : /clustered_cones        (전체 색상)
//            /left_cone_marker      (왼쪽 클러스터 전체)
//            /right_cone_marker     (오른쪽 클러스터 전체)
//            /stop_node             (0 또는 1, 100 ms 간격 퍼블리시)
// • 프레임 : os_sensor
// • 변경   : BFS → “마지막 추가 콘” 기준의 각도/거리/전방성 제약 연쇄 확장(grow_chain)
// ──────────────────────────────────────────────────────────────

#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <std_msgs/msg/int32.hpp>

#include "custom_interface/msg/modified_float32_multi_array.hpp"
#include "custom_interface/msg/tracked_cone_array.hpp"

#include <unordered_map>
#include <unordered_set>
#include <vector>
#include <queue>
#include <cmath>
#include <algorithm>
#include <atomic>
#include <cctype>
#include <limits>
#include <string>

using namespace std::chrono_literals;
using visualization_msgs::msg::Marker;
using visualization_msgs::msg::MarkerArray;
using geometry_msgs::msg::Point;
using std_msgs::msg::Int32;
using custom_interface::msg::ModifiedFloat32MultiArray;
using custom_interface::msg::TrackedConeArray;

static double rad2deg(double r){ return r * 180.0 / M_PI; }
static double deg2rad(double d){ return d * M_PI / 180.0; }
static double wrapToPi(double a){
  while (a >  M_PI) a -= 2*M_PI;
  while (a < -M_PI) a += 2*M_PI;
  return a;
}

class ConeClassifier : public rclcpp::Node {
public:
  ConeClassifier() : Node("cone_classifier")
  {
    // ===== 파라미터 =====
    declare_parameter<double>("eps", 4.0);                 // (미사용: 이전 BFS 잔존 호환)
    declare_parameter<double>("min_step_m", 0.0);          // 연쇄에서 허용되는 최소 이동 거리
    declare_parameter<double>("max_step_m", 4.0);          // 연쇄에서 허용되는 최대 이동 거리
    declare_parameter<double>("target_step_m", 0.5);       // 점수화에 쓰는 선호 이동 거리
    declare_parameter<double>("theta_max_deg", 70.0);      // 진행 헤딩 대비 허용 각도 편차(±)
    declare_parameter<double>("heading_blend", 1.0);       // 헤딩 지수평활 가중치(0~1)
    declare_parameter<bool>("forward_x_only", false);       // x가 후퇴하는 후보 금지
    declare_parameter<double>("forward_tol_m", 1.0);       // 후퇴 허용 오차(완전 고정 방지)
    declare_parameter<double>("y_side_tol_m", 1.0);       // 좌/우 일관성(부호) 판정 여유
    declare_parameter<double>("dist_weight", 0.10);        // 각도편차 + dist_weight*거리편차 점수화

    eps_             = get_parameter("eps").as_double();
    min_step_m_      = get_parameter("min_step_m").as_double();
    max_step_m_      = get_parameter("max_step_m").as_double();
    target_step_m_   = get_parameter("target_step_m").as_double();
    theta_max_deg_   = get_parameter("theta_max_deg").as_double();
    heading_blend_   = get_parameter("heading_blend").as_double();
    forward_x_only_  = get_parameter("forward_x_only").as_bool();
    forward_tol_m_   = get_parameter("forward_tol_m").as_double();
    y_side_tol_m_    = get_parameter("y_side_tol_m").as_double();
    dist_weight_     = get_parameter("dist_weight").as_double();

    // ===== 구독자 =====
    sub_sorted_ = create_subscription<TrackedConeArray>(
      "/sorted_cones_time_ukf", rclcpp::SensorDataQoS(),
      std::bind(&ConeClassifier::sortedCallback, this, std::placeholders::_1));

    sub_fused_ = create_subscription<ModifiedFloat32MultiArray>(
      "/fused_sorted_cones", rclcpp::SystemDefaultsQoS(),
      std::bind(&ConeClassifier::fusedCallback, this, std::placeholders::_1));

    // ===== 퍼블리셔 =====
    pub_clustered_     = create_publisher<MarkerArray>("/clustered_cones",   10);
    pub_left_cluster_  = create_publisher<MarkerArray>("/left_cone_marker",  10);
    pub_right_cluster_ = create_publisher<MarkerArray>("/right_cone_marker", 10);
    pub_stop_          = create_publisher<Int32>("/stop_node", 10);

    // 100ms 주기로 stop_node(0/1) 퍼블리시
    timer_ = this->create_wall_timer(
      100ms,
      [this]() {
        Int32 stop_msg;
        stop_msg.data = (red_cone_count_.load(std::memory_order_relaxed) >= kSkipRedThreshold) ? 1 : 0;
        pub_stop_->publish(stop_msg);
      }
    );

    RCLCPP_INFO(get_logger(),
      "ConeClassifier started. chain params: min=%.2f, max=%.2f, theta=%.1f deg, fwd=%s",
      min_step_m_, max_step_m_, theta_max_deg_, forward_x_only_ ? "true" : "false");
  }

private:
  // ===== 상수 =====
  static constexpr int  kSkipRedThreshold = 6;
  static constexpr char kFrameId[]        = "os_sensor";

  enum class Side { LEFT, RIGHT };

  // ===== /fused_sorted_cones 콜백: 전방 10m 'red' 카운트 =====
  void fusedCallback(const ModifiedFloat32MultiArray::SharedPtr msg)
  {
    int red_cnt = 0;
    const auto &cls = msg->class_names;
    const auto &d   = msg->data;
    size_t n = cls.size();
    if (d.size() < n * 3) { red_cone_count_.store(0, std::memory_order_relaxed); return; }

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

  // ===== /sorted_cones_time_ukf 콜백: 좌/우 체인 확장 + 마커 =====
  void sortedCallback(const TrackedConeArray::SharedPtr msg)
  {
    // 이전 마커 삭제(프레임 일관성 유지)
    Marker clear;
    clear.header.stamp    = msg->header.stamp;
    clear.header.frame_id = kFrameId;
    clear.action          = Marker::DELETEALL;
    MarkerArray del; del.markers.push_back(clear);
    pub_clustered_->publish(del);
    pub_left_cluster_->publish(del);
    pub_right_cluster_->publish(del);

    // 좌표 수집
    positions_.clear();
    positions_.reserve(msg->cones.size());
    for (size_t i = 0; i < msg->cones.size(); ++i) {
      const auto &c = msg->cones[i];
      positions_[static_cast<int>(i)] = {c.position.x, c.position.y};
    }
    if (positions_.empty()) return;

    // 시드 찾기(전과 동일: 좌측 [0°,90°), 우측 [270°,360°))
    left_seed_ = right_seed_ = -1;
    double bestL = std::numeric_limits<double>::infinity();
    double bestR = std::numeric_limits<double>::infinity();
    for (auto &kv : positions_) {
      double x = kv.second.first, y = kv.second.second;
      double ang = rad2deg(std::atan2(y, x));
      if (ang < 0) ang += 360.0;
      double d = std::hypot(x, y);
      if (ang >= 0   && ang < 90  && d < bestL) { bestL = d; left_seed_  = kv.first; }
      if (ang >= 270 && ang < 360 && d < bestR) { bestR = d; right_seed_ = kv.first; }
    }

    // 체인 확장 (BFS → 연쇄)
    auto L = grow_chain(left_seed_,  Side::LEFT);
    auto R = grow_chain(right_seed_, Side::RIGHT);

    // 마커 생성 헬퍼
    auto make_sphere = [&](int id, float r, float g, float b){
      Marker m;
      m.header.stamp    = this->get_clock()->now();
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
    auto make_strip = [&](const std::vector<int>& chain, int id_base, float r, float g, float b){
      Marker line;
      line.header.stamp    = this->get_clock()->now();
      line.header.frame_id = kFrameId;
      line.ns = "chains";
      line.id = id_base;
      line.type = Marker::LINE_STRIP;
      line.action = Marker::ADD;
      line.scale.x = 0.05; // 선 두께
      line.color.r = r; line.color.g = g; line.color.b = b; line.color.a = 0.9f;
      for (int idx : chain) {
        Point p;
        p.x = positions_[idx].first;
        p.y = positions_[idx].second;
        p.z = 0.0;
        line.points.push_back(p);
      }
      return line;
    };

    // membership set (O(1))
    std::unordered_set<int> setL(L.begin(), L.end()), setR(R.begin(), R.end());

    // 각 클러스터에 색상 부여(시드/체인 유지)
    MarkerArray all, outL, outR;

    // 체인 라인도 추가(디버그에 유용)
    if (!L.empty()) outL.markers.push_back(make_strip(L, 1000000, 0.0f, 0.6f, 1.0f)); // 청록 라인
    if (!R.empty()) outR.markers.push_back(make_strip(R, 2000000, 1.0f, 0.6f, 0.0f)); // 주황 라인

    for (auto &kv : positions_) {
      int id = kv.first;
      if      (id == left_seed_)  { auto m = make_sphere(id,0,1,0); all.markers.push_back(m); outL.markers.push_back(m); }
      else if (id == right_seed_) { auto m = make_sphere(id,1,0,0); all.markers.push_back(m); outR.markers.push_back(m); }
      else if (setL.count(id))    { auto m = make_sphere(id,0,0,1); all.markers.push_back(m); outL.markers.push_back(m); }
      else if (setR.count(id))    { auto m = make_sphere(id,1,1,0); all.markers.push_back(m); outR.markers.push_back(m); }
      else                        { auto m = make_sphere(id,0.5f,0.5f,0.5f); all.markers.push_back(m); }
    }

    // 퍼블리시
    pub_clustered_->publish(all);
    pub_left_cluster_->publish(outL);
    pub_right_cluster_->publish(outR);
  }

  // ===== 체인 확장: 마지막 점 기준, 각도/거리/전방성 제약 =====
  std::vector<int> grow_chain(int seed, Side side)
  {
    std::vector<int> chain;
    if (seed < 0 || positions_.empty()) return chain;

    std::unordered_set<int> used;
    used.insert(seed);
    chain.push_back(seed);

    // 초기 헤딩: 차량→시드 벡터
    {
      auto p = positions_[seed];
      heading_ = std::atan2(p.second, p.first);
    }

    for (;;){
      int last = chain.back();
      auto lp = positions_[last];

      int best_id = -1;
      double best_score = std::numeric_limits<double>::infinity();

      for (auto &kv : positions_) {
        int id = kv.first;
        if (used.count(id)) continue;
        auto p = kv.second;

        // 좌/우 y-부호 일관성
        if (side == Side::LEFT) {
          if (p.second < -y_side_tol_m_) continue;
        } else { // RIGHT
          if (p.second >  y_side_tol_m_) continue;
        }

        double dx = p.first - lp.first;
        double dy = p.second - lp.second;
        double dist = std::hypot(dx, dy);
        if (dist < min_step_m_ || dist > max_step_m_) continue;

        // 전방 진행(x가 크게 후퇴하지 않도록)
        if (forward_x_only_) {
          if ((p.first + forward_tol_m_) < lp.first) continue;
        }

        // 각도 제약: 이전 진행 헤딩과 후보 방향 각도 차
        double ang = std::atan2(dy, dx);
        double diff = std::fabs(wrapToPi(ang - heading_));
        double diff_deg = rad2deg(diff);
        if (diff_deg > theta_max_deg_) continue;

        // 점수: 각도 편차 + 거리 편차 가중
        double score = diff + dist_weight_ * std::fabs(dist - target_step_m_);
        if (score < best_score) {
          best_score = score;
          best_id = id;
        }
      }

      if (best_id == -1) break;

      // 채택
      chain.push_back(best_id);
      used.insert(best_id);

      // 헤딩 갱신(지수평활)
      {
        auto prev = lp;
        auto curr = positions_[best_id];
        double new_heading = std::atan2(curr.second - prev.second, curr.first - prev.first);
        heading_ = heading_blend_ * new_heading + (1.0 - heading_blend_) * heading_;
        heading_ = wrapToPi(heading_);
      }
    }
    return chain;
  }

  /* 멤버 변수 */
  rclcpp::Subscription<TrackedConeArray>::SharedPtr          sub_sorted_;
  rclcpp::Subscription<ModifiedFloat32MultiArray>::SharedPtr sub_fused_;
  rclcpp::Publisher<MarkerArray>::SharedPtr                  pub_clustered_, pub_left_cluster_, pub_right_cluster_;
  rclcpp::Publisher<Int32>::SharedPtr                        pub_stop_;
  rclcpp::TimerBase::SharedPtr                               timer_;

  std::unordered_map<int,std::pair<double,double>> positions_; // id -> (x,y)
  double eps_{4.0};
  int left_seed_{-1}, right_seed_{-1};
  std::atomic<int> red_cone_count_{0};

  // 체인 파라미터
  double min_step_m_{0.0};
  double max_step_m_{4.0};
  double target_step_m_{4.0};
  double theta_max_deg_{100.0};
  double heading_blend_{1.0};
  bool   forward_x_only_{false};
  double forward_tol_m_{1.0};
  double y_side_tol_m_{1.0};
  double dist_weight_{1.0};

  // 진행 헤딩(라디안)
  double heading_{0.0};
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ConeClassifier>());
  rclcpp::shutdown();
  return 0;
}

