/************************************************************
 * cone_marker_merger.cpp  (ROS 2 Humble)
 * /left|right_cone_marker/{real,virtual} -> /left|right_cone_marker 로 통합
 * - 반경 병합 r_merge 로 중복 제거
 ***********************************************************/
#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <geometry_msgs/msg/point.hpp>

#include <vector>
#include <cmath>
#include <algorithm>
#include <string>

using rclcpp::Node;
using visualization_msgs::msg::Marker;
using geometry_msgs::msg::Point;

class ConeMarkerMerger : public Node {
public:
  ConeMarkerMerger() : Node("cone_marker_merger")
  {
    r_merge_ = declare_parameter<double>("r_merge", 0.20); // m

    auto qos = rclcpp::QoS(1).reliable().transient_local();

    // 입력 구독
    sub_left_real_ = create_subscription<Marker>(
      "/left_cone_marker/real", 10,
      std::bind(&ConeMarkerMerger::cbLeftReal, this, std::placeholders::_1));
    sub_left_virtual_ = create_subscription<Marker>(
      "/left_cone_marker/virtual", 10,
      std::bind(&ConeMarkerMerger::cbLeftVirtual, this, std::placeholders::_1));
    sub_right_real_ = create_subscription<Marker>(
      "/right_cone_marker/real", 10,
      std::bind(&ConeMarkerMerger::cbRightReal, this, std::placeholders::_1));
    sub_right_virtual_ = create_subscription<Marker>(
      "/right_cone_marker/virtual", 10,
      std::bind(&ConeMarkerMerger::cbRightVirtual, this, std::placeholders::_1));

    // 출력 퍼블리셔 (플래너가 구독)
    pub_left_  = create_publisher<Marker>("/left_cone_marker",  qos);
    pub_right_ = create_publisher<Marker>("/right_cone_marker", qos);

    RCLCPP_INFO(get_logger(), "ConeMarkerMerger started (r_merge=%.2f m)", r_merge_);
  }

private:
  // 콜백: 최신 메시지 캐시 후 재계산/발행
  void cbLeftReal(const Marker::SharedPtr msg){ left_real_ = *msg;  recomputeLeft();  }
  void cbLeftVirtual(const Marker::SharedPtr msg){ left_virtual_ = *msg; recomputeLeft(); }
  void cbRightReal(const Marker::SharedPtr msg){ right_real_ = *msg; recomputeRight(); }
  void cbRightVirtual(const Marker::SharedPtr msg){ right_virtual_ = *msg; recomputeRight(); }

  // 좌/우 각각 병합 후 퍼블리시
  void recomputeLeft()  { publishMerged(true);  }
  void recomputeRight() { publishMerged(false); }

  static double dist2(const Point& a, const Point& b){
    double dx=a.x-b.x, dy=a.y-b.y, dz=a.z-b.z;
    return dx*dx+dy*dy+dz*dz;
  }

  static void appendIfFar(std::vector<Point>& out, const Point& p, double r2){
    for(const auto& q: out){
      if(dist2(p,q) <= r2) return; // 너무 가까우면 스킵(중복)
    }
    out.push_back(p);
  }

  void publishMerged(bool is_left){
    const Marker& real   = is_left ? left_real_   : right_real_;
    const Marker& virt   = is_left ? left_virtual_: right_virtual_;
    auto&         pub    = is_left ? pub_left_    : pub_right_;
    const int     id     = is_left ? 210 : 211;

    // SPHERE_LIST만 취급
    if(real.type != Marker::SPHERE_LIST && virt.type != Marker::SPHERE_LIST){
      // 아무것도 없으면 빈 마커라도 내보냄
      Marker empty;
      empty.header.stamp = now();
      empty.header.frame_id = "map";
      empty.ns="cone_side_merged"; empty.id=id;
      empty.type=Marker::SPHERE_LIST; empty.action=Marker::ADD;
      empty.scale.x=empty.scale.y=empty.scale.z=0.28f;
      empty.color.a=1.0f;
      if(is_left){ empty.color.r=0.0f; empty.color.g=0.3f; empty.color.b=1.0f; }
      else       { empty.color.r=1.0f; empty.color.g=1.0f; empty.color.b=0.0f; }
      pub->publish(empty);
      return;
    }

    // frame_id 통일: 우선순위 real, 없으면 virt, 기본 "map"
    std::string frame = "map";
    if(real.header.frame_id.size()) frame = real.header.frame_id;
    else if(virt.header.frame_id.size()) frame = virt.header.frame_id;

    // 병합
    std::vector<Point> merged;
    merged.reserve(real.points.size()+virt.points.size());
    const double r2 = r_merge_*r_merge_;

    // real 먼저 넣고
    for(const auto& p: real.points) appendIfFar(merged, p, r2);
    // virtual을 반경 병합으로 추가
    for(const auto& p: virt.points) appendIfFar(merged, p, r2);

    // 퍼블리시
    Marker mk;
    mk.header.stamp = now();
    mk.header.frame_id = frame;
    mk.ns   = "cone_side_merged";
    mk.id   = id;
    mk.type = Marker::SPHERE_LIST;
    mk.action = Marker::ADD;
    mk.scale.x = mk.scale.y = mk.scale.z = 0.28f; // 플래너 기본 스케일
    mk.color.a = 1.0f;
    if(is_left){ mk.color.r=0.0f; mk.color.g=0.3f; mk.color.b=1.0f; }
    else       { mk.color.r=1.0f; mk.color.g=1.0f; mk.color.b=0.0f; }

    mk.points.swap(merged);
    pub->publish(mk);
  }

  // 멤버
  double r_merge_;
  rclcpp::Subscription<Marker>::SharedPtr sub_left_real_, sub_left_virtual_;
  rclcpp::Subscription<Marker>::SharedPtr sub_right_real_, sub_right_virtual_;
  rclcpp::Publisher<Marker>::SharedPtr    pub_left_, pub_right_;
  Marker left_real_, left_virtual_, right_real_, right_virtual_;
};

int main(int argc, char** argv){
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ConeMarkerMerger>());
  rclcpp::shutdown();
  return 0;
}
