// src/Planning/cone_labeling/src/cone_classifier.cpp
// ──────────────────────────────────────────────────────────────
#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <geometry_msgs/msg/point.hpp>

#include "custom_interface/msg/modified_float32_multi_array.hpp"
#include "custom_interface/msg/tracked_cone_array.hpp"

#include <unordered_map>
#include <vector>
#include <queue>
#include <cmath>
#include <algorithm>
#include <atomic>
#include <cctype>

using visualization_msgs::msg::Marker;
using visualization_msgs::msg::MarkerArray;
using geometry_msgs::msg::Point;
using custom_interface::msg::ModifiedFloat32MultiArray;
using custom_interface::msg::TrackedConeArray;

static double rad2deg(double r){ return r * 180.0 / M_PI; }
static double deg2rad(double d){ return d * M_PI / 180.0; }

class ConeClassifier : public rclcpp::Node {
public:
  ConeClassifier() : Node("cone_classifier")
  {
    declare_parameter<double>("eps", 3.0);
    eps_ = get_parameter("eps").as_double();

    /* 구독 */
    sub_sorted_ = create_subscription<TrackedConeArray>(
      "/sorted_cones_time_ukf", rclcpp::SystemDefaultsQoS(),
      std::bind(&ConeClassifier::sortedCallback, this, std::placeholders::_1));

    sub_fused_ = create_subscription<ModifiedFloat32MultiArray>(
      "/fused_sorted_cones",     rclcpp::SystemDefaultsQoS(),
      std::bind(&ConeClassifier::fusedCallback,  this, std::placeholders::_1));

    /* 퍼블리셔  (left·right → Marker, 전체 → MarkerArray) */
    pub_clustered_     = create_publisher<MarkerArray>("/clustered_cones",   10);
    pub_left_cluster_  = create_publisher<Marker>     ("/left_cone_marker",  10);
    pub_right_cluster_ = create_publisher<Marker>     ("/right_cone_marker", 10);

    RCLCPP_INFO(get_logger(),
      "ConeClassifier started (eps=%.2f m). Skip when ≥ %d red cones within 10 m front.",
      eps_, kSkipRedThreshold);
  }

private:
  /* 상수 */
  static constexpr int  kSkipRedThreshold = 6;
  static constexpr char kFrameId[]        = "os_sensor";

  /* 빨간 콘 개수 집계 콜백 */
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
      if (x > 0.0 && std::hypot(x,y) <= 10.0) red_cnt++;
    }
    red_cone_count_.store(red_cnt, std::memory_order_relaxed);
  }

  /* 본체 콜백 */
  void sortedCallback(const TrackedConeArray::SharedPtr msg)
  {
    if (red_cone_count_.load() >= kSkipRedThreshold) return;

    /* 전체 DELETEALL */
    Marker clear_all; clear_all.header = msg->header; clear_all.action = Marker::DELETEALL;
    MarkerArray del_arr; del_arr.markers.push_back(clear_all);
    pub_clustered_->publish(del_arr);

    /* 좌우 리스트 초기화(DELETEALL) */
    Marker clear_lr = clear_all;
    pub_left_cluster_->publish(clear_lr);
    pub_right_cluster_->publish(clear_lr);

    /* 좌표 수집 */
    positions_.clear();
    for (size_t i=0;i<msg->cones.size();++i){
      const auto &c = msg->cones[i];
      positions_[static_cast<int>(i)] = {c.position.x, c.position.y};
    }
    if (positions_.empty()) return;

    /* 시드(가장 앞) 탐색 */
    left_seed_ = right_seed_ = -1;
    double bestL=1e9,bestR=1e9;
    for(auto &kv:positions_){
      double x=kv.second.first , y=kv.second.second;
      double ang=rad2deg(std::atan2(y,x)); if(ang<0)ang+=360.0;
      double d = std::hypot(x,y);
      if(ang>=0   && ang<90  && d<bestL){bestL=d; left_seed_=kv.first;}
      if(ang>=270 && ang<360 && d<bestR){bestR=d; right_seed_=kv.first;}
    }

    /* BFS 확장 */
    auto grow=[&](int seed){
      std::vector<int> cl; if(seed<0) return cl;
      std::queue<int> q; std::unordered_map<int,bool> vis; vis[seed]=true; q.push(seed); cl.push_back(seed);
      while(!q.empty()){
        int id=q.front(); q.pop(); auto p=positions_[id];
        for(auto &kv:positions_){
          int nid=kv.first; if(vis[nid]) continue;
          if(std::hypot(p.first-kv.second.first,p.second-kv.second.second)<=eps_){
            vis[nid]=true; q.push(nid); cl.push_back(nid);
          }
        }
      }
      return cl;
    };
    auto L = grow(left_seed_), R = grow(right_seed_);

    /* 마커 유틸 (단일 SPHERE) */
    auto make_marker=[&](int id,float r,float g,float b){
      Marker m;
      m.header.stamp    = now();
      m.header.frame_id = kFrameId;
      m.ns  = "cones";
      m.id  = id;
      m.type= Marker::SPHERE;
      m.action=Marker::ADD;
      m.pose.position.x = positions_[id].first;
      m.pose.position.y = positions_[id].second;
      m.pose.position.z = 0.0;
      m.scale.x = m.scale.y = m.scale.z = 0.3f;
      m.color.r = r; m.color.g = g; m.color.b = b; m.color.a = 1.0f;
      return m;
    };

    /* 전체 시각화용 MarkerArray */
    MarkerArray all_arr;

    /* 좌·우 SPHERE_LIST (파이썬 노드용) */
    Marker left_list, right_list;
    auto init_list=[&](Marker &m, const char* ns,float r,float g,float b){
      m.header.stamp    = now();
      m.header.frame_id = kFrameId;
      m.ns = ns;  m.id = 0;
      m.type = Marker::SPHERE_LIST;
      m.action = Marker::ADD;
      m.scale.x = m.scale.y = m.scale.z = 0.3f;
      m.color.r=r; m.color.g=g; m.color.b=b; m.color.a=1.0f;
    };
    init_list(left_list,  "left_list", 0.0f,0.0f,1.0f);   // BLUE
    init_list(right_list, "right_list",1.0f,1.0f,0.0f);   // YELLOW

    auto push_point=[&](Marker &m,int id){
      Point p; p.x=positions_[id].first; p.y=positions_[id].second;
      m.points.push_back(p);
    };

    for(auto &kv:positions_){
      int id = kv.first;
      if(id == left_seed_){
        auto m = make_marker(id,0,1,0);   // GREEN
        all_arr.markers.push_back(m);
        push_point(left_list,id);
      }else if(id == right_seed_){
        auto m = make_marker(id,1,0,0);   // RED
        all_arr.markers.push_back(m);
        push_point(right_list,id);
      }else if(std::count(L.begin(),L.end(),id)){
        auto m = make_marker(id,0,0,1);   // BLUE
        all_arr.markers.push_back(m);
        push_point(left_list,id);
      }else if(std::count(R.begin(),R.end(),id)){
        auto m = make_marker(id,1,1,0);   // YELLOW
        all_arr.markers.push_back(m);
        push_point(right_list,id);
      }else{
        all_arr.markers.push_back(make_marker(id,0.5f,0.5f,0.5f)); // GRAY
      }
    }

    /* 퍼블리시 */
    pub_clustered_->publish(all_arr);
    pub_left_cluster_->publish(left_list);
    pub_right_cluster_->publish(right_list);
  }

  /* 멤버 -----------------------------------------------------*/
  rclcpp::Subscription<TrackedConeArray>::SharedPtr          sub_sorted_;
  rclcpp::Subscription<ModifiedFloat32MultiArray>::SharedPtr sub_fused_;
  rclcpp::Publisher<MarkerArray>::SharedPtr pub_clustered_;
  rclcpp::Publisher<Marker>::SharedPtr      pub_left_cluster_, pub_right_cluster_;
  std::unordered_map<int,std::pair<double,double>> positions_;
  int    left_seed_{-1}, right_seed_{-1};
  double eps_;
  std::atomic<int> red_cone_count_{0};
};

int main(int argc,char**argv){
  rclcpp::init(argc,argv);
  rclcpp::spin(std::make_shared<ConeClassifier>());
  rclcpp::shutdown();
  return 0;
}
