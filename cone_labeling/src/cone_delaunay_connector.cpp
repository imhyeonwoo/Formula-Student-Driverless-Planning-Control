// src/delaunay_connector.cpp
// ──────────────────────────────────────────────────────────────
// • 입력 : /clustered_cones  (MarkerArray, 색상으로 좌/우 판단)
// • 출력 : /cone_delaunay_edges /cone_delaunay_midpoints
// • 조건 : Delaunay 삼각분할 엣지 중
//          └ ① 한 점은 왼쪽(녹·파), 다른 점은 오른쪽(빨·노)
//          └ ② 두 점 사이 거리 ≤ 3 m
// ──────────────────────────────────────────────────────────────

#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <unordered_map>
#include <vector>
#include <set>
#include <cmath>

using visualization_msgs::msg::Marker;
using visualization_msgs::msg::MarkerArray;
using geometry_msgs::msg::Point;

class DelaunayConnector : public rclcpp::Node {
public:
  DelaunayConnector()
  : Node("delaunay_connector")
  {
    sub_ = create_subscription<MarkerArray>(
      "/clustered_cones", rclcpp::SystemDefaultsQoS(),
      std::bind(&DelaunayConnector::callback, this, std::placeholders::_1)
    );
    pub_edges_ = create_publisher<MarkerArray>("/cone_delaunay_edges", 10);
    pub_mid_   = create_publisher<MarkerArray>("/cone_delaunay_midpoints", 10);
    RCLCPP_INFO(get_logger(), "DelaunayConnector started, listening on /clustered_cones");
  }

private:
  void callback(const MarkerArray::SharedPtr msg)
  {
    if (msg->markers.empty()) {
      return;
    }

    // 0) 이전 마커 전부 삭제
    Marker clear;
    clear.header = msg->markers.front().header;
    clear.action = Marker::DELETEALL;
    MarkerArray clear_edges; clear_edges.markers.push_back(clear);
    pub_edges_->publish(clear_edges);
    MarkerArray clear_mid;   clear_mid.markers.push_back(clear);
    pub_mid_->publish(clear_mid);

    // 1) positions 및 클러스터 ID 수집 (회색 콘은 건너뜀)
    positions_.clear();
    std::set<int> left_set, right_set;
    for (const auto &m : msg->markers) {
      if (m.ns != "cones") {
        continue;
      }
      auto c = m.color;
      bool is_left  = (c.g > 0.9f && c.r < 0.1f && c.b < 0.1f)  // 녹색
                    || (c.b > 0.9f && c.r < 0.1f && c.g < 0.1f); // 파랑
      bool is_right = (c.r > 0.9f && c.g < 0.1f && c.b < 0.1f)  // 빨강
                    || (c.r > 0.9f && c.g > 0.9f && c.b < 0.1f); // 노랑
      if (!(is_left || is_right)) {
        continue;
      }
      int id = m.id;
      positions_[id] = { m.pose.position.x, m.pose.position.y };
      if (is_left) {
        left_set.insert(id);
      } else {
        right_set.insert(id);
      }
    }

    if (left_set.empty() || right_set.empty()) {
      RCLCPP_WARN(get_logger(), "Left or right cluster empty, skipping Delaunay.");
      return;
    }

    // 2) Subdiv2D용 포인트 및 경계 계산
    std::vector<cv::Point2f> pts;
    std::vector<int> ids;
    double minx=1e6, miny=1e6, maxx=-1e6, maxy=-1e6;
    for (auto &kv : positions_) {
      int id = kv.first;
      double x = kv.second.first;
      double y = kv.second.second;
      pts.emplace_back(static_cast<float>(x), static_cast<float>(y));
      ids.push_back(id);
      minx = std::min(minx, x);
      miny = std::min(miny, y);
      maxx = std::max(maxx, x);
      maxy = std::max(maxy, y);
    }
    float margin = 1.0f;
    cv::Rect2f rect(minx - margin, miny - margin,
                    (maxx - minx) + 2 * margin, (maxy - miny) + 2 * margin);
    cv::Subdiv2D subdiv(rect);
    subdiv.insert(pts);

    // 3) Delaunay 삼각 리스트
    std::vector<cv::Vec6f> tris;
    subdiv.getTriangleList(tris);

    // 4) 엣지 & 중점 마커 생성 (거리 ≤ 8 m)
    constexpr float kMaxEdge = 8.0f;  // 길이 조건 (m)

    MarkerArray edges_arr;
    Marker line;
    line.header = msg->markers.front().header;
    line.ns     = "delaunay_edges";
    line.id     = 0;
    line.type   = Marker::LINE_LIST;
    line.action = Marker::ADD;
    line.scale.x = 0.02f;
    line.color.r = 1.0f;
    line.color.g = 0.0f;
    line.color.b = 1.0f;
    line.color.a = 0.8f;

    MarkerArray mid_arr;
    int mid_id = 0;

    auto find_id = [&](float vx, float vy) {
      int best = -1;
      float bd = 1e6f;
      for (size_t i = 0; i < pts.size(); ++i) {
        float d = std::hypot(pts[i].x - vx, pts[i].y - vy);
        if (d < bd) {
          bd = d;
          best = ids[i];
        }
      }
      return best;
    };

    std::set<std::pair<int,int>> used;
    for (auto &t : tris) {
      std::pair<float,float> v[3] = { {t[0],t[1]}, {t[2],t[3]}, {t[4],t[5]} };
      for (int i = 0; i < 3; ++i) {
        int j = (i + 1) % 3;
        int id1 = find_id(v[i].first, v[i].second);
        int id2 = find_id(v[j].first, v[j].second);
        if (id1 < 0 || id2 < 0 || id1 == id2) {
          continue;
        }
        bool cross = (left_set.count(id1) && right_set.count(id2))
                  || (left_set.count(id2) && right_set.count(id1));
        if (!cross) {
          continue;
        }
        auto edge = std::minmax(id1, id2);
        if (!used.insert(edge).second) {
          continue;
        }

        // 길이 조건 적용
        float dx = positions_[edge.first].first  - positions_[edge.second].first;
        float dy = positions_[edge.first].second - positions_[edge.second].second;
        float dist = std::hypot(dx, dy);
        if (dist > kMaxEdge) {
          continue;  // 3m 초과 엣지는 제외
        }

        // edge line
        Point p1; p1.x = positions_[edge.first].first;
                  p1.y = positions_[edge.first].second; p1.z = 0.0;
        Point p2; p2.x = positions_[edge.second].first;
                  p2.y = positions_[edge.second].second; p2.z = 0.0;
        line.points.push_back(p1);
        line.points.push_back(p2);

        // midpoint 마커
        Point mid;
        mid.x = (p1.x + p2.x) * 0.5f;
        mid.y = (p1.y + p2.y) * 0.5f;
        mid.z = 0.0f;
        Marker m;
        m.header = line.header;
        m.ns     = "delaunay_midpoints";
        m.id     = mid_id++;
        m.type   = Marker::SPHERE;
        m.action = Marker::ADD;
        m.pose.position = mid;
        m.scale.x = m.scale.y = m.scale.z = 0.3f;
        m.color.r = 1.0f;
        m.color.g = 0.5f;
        m.color.b = 0.0f;
        m.color.a = 1.0f;
        mid_arr.markers.push_back(m);
      }
    }

    edges_arr.markers.push_back(line);
    pub_edges_->publish(edges_arr);
    pub_mid_->publish(mid_arr);
  }

  rclcpp::Subscription<MarkerArray>::SharedPtr sub_;
  rclcpp::Publisher<MarkerArray>::SharedPtr pub_edges_, pub_mid_;
  std::unordered_map<int, std::pair<double,double>> positions_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DelaunayConnector>());
  rclcpp::shutdown();
  return 0;
}