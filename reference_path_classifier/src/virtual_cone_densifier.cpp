/************************************************************
 * virtual_cone_densifier.cpp  (ROS 2 Humble)
 * /left|right_cone_marker/real -> /left|right_cone_marker/virtual 생성
 * - 글로벌 경로(LINE_STRIP) 기준 Frenet(s, n) 매핑 후
 *   인접 콘 사이의 큰 갭을 s-도메인 선형보간으로 가상 콘 채움
 ***********************************************************/
#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <geometry_msgs/msg/point.hpp>

#include <Eigen/Dense>
#include <nanoflann.hpp>
#include <algorithm>
#include <vector>
#include <memory>
#include <limits>
#include <cmath>
#include <string>

using rclcpp::Node;
using visualization_msgs::msg::Marker;
using geometry_msgs::msg::Point;

/* ===== KD-Tree adaptor (Eigen row-major) ===== */
using PathMat = Eigen::Matrix<double, Eigen::Dynamic, 2, Eigen::RowMajor>;
using KDTree  = nanoflann::KDTreeEigenMatrixAdaptor<PathMat>;

class VirtualConeDensifier : public Node {
public:
  VirtualConeDensifier()
  : Node("virtual_cone_densifier")
  {
    // ---- Parameters ----
    densify_step_ = declare_parameter<double>("densify_step", 0.5);     // m (가상 콘 간격)
    max_gap_      = declare_parameter<double>("max_gap",      1.2);     // m (이보다 큰 갭만 채움)
    n_max_        = declare_parameter<double>("n_max",        6.0);     // m (허용 측방 한계)
    path_step_    = declare_parameter<double>("path_step",    0.20);    // m (경로 리샘플 간격)

    // ---- Subs ----
    sub_path_ = create_subscription<Marker>(
      "/global_path_marker", 10,
      std::bind(&VirtualConeDensifier::cbPath, this, std::placeholders::_1));

    sub_left_real_ = create_subscription<Marker>(
      "/left_cone_marker/real", 10,
      std::bind(&VirtualConeDensifier::cbLeftReal, this, std::placeholders::_1));

    sub_right_real_ = create_subscription<Marker>(
      "/right_cone_marker/real", 10,
      std::bind(&VirtualConeDensifier::cbRightReal, this, std::placeholders::_1));

    // ---- Pubs (RViz 친화: transient_local) ----
    auto qos = rclcpp::QoS(1).reliable().transient_local();

    pub_left_virtual_  = create_publisher<Marker>("/left_cone_marker/virtual", qos);
    pub_right_virtual_ = create_publisher<Marker>("/right_cone_marker/virtual", qos);

    RCLCPP_INFO(get_logger(), "VirtualConeDensifier started (densify_step=%.2f, max_gap=%.2f, n_max=%.2f)",
                densify_step_, max_gap_, n_max_);
  }

private:
  /* ---------- Path callback: build resampled path + KD-tree + s-cumulative ---------- */
  void cbPath(const Marker::SharedPtr msg) {
    if (msg->type != Marker::LINE_STRIP || msg->points.size() < 2) return;

    PathMat new_path; new_path.resize(0, 2);

    // 간단 리샘플: 누적 거리 기준으로 STEP 초과 시 점 채택
    double acc = 0.0;
    Eigen::Vector2d prev(msg->points[0].x, msg->points[0].y);
    // 시작점 포함
    new_path.conservativeResize(new_path.rows()+1, 2);
    new_path.row(new_path.rows()-1) = prev.transpose();

    for (size_t i = 1; i < msg->points.size(); ++i) {
      Eigen::Vector2d curr(msg->points[i].x, msg->points[i].y);
      acc += (curr - prev).norm();
      if (acc >= path_step_ || i + 1 == msg->points.size()) {
        new_path.conservativeResize(new_path.rows()+1, 2);
        new_path.row(new_path.rows()-1) = curr.transpose();
        acc = 0.0;
      }
      prev = curr;
    }

    if (new_path.rows() < 2) return;

    // s_cumulate
    std::vector<double> s(new_path.rows(), 0.0);
    for (int i = 1; i < new_path.rows(); ++i) {
      s[i] = s[i-1] + (new_path.row(i) - new_path.row(i-1)).norm();
    }

    path_pts_.swap(new_path);
    s_cum_.swap(s);

    kd_tree_ = std::make_unique<KDTree>(2, std::cref(path_pts_), 10);
    kd_tree_->index->buildIndex();

    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 3000,
      "경로 갱신 & KD-Tree 구축: %zu pts, length=%.1f m", path_pts_.rows(), s_cum_.empty()?0.0:s_cum_.back());

    // 경로가 갱신되면 즉시 재계산
    recomputeAndPublish();
  }

  /* ---------- Real cones callbacks ---------- */
  void cbLeftReal(const Marker::SharedPtr msg) {
    left_real_xy_.clear();
    if (msg->type == Marker::SPHERE_LIST) {
      left_real_xy_.reserve(msg->points.size());
      for (const auto& p : msg->points) left_real_xy_.emplace_back(p.x, p.y);
    }
    recomputeAndPublish();
  }

  void cbRightReal(const Marker::SharedPtr msg) {
    right_real_xy_.clear();
    if (msg->type == Marker::SPHERE_LIST) {
      right_real_xy_.reserve(msg->points.size());
      for (const auto& p : msg->points) right_real_xy_.emplace_back(p.x, p.y);
    }
    recomputeAndPublish();
  }

  /* ---------- Core: recompute virtual cones for both sides ---------- */
  void recomputeAndPublish() {
    if (!kd_tree_ || path_pts_.rows() < 2) return;

    // 좌/우 각각: real → (s,n) 정렬 → 갭 보간 → virtual (x,y)
    std::vector<Eigen::Vector2d> left_virtual, right_virtual;

    computeVirtualForSide(/*is_left=*/true,  left_real_xy_,  left_virtual);
    computeVirtualForSide(/*is_left=*/false, right_real_xy_, right_virtual);

    // Publish
    auto stamp = get_clock()->now();
    publishMarker(pub_left_virtual_,  left_virtual,  stamp, /*id=*/110, /*r=*/0.0f, /*g=*/0.3f, /*b=*/1.0f);
    publishMarker(pub_right_virtual_, right_virtual, stamp, /*id=*/111, /*r=*/1.0f, /*g=*/1.0f, /*b=*/0.0f);
  }

  struct SN {
    double s;
    double n;
  };

  void computeVirtualForSide(bool is_left,
                             const std::vector<Eigen::Vector2d>& real_xy,
                             std::vector<Eigen::Vector2d>& out_virtual) const
  {
    out_virtual.clear();
    if (real_xy.size() < 2) return;

    // 1) real points -> (s, n) with filtering by side & n_max
    std::vector<SN> sn;
    sn.reserve(real_xy.size());

    for (const auto& c : real_xy) {
      double s_proj, n;
      if (!projectToPath(c, s_proj, n)) continue;
      if (std::abs(n) > n_max_) continue;
      if (is_left && n <  0.0) continue;  // 좌는 n>=0
      if (!is_left && n > 0.0) continue;  // 우는 n<=0
      sn.push_back({s_proj, n});
    }

    if (sn.size() < 2) return;

    std::sort(sn.begin(), sn.end(), [](const SN& a, const SN& b){ return a.s < b.s; });

    // 2) densify between consecutive (s_i, n_i)
    for (size_t i = 0; i + 1 < sn.size(); ++i) {
      const double s0 = sn[i].s,   s1 = sn[i+1].s;
      const double n0 = sn[i].n,   n1 = sn[i+1].n;
      const double gap = s1 - s0;

      if (gap <= max_gap_) continue;

      for (double s_new = s0 + densify_step_; s_new < s1 - 1e-6; s_new += densify_step_) {
        double ratio = (s_new - s0) / gap;
        double n_new = (1.0 - ratio) * n0 + ratio * n1;

        Eigen::Vector2d center, t_hat, n_hat;
        if (!pointOnPathAtS(s_new, center, t_hat, n_hat)) continue;

        Eigen::Vector2d pos = center + n_new * n_hat;
        out_virtual.push_back(pos);
      }
    }
  }

  /* ---------- Projection of a point to the path: return s-projection & lateral n ---------- */
  bool projectToPath(const Eigen::Vector2d& c, double& s_out, double& n_out) const {
    constexpr size_t K = 1;
    std::vector<long int> idx(K);
    std::vector<double>   d2 (K);

    kd_tree_->index->knnSearch(c.data(), K, idx.data(), d2.data());
    size_t near_idx = static_cast<size_t>(idx[0]);

    size_t segN = (path_pts_.rows() > 1) ? (path_pts_.rows() - 1) : 0;
    if (segN == 0) return false;

    size_t best_i  = near_idx;
    double best_d2 = std::numeric_limits<double>::max();
    double best_u  = 0.0;

    for (int off = -1; off <= 0; ++off) {
      long si = static_cast<long>(near_idx) + off;
      if (si < 0 || static_cast<size_t>(si) >= segN) continue;
      size_t i = static_cast<size_t>(si);
      Eigen::Vector2d p = path_pts_.row(i);
      Eigen::Vector2d q = path_pts_.row(i+1);
      Eigen::Vector2d v = q - p;
      double v2 = v.squaredNorm(); if (v2 == 0.0) continue;
      double u = std::clamp(((c - p).dot(v)) / v2, 0.0, 1.0);
      Eigen::Vector2d proj = p + u * v;
      double dist2 = (c - proj).squaredNorm();
      if (dist2 < best_d2) { best_d2 = dist2; best_i = i; best_u = u; }
    }

    // s_out
    double seg_len = (path_pts_.row(best_i+1) - path_pts_.row(best_i)).norm();
    s_out = s_cum_[best_i] + best_u * seg_len;

    // n_out (sign by cross product with tangent)
    Eigen::Vector2d p = path_pts_.row(best_i);
    Eigen::Vector2d q = path_pts_.row(best_i+1);
    Eigen::Vector2d v = q - p;
    double vnorm = v.norm();
    if (vnorm < 1e-9) { n_out = 0.0; return true; }
    Eigen::Vector2d t_hat = v / vnorm;

    Eigen::Vector2d proj = p + best_u * v;
    Eigen::Vector2d d_vec = c - proj;
    double z = t_hat.x() * d_vec.y() - t_hat.y() * d_vec.x();
    n_out = (z >= 0.0 ? 1.0 : -1.0) * std::sqrt(best_d2);
    return true;
  }

  /* ---------- Retrieve center, tangent, normal at given s ---------- */
  bool pointOnPathAtS(double s_query,
                      Eigen::Vector2d& center,
                      Eigen::Vector2d& t_hat,
                      Eigen::Vector2d& n_hat) const
  {
    if (s_cum_.empty() || path_pts_.rows() < 2) return false;
    if (s_query <= s_cum_.front()) {
      center = path_pts_.row(0);
      Eigen::Vector2d v = path_pts_.row(1) - path_pts_.row(0);
      double vn = v.norm(); if (vn < 1e-9) return false;
      t_hat = v / vn;
    } else if (s_query >= s_cum_.back()) {
      center = path_pts_.row(path_pts_.rows()-1);
      Eigen::Vector2d v = path_pts_.row(path_pts_.rows()-1) - path_pts_.row(path_pts_.rows()-2);
      double vn = v.norm(); if (vn < 1e-9) return false;
      t_hat = v / vn;
    } else {
      // lower_bound to find segment
      auto it = std::upper_bound(s_cum_.begin(), s_cum_.end(), s_query);
      size_t j = std::distance(s_cum_.begin(), it) - 1; // s_cum_[j] <= s_query < s_cum_[j+1]
      double seg_len = s_cum_[j+1] - s_cum_[j];
      if (seg_len < 1e-9) return false;
      double u = (s_query - s_cum_[j]) / seg_len;

      Eigen::Vector2d p = path_pts_.row(j);
      Eigen::Vector2d q = path_pts_.row(j+1);
      center = p + u * (q - p);

      Eigen::Vector2d v = q - p;
      double vn = v.norm(); if (vn < 1e-9) return false;
      t_hat = v / vn;
    }

    n_hat = Eigen::Vector2d(-t_hat.y(), t_hat.x()); // left-hand normal (좌측이 +)
    return true;
  }

  void publishMarker(const rclcpp::Publisher<Marker>::SharedPtr& pub,
                     const std::vector<Eigen::Vector2d>& pts,
                     const rclcpp::Time& stamp,
                     int id, float r, float g, float b) const
  {
    Marker mk;
    mk.header.stamp    = stamp;
    mk.header.frame_id = "map";
    mk.ns   = "cone_side_virtual";
    mk.id   = id;
    mk.type = Marker::SPHERE_LIST;
    mk.action = Marker::ADD;
    mk.scale.x = mk.scale.y = mk.scale.z = 0.20f; // 가상: 더 작게
    mk.color.r = r; mk.color.g = g; mk.color.b = b; mk.color.a = 0.65f;

    mk.points.reserve(pts.size());
    for (const auto& p : pts) {
      Point pt; pt.x = p.x(); pt.y = p.y(); pt.z = 0.0;
      mk.points.push_back(pt);
    }
    pub->publish(mk);
  }

  // ---- Members ----
  rclcpp::Subscription<Marker>::SharedPtr sub_path_;
  rclcpp::Subscription<Marker>::SharedPtr sub_left_real_;
  rclcpp::Subscription<Marker>::SharedPtr sub_right_real_;

  rclcpp::Publisher<Marker>::SharedPtr pub_left_virtual_;
  rclcpp::Publisher<Marker>::SharedPtr pub_right_virtual_;

  PathMat                 path_pts_;   // [N,2]
  std::vector<double>     s_cum_;      // [N]
  std::unique_ptr<KDTree> kd_tree_;

  std::vector<Eigen::Vector2d> left_real_xy_, right_real_xy_;

  double densify_step_, max_gap_, n_max_, path_step_;
};

/* ---------------- main ---------------- */
int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<VirtualConeDensifier>());
  rclcpp::shutdown();
  return 0;
}
