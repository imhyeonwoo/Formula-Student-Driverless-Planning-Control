/************************************************************
 * classify_cones_by_side.cpp  (ROS 2 Humble)
 * KD‑Tree + OpenMP 버전 — 컴파일 에러 수정
 ***********************************************************/

 #include <rclcpp/rclcpp.hpp>
 #include <visualization_msgs/msg/marker.hpp>
 #include <geometry_msgs/msg/point.hpp>
 #include <custom_interface/msg/modified_float32_multi_array.hpp>
 
 #include <tf2_ros/transform_listener.h>
 #include <tf2_ros/buffer.h>
 
 #include <Eigen/Dense>
 #include <nanoflann.hpp>
 #include <algorithm>
 #include <memory>
 #include <vector>
 
 #ifdef _OPENMP
   #include <omp.h>
 #endif
 
 using rclcpp::Node;
 using visualization_msgs::msg::Marker;
 using geometry_msgs::msg::Point;
 using custom_interface::msg::ModifiedFloat32MultiArray;
 using std::placeholders::_1;
 
 /* ===== KD‑Tree adaptor (Eigen row‑major) ===== */
 using PathMat = Eigen::Matrix<double, Eigen::Dynamic, 2, Eigen::RowMajor>;
 using KDTree  = nanoflann::KDTreeEigenMatrixAdaptor<PathMat>;
 
 class ConeSideClassifier : public Node
 {
 public:
   ConeSideClassifier()
   : Node("classify_cones_by_side"),
     tf_buffer_(this->get_clock()),
     tf_listener_(tf_buffer_)
   {
     sub_path_ = create_subscription<Marker>(
         "/global_path_marker", 10,
         std::bind(&ConeSideClassifier::cbPath, this, _1));
 
     sub_cone_ = create_subscription<ModifiedFloat32MultiArray>(
         "/sorted_cones_time", 10,
         std::bind(&ConeSideClassifier::cbCones, this, _1));
 
     pub_left_  = create_publisher<Marker>("/left_cone_marker",  1);
     pub_right_ = create_publisher<Marker>("/right_cone_marker", 1);
 
     RCLCPP_INFO(get_logger(), "ConeSideClassifier (KD‑Tree) started");
   }
 
 private:
   /* ---------- 경로 콜백 ---------- */
   void cbPath(const Marker::SharedPtr msg)
   {
     if (msg->type != Marker::LINE_STRIP || msg->points.size() < 2) return;
 
     constexpr double STEP = 0.20;   // 20 cm
     PathMat new_path; new_path.resize(0, 2);
 
     double acc = 0.0;
     for (size_t i = 1; i < msg->points.size(); ++i)
     {
       Eigen::Vector2d prev(msg->points[i-1].x, msg->points[i-1].y);
       Eigen::Vector2d curr(msg->points[i  ].x, msg->points[i  ].y);
       acc += (curr - prev).norm();
       if (acc >= STEP || i + 1 == msg->points.size()) {
         new_path.conservativeResize(new_path.rows()+1, 2);
         new_path.row(new_path.rows()-1) = curr.transpose();
         acc = 0.0;
       }
     }
 
     path_pts_.swap(new_path);
     kd_tree_ = std::make_unique<KDTree>(2, std::cref(path_pts_), 10);
     kd_tree_->index->buildIndex();
 
     RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 3000,
                          "경로 갱신 & KD‑Tree 구축: %zu pts", path_pts_.rows());
   }
 
   /* ---------- 콘 콜백 ---------- */
   void cbCones(const ModifiedFloat32MultiArray::SharedPtr msg)
   {
     if (!kd_tree_) return;
 
     /* TF(reference ← os_sensor) */
     geometry_msgs::msg::TransformStamped tf;
     try {
       tf = tf_buffer_.lookupTransform("reference", "os_sensor", rclcpp::Time(0));
     } catch (const tf2::TransformException &e) {
       RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                            "TF(reference←os_sensor) 미획득… (%s)", e.what());
       return;
     }
 
     const auto &qr = tf.transform.rotation;
     Eigen::Quaterniond q(qr.w, qr.x, qr.y, qr.z);
     Eigen::Matrix3d R = q.toRotationMatrix();
     Eigen::Vector3d t(tf.transform.translation.x,
                       tf.transform.translation.y,
                       tf.transform.translation.z);
 
     /* 센서 → reference */
     std::vector<Eigen::Vector2d> cones_ref;
     cones_ref.reserve(msg->data.size() / 3);
     for (size_t i=0; i+2<msg->data.size(); i+=3) {
       Eigen::Vector3d ps(msg->data[i], msg->data[i+1], 0.0);
       Eigen::Vector3d pr = R*ps + t;
       cones_ref.emplace_back(pr.x(), pr.y());
     }
 
     /* 좌/우 분류 */
     std::vector<Eigen::Vector2d> left_pts, right_pts;
     classifyCones(cones_ref, left_pts, right_pts);
 
     /* 마커 발행 */
     auto stamp = get_clock()->now();
     publishMarker(pub_left_,  left_pts,  stamp, 0.0f, 0.3f, 1.0f);
     publishMarker(pub_right_, right_pts, stamp, 1.0f, 1.0f, 0.0f);
   }
 
   /* ---------- KD‑Tree Frenet 분류 ---------- */
   void classifyCones(const std::vector<Eigen::Vector2d>& cones,
                      std::vector<Eigen::Vector2d>& left_out,
                      std::vector<Eigen::Vector2d>& right_out) const
   {
     constexpr size_t K = 1;
     std::vector<long int> idx(K);
     std::vector<double>   d2 (K);
     size_t segN = path_pts_.rows() - 1;
 
     for (const auto &c : cones)
     {
       kd_tree_->index->knnSearch(c.data(), K, idx.data(), d2.data());
       size_t near_idx = static_cast<size_t>(idx[0]);
 
       /* 후보 세그먼트 */
       size_t best_i  = near_idx;
       double best_d2 = std::numeric_limits<double>::max();
       Eigen::Vector2d best_proj;
 
       for (int off=-1; off<=0; ++off) {
         long si = static_cast<long>(near_idx) + off;
         if (si<0 || static_cast<size_t>(si)>=segN) continue;
         size_t i = static_cast<size_t>(si);
         Eigen::Vector2d p = path_pts_.row(i);
         Eigen::Vector2d q = path_pts_.row(i+1);
         Eigen::Vector2d v = q - p;
         double v2 = v.squaredNorm(); if (v2==0.0) continue;
         double u = std::clamp(((c-p).dot(v))/v2, 0.0, 1.0);
         Eigen::Vector2d proj = p + u*v;
         double dist2 = (c - proj).squaredNorm();
         if (dist2 < best_d2) { best_d2 = dist2; best_i = i; best_proj = proj; }
       }
 
       Eigen::Vector2d t_hat = path_pts_.row(best_i+1) - path_pts_.row(best_i);
       Eigen::Vector2d d_vec = c - best_proj;
       double z = t_hat.x()*d_vec.y() - t_hat.y()*d_vec.x();
       (z>0 ? left_out : right_out).push_back(c);
     }
   }
 
   /* ---------- 마커 작성 ---------- */
   void publishMarker(const rclcpp::Publisher<Marker>::SharedPtr &pub,
                      const std::vector<Eigen::Vector2d> &pts,
                      const rclcpp::Time &stamp,
                      float r, float g, float b) const
   {
     Marker mk;
     mk.header.stamp = stamp;
     mk.header.frame_id = "reference";
     mk.ns   = "cone_side";
     mk.id   = (r > b ? 1 : 0);
     mk.type = Marker::SPHERE_LIST;
     mk.action = Marker::ADD;
     mk.scale.x = mk.scale.y = mk.scale.z = 0.30f;
     mk.color.r = r; mk.color.g = g; mk.color.b = b; mk.color.a = 1.0f;
 
     mk.points.reserve(pts.size());
     for (const auto &p : pts) {
       Point pt; pt.x = p.x(); pt.y = p.y(); pt.z = 0.0;
       mk.points.push_back(pt);
      }
      pub->publish(mk);
    }
  
    /* ---------- 멤버 ---------- */
    rclcpp::Subscription<Marker>::SharedPtr                    sub_path_;
    rclcpp::Subscription<ModifiedFloat32MultiArray>::SharedPtr sub_cone_;
    rclcpp::Publisher<Marker>::SharedPtr                       pub_left_;
    rclcpp::Publisher<Marker>::SharedPtr                       pub_right_;
  
    tf2_ros::Buffer            tf_buffer_;
    tf2_ros::TransformListener tf_listener_;
  
    PathMat                    path_pts_;   // [N,2]  (row‑major)
    std::unique_ptr<KDTree>    kd_tree_;    // KD‑Tree 인덱스
  };
  
  /* ---------------- main ---------------- */
  int main(int argc, char **argv)
  {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ConeSideClassifier>());
    rclcpp::shutdown();
    return 0;
  }
  