/************************************************************
 * classify_cones_by_side.cpp  (ROS 2 Humble)
 *
 * • /global_path_marker  (Marker, LINE_STRIP, frame = reference)
 * • /sorted_cones_time   (ModifiedFloat32MultiArray, frame = os_sensor)
 *         └─ TF(reference←os_sensor) 변환 → reference 평면 좌표
 *         └─ Frenet 외적(z) 기준 좌/우 분류
 *         └─ /left_cone_marker  (SPHERE_LIST, blue )
 *         └─ /right_cone_marker (SPHERE_LIST, yellow)
 ***********************************************************/

 #include <rclcpp/rclcpp.hpp>

 #include <visualization_msgs/msg/marker.hpp>
 #include <geometry_msgs/msg/point.hpp>
 #include <custom_interface/msg/modified_float32_multi_array.hpp>
 
 #include <tf2_ros/transform_listener.h>
 #include <tf2_ros/buffer.h>
 
 #include <Eigen/Dense>
 #include <algorithm>    // std::clamp
 #include <limits>       // std::numeric_limits
 
 using rclcpp::Node;
 using visualization_msgs::msg::Marker;
 using geometry_msgs::msg::Point;
 using custom_interface::msg::ModifiedFloat32MultiArray;
 using std::placeholders::_1;
 
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
 
     RCLCPP_INFO(get_logger(), "ConeSideClassifier started");
   }
 
 private:
   /* -------- 경로 콜백 -------- */
   void cbPath(const Marker::SharedPtr msg)
   {
     if (msg->type != Marker::LINE_STRIP || msg->points.empty()) return;
 
     size_t N = msg->points.size();
     path_pts_.resize(N, 2);
     for (size_t i = 0; i < N; ++i) {
       path_pts_(i, 0) = msg->points[i].x;
       path_pts_(i, 1) = msg->points[i].y;
     }
     RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 3000,
                          "경로 수신: %zu pts", N);
   }
 
   /* -------- 콘 콜백 -------- */
   void cbCones(const ModifiedFloat32MultiArray::SharedPtr msg)
   {
     if (path_pts_.rows() < 2) return;   // 경로 없음
 
     /* TF(reference ← os_sensor) */
     geometry_msgs::msg::TransformStamped tf;
     try {
       tf = tf_buffer_.lookupTransform("reference", "os_sensor",
                                       rclcpp::Time(0));
     } catch (const tf2::TransformException &e) {
       RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                            "TF(reference←os_sensor) 미획득… (%s)", e.what());
       return;
     }
 
     /* Quaternion → Eigen 행렬 */
     const auto &qr = tf.transform.rotation;
     Eigen::Quaterniond q_eig(qr.w, qr.x, qr.y, qr.z);   // (w,x,y,z)
     Eigen::Matrix3d    R   = q_eig.toRotationMatrix();
     Eigen::Vector3d    t(tf.transform.translation.x,
                          tf.transform.translation.y,
                          tf.transform.translation.z);
 
     /* os_sensor → reference 변환 */
     std::vector<Eigen::Vector2d> cones_ref;
     const std::vector<float> &data = msg->data;
     for (size_t i = 0; i + 2 < data.size(); i += 3) {
       Eigen::Vector3d ps(data[i], data[i + 1], 0.0);
       Eigen::Vector3d pr = R * ps + t;
       cones_ref.emplace_back(pr.x(), pr.y());
     }
 
     /* 좌/우 분류 */
     std::vector<Eigen::Vector2d> left_pts, right_pts;
     classifyCones(cones_ref, left_pts, right_pts);
 
     /* 마커 발행 */
     rclcpp::Time stamp = this->get_clock()->now();
     publishMarker(pub_left_,  left_pts,  stamp, 0.0, 0.3, 1.0); // blue
     publishMarker(pub_right_, right_pts, stamp, 1.0, 1.0, 0.0); // yellow
   }
 
   /* -------- Frenet 분류 -------- */
   void classifyCones(const std::vector<Eigen::Vector2d>& cones,
                      std::vector<Eigen::Vector2d>& left_out,
                      std::vector<Eigen::Vector2d>& right_out)
   {
     size_t segN = path_pts_.rows() - 1;
     for (const auto &c : cones)
     {
       double best_d2 = std::numeric_limits<double>::max();
       size_t best_i  = 0;
       Eigen::Vector2d best_proj;
 
       /* 최근접 세그먼트 탐색 */
       for (size_t i = 0; i < segN; ++i)
       {
         Eigen::Vector2d p = path_pts_.row(i);
         Eigen::Vector2d q = path_pts_.row(i + 1);
         Eigen::Vector2d v = q - p;
         double v2 = v.squaredNorm();
         if (v2 == 0.0) continue;
 
         double u = std::clamp(((c - p).dot(v)) / v2, 0.0, 1.0);
         Eigen::Vector2d proj = p + u * v;
         double dist2 = (c - proj).squaredNorm();
         if (dist2 < best_d2) {
           best_d2  = dist2;
           best_i   = i;
           best_proj = proj;
         }
       }
 
       /* 외적 z 값 */
       Eigen::Vector2d t_hat = path_pts_.row(best_i + 1) - path_pts_.row(best_i);
       Eigen::Vector2d d_vec = c - best_proj;
       double z = t_hat.x() * d_vec.y() - t_hat.y() * d_vec.x();
       (z > 0 ? left_out : right_out).push_back(c);
     }
   }
 
   /* -------- 마커 작성 -------- */
   void publishMarker(const rclcpp::Publisher<Marker>::SharedPtr &pub,
                      const std::vector<Eigen::Vector2d> &pts,
                      const rclcpp::Time &stamp,
                      float r, float g, float b)
   {
     Marker mk;
     mk.header.stamp    = stamp;
     mk.header.frame_id = "reference";
     mk.ns   = "cone_side";
     mk.id   = (r > b ? 1 : 0);                // blue=0, yellow=1
     mk.type = Marker::SPHERE_LIST;
     mk.action = Marker::ADD;
     mk.scale.x = mk.scale.y = mk.scale.z = 0.3;
     mk.color.r = r; mk.color.g = g; mk.color.b = b; mk.color.a = 1.0;
 
     mk.points.reserve(pts.size());
     for (const auto &p : pts) {
       Point pt;
       pt.x = p.x(); pt.y = p.y(); pt.z = 0.0;
       mk.points.push_back(pt);
     }
     pub->publish(mk);
   }
 
   /* ------- 멤버 ------- */
   rclcpp::Subscription<Marker>::SharedPtr                sub_path_;
   rclcpp::Subscription<ModifiedFloat32MultiArray>::SharedPtr sub_cone_;
   rclcpp::Publisher<Marker>::SharedPtr                   pub_left_;
   rclcpp::Publisher<Marker>::SharedPtr                   pub_right_;
 
   tf2_ros::Buffer               tf_buffer_;
   tf2_ros::TransformListener    tf_listener_;
 
   Eigen::MatrixXd path_pts_;   // [N,2]
 };
 
 /* ---------------------------------------------------------- */
 int main(int argc, char **argv)
 {
   rclcpp::init(argc, argv);
   rclcpp::spin(std::make_shared<ConeSideClassifier>());
   rclcpp::shutdown();
   return 0;
 }
 