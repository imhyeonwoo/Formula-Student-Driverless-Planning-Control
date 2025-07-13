/****************************************************************
 * speed_planner.cpp  (ROS 2 Humble · 2025-07-09)
 *
 * 입력
 *   • /final_waypoints   (MarkerArray – 좌표만 사용)
 *   • /vehicle_speed     (std_msgs/Float32  현재 차량 속도 [m/s])
 *
 * 출력
 *   • /desired_speed_profile (Float32MultiArray …  각 웨이포인트별 v_target)
 *   • /target_speed          (Float32 … 1-step ahead v_next)
 *   • (선택) /planned_waypoints (MarkerArray – 디버그 시각화)
 *
 * v_target = √(a_lat_max / κ̄)  (κ̄ : look-ahead 평균 곡률)
 * v_next   = a_long_max 로 한계 가·감속 후속 속도
 ****************************************************************/

 #include <rclcpp/rclcpp.hpp>
 #include <std_msgs/msg/float32.hpp>
 #include <std_msgs/msg/float32_multi_array.hpp>
 #include <visualization_msgs/msg/marker_array.hpp>
 
 #include <Eigen/Dense>
 
 #include <vector>
 #include <algorithm>
 #include <cmath>
 
 using rclcpp::Node;
 using std::placeholders::_1;
 using geometry_msgs::msg::Point;
 using visualization_msgs::msg::Marker;
 using visualization_msgs::msg::MarkerArray;
 using std_msgs::msg::Float32;
 using std_msgs::msg::Float32MultiArray;
 using Eigen::Vector2d;
 
 /* ───────────────────────── 보조: 세 점 곡률 ───────────────────────── */
 static double curvature(const Vector2d &p_prev,
                         const Vector2d &p_curr,
                         const Vector2d &p_next)
 {
   double a = (p_curr - p_prev).norm();
   double b = (p_next - p_curr).norm();
   double c = (p_next - p_prev ).norm();
   if (a < 1e-3 || b < 1e-3 || c < 1e-3) return 0.0;
 
   double s = 0.5 * (a + b + c);                          // 반둘레
   double A2 = s * (s - a) * (s - b) * (s - c);           // 헤론 정리
   if (A2 <= 0.0) return 0.0;
   double A = std::sqrt(A2);
   return 4.0 * A / (a * b * c);                          // κ
 }
 
 /* ─────────────────────────── 클래스 ──────────────────────────────── */
 class SpeedPlanner : public Node
 {
 public:
   SpeedPlanner()
   : Node("speed_planner")
   {
     /* ───── 파라미터 ───── */
     a_lat_max_   = declare_parameter("a_lat_max",   3.0);  // [m/s²]
     a_long_max_  = declare_parameter("a_long_max",  2.0);
     lookahead_n_ = declare_parameter("lookahead_n", 10);
     dt_default_  = declare_parameter("control_dt",  0.05); // [s]
     v_min_       = declare_parameter("min_speed",   0.5);
     v_max_       = declare_parameter("max_speed",  20.0);
     publish_debug_ = declare_parameter("publish_debug_marker", false);
 
     /* ───── Pub/Sub ───── */
     sub_wp_ = create_subscription<MarkerArray>(
         "/final_waypoints", 10, std::bind(&SpeedPlanner::cbWaypoints, this, _1));
 
     sub_vel_ = create_subscription<Float32>(
         "/vehicle_speed", 10,
         [&](Float32::SharedPtr m) { v_measured_ = m->data; });
 
     pub_profile_ = create_publisher<Float32MultiArray>(
         "/desired_speed_profile", 10);
     pub_target_  = create_publisher<Float32>("/target_speed", 10);
 
     if (publish_debug_)
       pub_debug_ = create_publisher<MarkerArray>("/planned_waypoints", 10);
 
     /* ───── 타이머 ───── */
     create_wall_timer(std::chrono::duration<double>(dt_default_),
                       std::bind(&SpeedPlanner::timerCB, this));
 
     RCLCPP_INFO(get_logger(), "SpeedPlanner ready");
   }
 
 private:
   /* ───────── Waypoint 수신 ───────── */
   void cbWaypoints(const MarkerArray::SharedPtr msg)
   {
     pts_.clear();
     for (const auto &mk : msg->markers)
       if (mk.type == Marker::SPHERE)
         pts_.emplace_back(mk.pose.position.x, mk.pose.position.y);
   }
 
   /* ───────── 주기 계산 ───────── */
   void timerCB()
   {
     const size_t N = pts_.size();
     if (N < 3) return;
 
     /* 1) 곡률 κ_i */
     std::vector<double> kappa(N, 0.0);
     for (size_t i = 1; i + 1 < N; ++i)
       kappa[i] = curvature(pts_[i - 1], pts_[i], pts_[i + 1]);
 
     /* 2) look-ahead 평균 κ̄ */
     std::vector<double> k_avg(N, 0.0);
     for (size_t i = 0; i < N; ++i) {
       double sum = 0.0; int cnt = 0;
       for (size_t j = i; j < std::min(N, i + lookahead_n_); ++j) {
         sum += kappa[j]; ++cnt;
       }
       k_avg[i] = (cnt ? sum / cnt : 0.0);
     }
 
     /* 3) v_target */
     std::vector<double> v_target(N, v_max_);
     for (size_t i = 0; i < N; ++i) {
       if (k_avg[i] > 1e-6)
         v_target[i] = std::clamp(std::sqrt(a_lat_max_ / k_avg[i]),
                                  v_min_, v_max_);
     }
 
     /* 4) 1-step v_next */
     double v_next = v_target[0];
     double dt = dt_default_;
     if (v_measured_ >= 0.0) {
       if (v_target[0] > v_measured_)
         v_next = std::min(v_target[0], v_measured_ + a_long_max_ * dt);
       else
         v_next = std::max(v_target[0], v_measured_ - a_long_max_ * dt);
     }
 
     /* 5) 퍼블리시 */
     publishSpeedProfile(v_target);
     Float32 vmsg; vmsg.data = static_cast<float>(v_next);
     pub_target_->publish(vmsg);
 
     if (publish_debug_) publishDebugMarkers(v_target);
   }
 
   /* ───────── 속도 배열 퍼블리시 ───────── */
   void publishSpeedProfile(const std::vector<double>& vec)
   {
     Float32MultiArray arr;
     arr.data.reserve(vec.size());
     for (double v : vec) arr.data.push_back(static_cast<float>(v));
     pub_profile_->publish(arr);
   }
 
   /* ───────── 디버그 MarkerArray ───────── */
   void publishDebugMarkers(const std::vector<double>& v_tar)
   {
     rclcpp::Time stamp = get_clock()->now();
     MarkerArray out;
 
     for (size_t i = 0; i < pts_.size(); ++i)
     {
       /* 위치 Sphere */
       Marker sp;
       sp.header.frame_id = "os_sensor";
       sp.header.stamp    = stamp;
       sp.ns   = "planned_wp"; sp.id = static_cast<int>(i);
       sp.type = Marker::SPHERE; sp.action = Marker::ADD;
       sp.pose.position.x = pts_[i].x(); sp.pose.position.y = pts_[i].y();
       sp.pose.orientation.w = 1.0;
       sp.scale.x = sp.scale.y = sp.scale.z = 0.15;
       sp.color.r = 0.0; sp.color.g = 0.4; sp.color.b = 1.0; sp.color.a = 1.0;
       out.markers.push_back(sp);
 
       /* 속도 막대 Cube */
       Marker cb = sp;
       cb.ns = "planned_speed"; cb.type = Marker::CUBE;
       cb.scale.x = cb.scale.y = 0.2;
       cb.scale.z = std::max(v_tar[i], 0.01);
       cb.pose.position.z = cb.scale.z * 0.5;
       cb.color.r = 1.0; cb.color.g = 0.6; cb.color.b = 0.0; cb.color.a = 0.8;
       out.markers.push_back(cb);
     }
     pub_debug_->publish(out);
   }
 
   /* ───────── 멤버 ───────── */
   rclcpp::Subscription<MarkerArray>::SharedPtr sub_wp_;
   rclcpp::Subscription<Float32>::SharedPtr     sub_vel_;
   rclcpp::Publisher<Float32MultiArray>::SharedPtr pub_profile_;
   rclcpp::Publisher<Float32>::SharedPtr            pub_target_;
   rclcpp::Publisher<MarkerArray>::SharedPtr        pub_debug_;
 
   /* 파라미터 */
   double a_lat_max_, a_long_max_, dt_default_;
   double v_min_, v_max_;
   int    lookahead_n_;
   bool   publish_debug_;
   double v_measured_{-1.0};
 
   std::vector<Vector2d> pts_;
 };
 
 /* ────────────────────── main ────────────────────── */
 int main(int argc, char **argv)
 {
   rclcpp::init(argc, argv);
   rclcpp::spin(std::make_shared<SpeedPlanner>());
   rclcpp::shutdown();
   return 0;
 }
 