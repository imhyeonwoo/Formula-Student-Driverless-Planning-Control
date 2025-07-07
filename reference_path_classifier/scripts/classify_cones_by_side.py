#!/usr/bin/env python3
import math, rclpy, numpy as np
from rclpy.node import Node
from visualization_msgs.msg import MarkerArray, Marker
from nav_msgs.msg import Path

class ConeSideClassifier(Node):
    def __init__(self):
        super().__init__("classify_cones_by_side")
        self.ref_vec = None; self.ref_pt = None
        self.left_pub  = self.create_publisher(MarkerArray,"left_cones",10)
        self.right_pub = self.create_publisher(MarkerArray,"right_cones",10)
        self.create_subscription(Path, "reference_path", self.cb_path, 10)
        self.create_subscription(MarkerArray, "grey_cones", self.cb_cones, 10)

    def cb_path(self, msg: Path):
        if len(msg.poses) < 2: return
        p0, p1 = msg.poses[0].pose.position, msg.poses[-1].pose.position
        self.ref_pt = np.array([p0.x, p0.y])
        v = np.array([p1.x-p0.x, p1.y-p0.y])
        if np.linalg.norm(v)==0: return
        self.ref_vec = v / np.linalg.norm(v)

    def cb_cones(self, arr: MarkerArray):
        if self.ref_vec is None: return
        left_arr, right_arr = MarkerArray(), MarkerArray()
        now = self.get_clock().now().to_msg()
        for m in arr.markers:
            pt = np.array([m.pose.position.x, m.pose.position.y])
            rel = pt - self.ref_pt
            cross = self.ref_vec[0]*rel[1] - self.ref_vec[1]*rel[0]
            target = left_arr if cross > 0 else right_arr
            new_m = Marker()
            new_m.header.stamp=now; new_m.header.frame_id="reference"
            new_m.ns="left" if cross>0 else "right"; new_m.id=m.id
            new_m.type=new_m.CYLINDER; new_m.action=new_m.ADD
            new_m.pose = m.pose; new_m.scale = m.scale
            if cross>0:  new_m.color.r,new_m.color.g,new_m.color.b = 0.0,0.0,1.0
            else:        new_m.color.r,new_m.color.g,new_m.color.b = 1.0,0.0,0.0
            new_m.color.a = 1.0
            target.markers.append(new_m)
        self.left_pub.publish(left_arr); self.right_pub.publish(right_arr)

def main(): rclpy.init(); rclpy.spin(ConeSideClassifier()); rclpy.shutdown()
if __name__ == "__main__": main()
