#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker
import math

class DetectBall3d:

    def __init__(self):
        rospy.init_node('detect_ball_3d', anonymous=True)

        rospy.loginfo('Detecting in 3D')

        self.ball2d_sub = rospy.Subscriber("/detected_ball", Point, self.ball_rcv_callback)
        self.ball3d_pub = rospy.Publisher("/detected_ball_3d", Point, queue_size=1)
        self.ball_marker_pub = rospy.Publisher("/ball_3d_marker", Marker, queue_size=1)

        self.h_fov = rospy.get_param("/detect_ball_3d/h_fov", 1.089)
        self.ball_radius = rospy.get_param("/detect_ball_3d/ball_radius", 0.033)
        self.aspect_ratio = rospy.get_param("/detect_ball_3d/aspect_ratio", 4.0/3.0)
        self.camera_frame = rospy.get_param("/detect_ball_3d/camera_frame", 'camera_link_optical')

        self.v_fov = self.h_fov / self.aspect_ratio

    def ball_rcv_callback(self, data):
        # Calculate angular size and consequently distance
        ang_size = data.z * self.h_fov
        d = self.ball_radius / math.atan(ang_size / 2)

        # Calculate angular and distance deviations in X and Y
        y_ang = data.y * self.v_fov / 2
        y = d * math.sin(y_ang)
        d_proj = d * math.cos(y_ang)

        x_ang = data.x * self.h_fov / 2
        x = d_proj * math.sin(x_ang)
        z = d_proj * math.cos(x_ang)

        p = Point()
        p.x = x
        p.y = y
        p.z = z
        self.ball3d_pub.publish(p)

        m = Marker()
        m.header.frame_id = self.camera_frame
        m.header.stamp = rospy.Time.now()

        m.id = 0
        m.type = Marker.SPHERE
        m.action = Marker.ADD
        m.pose.position.x = x
        m.pose.position.y = y
        m.pose.position.z = z
        m.scale.x = self.ball_radius * 2
        m.scale.y = self.ball_radius * 2
        m.scale.z = self.ball_radius * 2
        m.color.r = 0.933
        m.color.g = 1.0
        m.color.b = 0.0
        m.color.a = 1.0

        self.ball_marker_pub.publish(m)

def main():
    detect_ball_3d = DetectBall3d()
    rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
