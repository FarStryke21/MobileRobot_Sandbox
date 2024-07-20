#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image
from geometry_msgs.msg import Point
from cv_bridge import CvBridge, CvBridgeError
import process_image as proc

class DetectBall:

    def __init__(self):
        rospy.init_node('detect_ball', anonymous=True)
        self.bridge = CvBridge()
        
        rospy.loginfo('Looking for the ball...')
        self.image_sub = rospy.Subscriber("/color/image_raw", Image, self.callback)
        self.image_out_pub = rospy.Publisher("/image_out", Image, queue_size=1)
        self.image_tuning_pub = rospy.Publisher("/image_tuning", Image, queue_size=1)
        self.ball_pub = rospy.Publisher("/detected_ball", Point, queue_size=1)

        self.tuning_mode = rospy.get_param('/detect_ball/tuning_mode', False)
        rospy.loginfo(f"Tuning Mode set to : {self.tuning_mode}")

        self.tuning_params = {
            'x_min': rospy.get_param('/detect_ball/x_min', 0),
            'x_max': rospy.get_param('/detect_ball/x_max', 100),
            'y_min': rospy.get_param('/detect_ball/y_min', 0),
            'y_max': rospy.get_param('/detect_ball/y_max', 100),
            'h_min': rospy.get_param('/detect_ball/h_min', 0),
            'h_max': rospy.get_param('/detect_ball/h_max', 180),
            's_min': rospy.get_param('/detect_ball/s_min', 0),
            's_max': rospy.get_param('/detect_ball/s_max', 255),
            'v_min': rospy.get_param('/detect_ball/v_min', 0),
            'v_max': rospy.get_param('/detect_ball/v_max', 255),
            'sz_min': rospy.get_param('/detect_ball/sz_min', 0),
            'sz_max': rospy.get_param('/detect_ball/sz_max', 100)
        }
        rospy.loginfo(self.tuning_params)

        if self.tuning_mode:
            proc.create_tuning_window(self.tuning_params)

    def callback(self, data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
            # print("Image received!")
        except CvBridgeError as e:
            rospy.logerr(e)
            return

        try:
            if self.tuning_mode:
                self.tuning_params = proc.get_tuning_params()
            # print("Finding circles...")
            keypoints_norm, out_image, tuning_image = proc.find_circles(cv_image, self.tuning_params)
            # print("Circles Found...")
            img_to_pub = self.bridge.cv2_to_imgmsg(out_image, "bgr8")
            img_to_pub.header = data.header
            self.image_out_pub.publish(img_to_pub)

            img_to_pub = self.bridge.cv2_to_imgmsg(tuning_image, "bgr8")
            img_to_pub.header = data.header
            self.image_tuning_pub.publish(img_to_pub)

            point_out = Point()

            for i, kp in enumerate(keypoints_norm):
                x = kp.pt[0]
                y = kp.pt[1]
                s = kp.size

                # rospy.loginfo(f"Pt {i}: ({x},{y},{s})")

                if s > point_out.z:
                    point_out.x = x
                    point_out.y = y
                    point_out.z = s

            if point_out.z > 0:
                self.ball_pub.publish(point_out)
        except CvBridgeError as e:
            rospy.logerr(e)

def main():
    detect_ball = DetectBall()
    rate = rospy.Rate(10)  # 10 Hz

    while not rospy.is_shutdown():
        proc.wait_on_gui()
        rate.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
