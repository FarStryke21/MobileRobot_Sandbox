#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Point
from geometry_msgs.msg import Twist
import time

class FollowBall:

    def __init__(self):
        rospy.init_node('follow_ball', anonymous=True)

        self.subscription = rospy.Subscriber('/detected_ball', Point, self.listener_callback)
        self.publisher_ = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

        self.rcv_timeout_secs = rospy.get_param('/follow_ball/rcv_timeout_secs', 1.0)
        self.angular_chase_multiplier = rospy.get_param('/follow_ball/angular_chase_multiplier', 0.7)
        self.forward_chase_speed = rospy.get_param('/follow_ball/forward_chase_speed', 0.1)
        self.search_angular_speed = rospy.get_param('/follow_ball/search_angular_speed', 0.5)
        self.max_size_thresh = rospy.get_param('/follow_ball/max_size_thresh', 0.1)
        self.filter_value = rospy.get_param('/follow_ball/filter_value', 0.9)

        self.target_val = 0.0
        self.target_dist = 0.0
        self.lastrcvtime = time.time() - 10000

        self.timer = rospy.Timer(rospy.Duration(0.1), self.timer_callback)

    def timer_callback(self, event):
        msg = Twist()
        if (time.time() - self.lastrcvtime < self.rcv_timeout_secs):
            rospy.loginfo('Target: {}'.format(self.target_val))
            print(self.target_dist)
            if (self.target_dist < self.max_size_thresh):
                msg.linear.x = self.forward_chase_speed
            msg.angular.z = -self.angular_chase_multiplier * self.target_val
        else:
            rospy.loginfo('Target lost')
            msg.angular.z = self.search_angular_speed
        self.publisher_.publish(msg)

    def listener_callback(self, msg):
        f = self.filter_value
        self.target_val = self.target_val * f + msg.x * (1 - f)
        self.target_dist = self.target_dist * f + msg.z * (1 - f)
        self.lastrcvtime = time.time()
        # rospy.loginfo('Received: {} {}'.format(msg.x, msg.y))

def main():
    follow_ball = FollowBall()
    rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
