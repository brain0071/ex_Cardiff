#!/usr/bin/env python
import rospy
import tf
from nav_msgs.msg import Odometry

class BaseFootprintBroadcaster:

    def __init__(self):
        self.br = tf.TransformBroadcaster()

        # subscriber
        self.odom_sub = rospy.Subscriber('/pepper_robot/odom', Odometry, self.odomCb)

    def odomCb(self, data):
        
        self.br.sendTransform(
            (data.pose.pose.position.x, data.pose.pose.position.y, 0),
            (data.pose.pose.orientation.x, data.pose.pose.orientation.y, data.pose.pose.orientation.z, data.pose.pose.orientation.w),
            rospy.Time.now(),
            "/base_footprint_dum",
            "/odom",
        )

    def run(self):
        while not rospy.is_shutdown():
            rospy.sleep(0.001)

if __name__ == "__main__":
    rospy.init_node("odom_tf_broadcaster")

    odom_broadcaster = BaseFootprintBroadcaster()
    odom_broadcaster.run()
