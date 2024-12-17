#! /usr/bin/env python3

'''
Created: hello-robot company
Modified: Steven Alexander Silva Mendoza, silvas1@cardiff.ac.uk
'''

from gazebo_msgs.srv import GetModelState, GetModelStateRequest, GetWorldProperties
from nav_msgs.msg import Odometry
from std_msgs.msg import Header
from std_srvs.srv import Empty
import rospy
import time
import tf

rospy.init_node('ground_truth_odometry_publisher')
odom_pub=rospy.Publisher('ground_truth', Odometry, queue_size=10)

rospy.wait_for_service('/gazebo/get_model_state')
get_model_srv = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
get_world_properties = rospy.ServiceProxy('/gazebo/get_world_properties', GetWorldProperties)
unpause_physics = rospy.ServiceProxy('/gazebo/unpause_physics', Empty)

odom=Odometry()
header = Header()
header.frame_id='/ground_truth'
model = GetModelStateRequest()
model.model_name='robot'
models = []
r = rospy.Rate(20)

pause_timeout = time.time() + 4.0
while time.time() < pause_timeout:
    rospy.logwarn("Waiting %.2f seconds to unpause physics", pause_timeout - time.time())
    time.sleep(1.0)
unpause_physics()

br = tf.TransformBroadcaster()

while not rospy.is_shutdown():
    if model.model_name not in models:
        models = get_world_properties().model_names
        rospy.logwarn("Waiting for %s to spawn to publish ground truth odometry", model.model_name)
    else:
        result = get_model_srv(model)
        odom.pose.pose = result.pose
        odom.twist.twist = result.twist
        header.stamp = rospy.Time.now()
        odom.header = header
        odom_pub.publish(odom)

        br.sendTransform(
            (result.pose.position.x, result.pose.position.y, result.pose.position.z),
            (result.pose.orientation.x, result.pose.orientation.y, result.pose.orientation.z, result.pose.orientation.w),
            rospy.Time.now(),
            "/base_link",
            "/odom",
        )

    r.sleep()