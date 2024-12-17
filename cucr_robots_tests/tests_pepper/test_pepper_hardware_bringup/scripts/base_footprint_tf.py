#!/usr/bin/env python
import rospy
import tf


if __name__ == "__main__":
    rospy.init_node("base_footprint_dum_tf_broadcaster")

    listener = tf.TransformListener()

    br = tf.TransformBroadcaster()

    rate = rospy.Rate(100.0)
    while not rospy.is_shutdown():

        # lidar
        try:
            (trans, rot) = listener.lookupTransform(
                "/base_link", "/base_footprint", rospy.Time(0)
            )
        except (
            tf.LookupException,
            tf.ConnectivityException,
            tf.ExtrapolationException,
        ):
            continue

        br.sendTransform(
            (-trans[0], -trans[1], -trans[2]),
            (rot[0], rot[1], rot[2], rot[3]),
            rospy.Time.now(),
            "/base_link",
            "/base_footprint_dum",
        )

        rate.sleep()
