#!/usr/bin/env python

import rospy
import tf


rospy.init_node('project2_tf_output', anonymous=True)

listener = tf.TransformListener()



pos = None
ori = None

while not rospy.is_shutdown():
    try:
        (pos, ori) = listener.lookupTransform('/map', '/base_footprint', rospy.Time(0))
        rospy.loginfo('position {}'.format(pos))
        rospy.loginfo('orientation {}'.format(ori))
    except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
        rospy.logwarn(e)
    rospy.sleep(0.5)