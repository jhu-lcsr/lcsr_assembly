#!/usr/bin/env python

import rospy
import tf

def main():
    rospy.init_node('tf_scaler')

    scale = rospy.get_param('~scale', 50.0)
    parent_frame_id = rospy.get_param('~parent_frame_id', '/world')
    unscaled_frame_id = rospy.get_param('~unscaled_frame_id', '/unscaled')
    scaled_frame_id = rospy.get_param('~scaled_frame_id', '/scaled')
    listener = tf.TransformListener()
    broadcaster = tf.TransformBroadcaster()

    while not rospy.is_shutdown():
        try:
            listener.waitForTransform(parent_frame_id, unscaled_frame_id, rospy.Time(0), rospy.Duration(10.0))

            now = rospy.Time.now()

            listener.waitForTransform(parent_frame_id, unscaled_frame_id, now, rospy.Duration(10.0))
            (trans,rot) = listener.lookupTransform(parent_frame_id, unscaled_frame_id, now)
        except (tf.Exception, tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue

        broadcaster.sendTransform(
            [scale*v for v in trans],
            rot,
            now,
            scaled_frame_id,
            parent_frame_id)

    return 0

if __name__ == '__main__':
    main()
