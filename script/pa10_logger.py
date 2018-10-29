#! /usr/env/bin python2

import rospy
import tf
import csv


class PA10Logger(object):
    def __init__(self):
        rospy.init_node(name='pa10_logger_node')
        self.listener = tf.TransformListener()
        self.loop_rate = rospy.Rate(hz=10)

        self.file = open('pa10_log.csv', 'w')
        self.writer = csv.writer(self.file, lineterminator='\n')

    def getPA10Status(self):
        self.listener.waitForTransform(
            target_frame='world', source_frame='link7', time=rospy.Time(), timeout=rospy.Duration(1.0))
        (trans, rot) = self.listener.lookupTransform(
            target_frame='world', source_frame='link7', time=rospy.Time(secs=0))
        r, p, y = tf.transformations.euler_from_quaternion(rot)

        line = [trans[0], trans[1], trans[2], r, p, y]
        self.writer.writerow(line)

    def fileClose(self):
        self.file.close()


if __name__ == "__main__":
    logger = PA10Logger()

    while not rospy.is_shutdown():
        try:
            logger.getPA10Status()
            logger.loop_rate.sleep()

        except KeyboardInterrupt:
            continue

    logger.fileClose()
