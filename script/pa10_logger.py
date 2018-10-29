#! /usr/env/bin python2

import rospy
import tf
import csv
import time


class PA10Logger(object):
    def __init__(self):
        rospy.init_node(name='pa10_logger_node')
        self.listener = tf.TransformListener()
        self.loop_rate = rospy.Rate(hz=10)

        self.file = open('pa10_log.csv', 'w')
        self.writer = csv.writer(self.file, lineterminator='\n')
        self.writer.writerow(["#time", "x", "y", "z", "roll", "pitch", "yaw"])
        self.time_from_start = time.time()

    def getPA10Status(self):
        self.listener.waitForTransform(
            target_frame='world', source_frame='link7', time=rospy.Time(), timeout=rospy.Duration(1.0))
        (trans, rot) = self.listener.lookupTransform(
            target_frame='world', source_frame='link7', time=rospy.Time(secs=0))
        r, p, y = tf.transformations.euler_from_quaternion(rot)

        now = time.time() - self.time_from_start
        line = [round(now, 4), round(trans[0], 6), round(trans[1], 6), round(
            trans[2], 6), round(r, 6), round(p, 6), round(y, 6)]
        self.writer.writerow(line)
        print line

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
