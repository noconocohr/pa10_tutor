#! /usr/env/bin python2

import rospy
import tf
import csv


class PA10Logger(object):
    def __init__(self):
        # rospy.init_node(name='pa10_logger_node')
        self.file = open('pa10_log.csv', 'w')
        # self.listener = tf.TransformListener()


if __name__ == "__main__":
    pa10 = PA10Logger()
