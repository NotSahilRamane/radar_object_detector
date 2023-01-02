#!/usr/bin/env python3

from __future__ import absolute_import
import rospy
from radar_object_detector import RadarObjectDetector

def main():
    rospy.init_node('radar_object_detector')
    detector = RadarObjectDetector()
    detector.subscribeToTopics()
    detector.publishToTopics()
    rospy.spin()


if __name__ == u'__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass