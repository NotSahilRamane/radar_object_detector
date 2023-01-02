#!/usr/bin/env python3

from __future__ import absolute_import
import rospy
from sensor_msgs.msg import PointCloud2
from fsd_common_msgs.msg import ConeOdom
from nav_msgs.msg import Odometry
from fsd_common_msgs.msg import Cone
from ros_numpy import point_cloud2
import numpy
import math

import dbscan


class RadarObjectDetector(object):
    def __init__(self):

        self.conedetections = ConeOdom()
        self.POINTCLOUD_RECEIVED = 0
        self.loadParameters()
        self.pointcloud = PointCloud2()
        self.x_data, self.y_data = [], []

    def publishToTopics(self):
        rospy.loginfo("Published to topics")
        self.coneDetectionsPublisher = rospy.Publisher(
            self.cone_detections_topic_name, ConeOdom, queue_size=1)

    def run(self, pointcloud):
        cones = self.find_cones(pointcloud)

        # self.conedetections.cone_detections = cones
        # self.conedetections.header.stamp = rospy.get_rostime()
        # self.conedetections.header.frame_id = "fsds/FSCar"

        # self.coneDetectionsPublisher.publish(self.conedetections)

        rospy.loginfo(cones)

    def getNodeRate(self):
        return self.node_rate_

    def loadParameters(self):
        rospy.loginfo("loading handle parameters")

        self.cone_detections_topic_name = rospy.get_param(
            "lidar_cone_detector/cone_detections_topic_name", "/perception/lidar/cone_detections")

        self.node_rate_ = rospy.get_param("node_rate", 1)

        self.lidar_topicname = rospy.get_param(
            "lidar_cone_detector/lidar_topicname", "/carla/ego_vehicle/radar_front")


    def subscribeToTopics(self):
        rospy.loginfo("Subscribed to topics")
        rospy.Subscriber(self.lidar_topicname, PointCloud2,
                         self.run, queue_size=1)

    def pointGroupToCone(self, group):
        average_x = 0
        average_y = 0
        average_z = 0
        average_v = 0
        for point in group:
            average_x += point[0]
            average_y += point[1]
            average_z += point[2]
            average_v += point[3]
        average_x = average_x / len(group)
        average_y = average_y / len(group)
        average_z = average_z / len(group)
        average_v = average_v / len(group)

        return {'x': average_x, 'y': average_y,'z': average_z, 'v': average_v}

    def find_cones(self, pointcloud):
        points_4d = []
        cones = []

        points = point_cloud2.pointcloud2_to_array(pointcloud, squeeze=True)

                
        for i in range(len(points)):
            points_4d.append((points[i][0], points[i][1], points[i][2], points[i][4]))
        clusters = dbscan.get_clusters(points_4d)
        for i in list(clusters.keys()):
            cone_3d_pos = Cone()
            cluster = numpy.asarray(clusters[i])
            avg_cone = self.pointGroupToCone(cluster)

            x = avg_cone['x']
            y = avg_cone['y']
            z = avg_cone['z']
            v = avg_cone['v']
            print(x,y,z,v)

# ADD MESSAGE AND PUBLISHER HERE

        return "done"