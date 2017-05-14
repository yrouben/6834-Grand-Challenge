from __future__ import division
import math
import numpy as np
import random
from graph import Graph, Edge
from search_classes import SearchNode, Path
from utils import *
from shapely.geometry import Point, LineString

import rospy

import cogrob.msg.MapInfo
import nav_msgs.msg.Path
#from geometry_msgs.msg import PoseStamped, Pose
import cogrog.msg.PathEndPoints

class PRMNode():

    """ Ros node plans path using PRM algorithm on a 2D environment

    """

    def __init__(self, radius, resolution, isLazy):
        self.prm_planner = PRMPathPlanner()
        self.radius = radius
        self.resolution = resolution
        self.isLazy = isLazy

        self.env = None
        self.start = None
        self.goal = None

        rospy.init_node('PRM_Node')
        rospy.Subscriber('map_info', cogrob.msg.MapInfo, self.prepare_environment)

        self.publisher = rospy.Publisher('PRM_Path', nav_msgs.msg.Path)


    def create_environment(polygons, bounds):
        """
        Creates an environment object from a list of polygons representing obstacles.

        Args:
           polygons (A list of Polygons representing obstaces).

        Returns:
            environment (an environment object)
        """
        env = Environment()
        for (i,poly) in enumerate(polygons):
            env.obstacles.append(poly)
            env.obstacles.map[i]
        env.expanded_obstacles = [obs.buffer(.75/2, resolution=2) for obs in env.obstacles]
        env.bounds = bounds
        return env

    def prepare_environment(self, message):
        polygons = None
        bounds = None
        self.env = create_environment(polygons, bound)

    def plan(self):
        return self.prm_planner.path(self.env, self.env.bounds, self.start, self.goal, self.radius, self.resolution, self.isLazy)

    def prepare_output(path):
        output = None
        return None

    def path_callback(self, message)
        if self.env is not None:
            self.start = None
            self.goal = None
            self.publisher.publish(prepare_output(self.plan()))

    def main(self):
        rospy.Subscriber('start_and_end', cogrob.msg.PathEndPoints, self.path_callback)
        rospy.spin()

if __name__ = '__main__':
    prm_planner = PRMNode()
    prm_planner.main()

