#! /usr/bin/env python

from PRMPlanner import PRMPathPlanner
from shapely.geometry import Polygon

import rospy

from sampling_path_planning.msg import MapInfo
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from sampling_path_planning.msg import PathEndPoints

class PRMNode():

    """ Ros node plans path using PRM algorithm on a 2D environment

    """

    def __init__(self, radius=.01, resolution=3, isLazy=True):
        self.prm_planner = PRMPathPlanner()
        self.radius = radius
        self.resolution = resolution
        self.isLazy = isLazy

        self.env = None
        self.start = None
        self.goal = None

        rospy.init_node('PRM_Node')
        rospy.Subscriber('map_info', MapInfo, self.prepare_environment)

        self.publisher = rospy.Publisher('PRM_Path', Path)


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

    def prepare_environment(self, msg):
        for rospoly in msg.polygons:
            polygon = []
            for point in rospoly.points:
                polygon.append([point.x, point.y])
            polygons.append(Polygon(polygon))
        bounds = [msg.originX, msg.originY, msg.originX + msg.lenX, msg.originY + msg.lenY]
        self.env = create_environment(polygons, bounds)

    def plan(self):
        return self.prm_planner.path(self.env, self.env.bounds, self.start, self.goal, self.radius, self.resolution, self.isLazy)

    def prepare_output(path):
        output = Path()
        poses = []
        for point in path:
            new_pose = PoseStamped()
            new_pose.pose.position.x = point[0]
            new_pose.pose.position.y = point[1]
            poses.append(new_pose)
        output.poses = poses
        return output

    def path_callback(self, message):
        if self.env is not None:
            self.start = message.startpose.position.x, message.startpose.position.y
            goalx, goaly = message.startpose.position.x, message.startpose.position.y
            self.goal = Polygon([
                (goalx-.01,goaly-.01),
                (goalx-.01,goaly+.01),
                (goalx+.01,goaly-.01),
                (goalx+.01,goaly+.01)])

            self.publisher.publish(prepare_output(self.plan()))

    def main(self):
        rospy.Subscriber('start_and_end', PathEndPoints, self.path_callback)
        rospy.spin()

if __name__ == '__main__':
    prm_planner = PRMNode()
    prm_planner.main()
