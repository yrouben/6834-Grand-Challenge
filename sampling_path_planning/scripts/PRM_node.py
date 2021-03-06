#! /usr/bin/env python

from PRMPlanner import PRMPathPlanner
from shapely.geometry import Polygon, LineString
from environment import Environment

import rospy

from sampling_path_planning.msg import MapInfo
from nav_msgs.msg import Path
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point, PoseStamped
from sampling_path_planning.msg import PathEndPoints
from shapely.geometry import LineString


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
        self.polygon_pub = rospy.Publisher('polygon_vis', MarkerArray)

        self.publisher = rospy.Publisher('PRM_Path', Path)


    def create_environment(self, polygons, bounds=None):
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
            env.obstacles_map[i] = poly
        env.expanded_obstacles = [obs.buffer(.75/2, resolution=2) for obs in env.obstacles]
        env.bounds = bounds
        if env.bounds == None:
            env.calculate_scene_dimensions()
        return env

    def prepare_environment(self, msg):
        polygons = []
        for rospoly in msg.polygons:
            polygon = []
            for point in rospoly.points:
                polygon.append([point.x, point.y])
            if len(polygon) >= 3:
                polygons.append(Polygon(polygon).buffer(0.5))
            elif len(polygon) == 2:
                line = LineString(polygon)
                polygons.append(line.buffer(self.radius, self.resolution))
            #else:
            #    raise NameError("What, is there a polygon of len 2 ??")

        # bounds = [msg.originX, msg.originY, msg.originX + msg.lenX, msg.originY + msg.lenY]

        bounds=None
        self.env = self.create_environment(polygons, bounds)

        #convert to MarkerArray
        marker_array = MarkerArray()
        markers = []
        temp_id = 0
        for polygon in polygons:
            marker = Marker()
            marker.header.frame_id = 'map'
            marker.type = 4
            marker.ns = 'sampling'
            marker.action = 0
            marker.color.a = 1
            marker.color.r = 1
            marker.scale.x = 0.05
            marker.id = temp_id
            temp_id += 1
            points = []
            for point in polygon.exterior.coords:
                new_point = Point()
                new_point.x = point[0]
                new_point.y = point[1]
                new_point.z = 0
                points.append(new_point)
            marker.points = points
            markers.append(marker)

        marker_array.markers = markers
        self.polygon_pub.publish(marker_array);

    def plan(self):
        return self.prm_planner.path(self.env, self.env.bounds, self.start, self.goal, self.radius, self.resolution, self.isLazy)[0]

    def prepare_output(self, path):
        output = Path()
        poses = []
        for point in path:
            new_pose = PoseStamped()
            new_pose.header.frame_id = "map"
            new_pose.pose.position.x = point[0]
            new_pose.pose.position.y = point[1]
            new_pose.pose.orientation.w = 1
            poses.append(new_pose)
        output.poses = poses

        #raise NameError("prepare output about to output")

        return output

    def path_callback(self, message):
        if self.env is not None:
            self.start = message.startpose.position.x, message.startpose.position.y
            goalx, goaly = message.goalpose.position.x, message.goalpose.position.y
            self.goal = Polygon([
                (goalx-.05,goaly-.05),
                (goalx-.05,goaly+.05),
                (goalx+.05,goaly-.05),
                (goalx+.05,goaly+.05)])

            self.publisher.publish(self.prepare_output(self.plan()))
            #raise NameError("publish was called")

    def main(self):
        rospy.Subscriber('start_and_end', PathEndPoints, self.path_callback)
        rospy.spin()

if __name__ == '__main__':
    prm_planner = PRMNode()
    prm_planner.main()

