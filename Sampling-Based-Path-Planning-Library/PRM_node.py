from __future__ import division
import math
import numpy as np
import random
from graph import Graph, Edge
from search_classes import SearchNode, Path
from utils import *
from shapely.geometry import Point, LineString

class PRMNode():

    """ Ros node plans path using PRM algorithm on a 2D environment

    """

    def __init__(self, radius, resolution, isLazy):
        self.prm_planner = PRMPathPlanner()
        self.radius = radius
        self.resolution = resolution
        self.isLazy = isLazy

    def create_environment(polygons):
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
        env.calculate_scene_dimensions()
        return env

    def plan(self, env, start, goal):
        return self.prm_planner.path(env, env.bounds, start, goal, self.radius, self.resolution, self.isLazy)

    def prepare_input(message):
        polygons = None
        start = None
        goal = None
        return polygons, start, goal

    def prepare_output(path, V, E):
        pass

    def main():
        #listen for topic -> get message
        #polygons, start, goal = prepare_input(message)
        #env = create_environment(polygons)
        #path, V, E = self.plan(env, start, goal)
        #prepare_output


