import glob
import os
import sys
from collections import deque

import math
import pandas as pd
import networkx as nx

try:
    sys.path.append(glob.glob('**/carla/dist/carla-*%d.%d-%s.egg' % (
        sys.version_info.major,
        sys.version_info.minor,
        'win-amd64' if os.name == 'nt' else 'linux-x86_64'))[0])
except IndexError:
    print("Can't find CARLA")
    exit(0)

import carla


class Navigator(object):
    def __init__(self, world):
        print("Getting world...")
        self.world = world
        self.map = self.world.get_map()
        self.adjacency, self.waypoints = self.create_graph_and_matrix()

    def create_graph_and_matrix(self):
        topology = self.map.get_topology()
        adjacency = nx.DiGraph()
        waypoints = pd.DataFrame(columns=['id', 'location', 'rotation'])

        for edge in topology:
            start = edge[0]
            end = edge[1]
            distance = start.transform.location.distance(end.transform.location)
            adjacency.add_weighted_edges_from([(start.id, end.id, distance)])

            if start.id not in waypoints['id'].values:
                waypoints = waypoints.append({'id': start.id,
                                              'transform': start.transform},
                                             ignore_index=True)

            if end.id not in waypoints['id'].values:
                waypoints = waypoints.append({'id': end.id,
                                              'transform': start.transform},
                                             ignore_index=True)
        return adjacency, waypoints

    # Use map topology to create a high level plan for the route
    def find_path(self, start, end):
        route = nx.dijkstra_path(self.adjacency, start, end)

        path = deque([])
        for index in route:
            location = self.waypoints[self.waypoints.id == index]['transform'].values[0].location
            if location not in path:
                path.append(location)
                print(location)
            self.world.debug.draw_string(location, str(len(path)), color=carla.Color(r=0, g=255, b=255), life_time=10.0)

        self.world.debug.draw_string(path[0], 'START', color=carla.Color(r=0, g=255, b=255), life_time=10.0)
        self.world.debug.draw_string(path[len(path)-1], 'END', color=carla.Color(r=0, g=255, b=255), life_time=10.0)

        return path

    def find_closest_waypoint(self, location):
        found = None
        found_distance = math.inf
        for point in self.waypoints.values[3:]:
            distance = point[3].location.distance(location)
            if distance < found_distance:
                found_distance = distance
                found = point
        return found

    def draw_all_waypoints(self):
        adjacency, waypoints = self.create_graph_and_matrix()
        for point in waypoints['transform']:
            location = point.location
            self.world.debug.draw_string(location, 'o', color=carla.Color(r=0, g=255, b=255), life_time=10.0)


if __name__ == '__main__':
    print('Setting up client...')
    client = carla.Client('localhost', 2000)
    client.set_timeout(2.0)

    ex2 = [carla.Vector3D(42.5959, -4.3443, 1.8431), carla.Vector3D(-30, 167, 1.8431)]
    navigator = Navigator(client.get_world())
    print("Finding waypoints...")
    start = navigator.find_closest_waypoint(ex2[0])
    print("Starting graph point: {}".format(ex2[0]))
    end = navigator.find_closest_waypoint(ex2[1])
    print("Ending graph point: {}".format(ex2[1]))
    path = navigator.find_path(start[0], end[0])
