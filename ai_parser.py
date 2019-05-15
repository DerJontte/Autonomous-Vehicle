#!/usr/bin/env python

import glob
import os
import sys
import math
import weakref

try:
    sys.path.append(glob.glob('**/carla/dist/carla-*%d.%d-%s.egg' % (
        sys.version_info.major,
        sys.version_info.minor,
        'win-amd64' if os.name == 'nt' else 'linux-x86_64'))[0])
except IndexError:
    print("Can't find CARLA")
    exit(0)

import carla


# Monitor is responsible for reading the data from the sensors and telling it to the knowledge
# TODO: Implement other sensors (lidar and depth sensors mainly)
class Monitor(object):
    def __init__(self, knowledge, vehicle):
        self.vehicle = vehicle
        self.knowledge = knowledge
        weak_self = weakref.ref(self)

        # Initialize all defined memory variables
        self.update(0)

        world = self.vehicle.get_world()
        self.knowledge.update_data('world', world)

        # Set up lane detector
        lane_sensor = world.get_blueprint_library().find('sensor.other.lane_invasion')
        self.lane_detector = world.spawn_actor(lane_sensor, carla.Transform(), attach_to=self.vehicle)
        self.lane_detector.listen(lambda event: self._on_invasion(weak_self, event))

        # Set up collision sensor
        collision_sensor = world.get_blueprint_library().find('sensor.other.collision')
        self.collision_detector = world.spawn_actor(collision_sensor, carla.Transform(), attach_to=self.vehicle)
        self.collision_detector.listen(lambda event: self._on_collision(weak_self, event))

        obstacle_sensor = world.get_blueprint_library().find('sensor.other.obstacle')
        self.obstacle_sensor = world.spawn_actor(obstacle_sensor, carla.Transform(carla.Location(x=1.5, y=1, z=2)))
        self.obstacle_sensor.listen(lambda event: self._on_obstacle(weak_self, event))

        # Set up a camera to enable rendering a first person perspective view, and to be able to do object recognition
        # if the need arises.
        # camera_rgb_blueprint = world.get_blueprint_library().find('sensor.camera.rgb')
        # camera_rgb_blueprint.set_attribute('image_size_x', '960')
        # camera_rgb_blueprint.set_attribute('image_size_y', '540')
        # camera_rgb_blueprint.set_attribute('fov', '110')
        # camera_rgb_blueprint.set_attribute('sensor_tick', '0.1')
        # camera_transform = carla.Transform(carla.Location(x=1.5, z=2.4))
        # self.camera_rgb = world.spawn_actor(camera_rgb_blueprint, camera_transform, attach_to=self.vehicle)
        # self.camera_rgb.listen(lambda image: self.knowledge.update_data('rgb_camera', image))

    # Function that is called at time intervals to update ai-state
    def update(self, time_elapsed):
        # Update the position of vehicle into knowledge
        location = self.vehicle.get_transform().location
        destination = self.knowledge.get_current_destination()
        distance = location.distance(destination)

        self.knowledge.update_data('location', self.vehicle.get_transform().location)
        self.knowledge.update_data('rotation', self.vehicle.get_transform().rotation)
        self.knowledge.update_data('velocity', self.vehicle.get_velocity()) # This is a vector that will be processed in the analyzer

        self.knowledge.update_data('speed_limit', self.vehicle.get_speed_limit())

        # at_lights is true iff the vehicle is at a traffic light that is red or yellow
        self.knowledge.update_data('at_lights', self.vehicle.is_at_traffic_light() and
                                   str(self.vehicle.get_traffic_light_state()) == 'Red' or
                                   str(self.vehicle.get_traffic_light_state()) == 'Yellow')

    @staticmethod
    def _on_invasion(weak_self, event):
        self = weak_self()
        self.knowledge.update_data('lane_invasion', True)
        if not self:
            return

    @staticmethod
    def _on_collision(weak_self, event):
        self = weak_self()
        if not self:
            return

    @staticmethod
    def _on_obstacle(weaak_self, event):
        self = weaak_self()
        print("Obstacle:", event.other_actor, event.distance)

# Analyser is responsible for parsing all the data that the knowledge has received from Monitor and turning it into something usable
# TODO: During the update step parse the data inside knowledge into information that could be used by planner to plan the route
class Analyser(object):
    def __init__(self, knowledge):
        self.knowledge = knowledge

    # Function that is called at time intervals to update ai-state
    def update(self, time_elapsed):
        heading_diff = self.calculate_heading_diff()

        # self.knowledge.update_data('heading', heading)
        self.knowledge.update_data('heading_diff', heading_diff)
        self.knowledge.update_data('speed', self.get_speed())
        return

    def get_speed(self):
        velocity = self.knowledge.retrieve_data('velocity')
        return math.sqrt(velocity.x ** 2.0 + velocity.y ** 2.0 + velocity.z ** 2.0) * 3.6

    def calculate_heading_diff(self):
        current_angle = math.radians(self.knowledge.get_rotation())
        target_vector = self.knowledge.get_current_destination() - self.knowledge.get_location()
        target_angle = math.atan2(target_vector.y, target_vector.x)

        diff = target_angle - current_angle
        if diff > math.pi: diff = diff - (2 * math.pi)
        if diff < - math.pi: diff = diff + (2 * math.pi)

        return diff
