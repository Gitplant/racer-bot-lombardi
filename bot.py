from typing import Tuple

from pygame import Vector2

from ...bot import Bot
from ...linear_math import Transform
from racer.constants import framerate

import numpy as np
from .track_utils import change_track

class Lombardi(Bot):

    def __init__(self, track):
        super().__init__(track)
        self.next_way_point = None
        self.total_distance = 0
        self.dt = 1 / framerate
        self.alpha = 100
        self.deceleration_distance = 0
        self.track.lines = change_track(self.track, max_dist_ratio=0.9)

    @property
    def name(self):
        return "Lombardi"

    @property
    def contributor(self):
        return "Gitplant"

    def compute_commands(self, next_waypoint: int, position: Transform, velocity: Vector2) -> Tuple:

        absolute_target = self.track.lines[next_waypoint]
        
        # calculate the target in the frame of the robot
        relative_target = position.inverse() * absolute_target

        # calculate the angle to the target
        distance, angle = relative_target.as_polar()  # In degrees! Who would have thought?
        
        # Angle between velocity and vector to target
        alpha = self.compute_alpha(velocity, absolute_target, position)

        # calculate the throttle
        way_point_velocity = 200

        if next_waypoint and next_waypoint != self.next_way_point:
            # print(f' --- Way point {next_waypoint} --- ')
            self.next_way_point = next_waypoint
            self.total_distance = distance

            x2 = self.total_distance - self.track.track_width
            v0 = velocity.length()
            self.deceleration_distance = self.compute_deceleration_distance(x2, v0, way_point_velocity)
            # print(f'deceleration_distance = {self.deceleration_distance}, x2 = {x2}, v0 = {v0}, way_point_velocity = {way_point_velocity}')

        # print(f'alpha = {alpha}')
        if (distance < self.deceleration_distance and velocity.length() > way_point_velocity) \
        or ((alpha > 10 or alpha < -10) and (velocity.length() > 100)):
            throttle = -1
        else:
            throttle = 1
        if np.abs(alpha) > 90:  # Car is moving backwards
            throttle = 1

        if angle > 0:
            return throttle, 1
        else:
            return throttle, -1
        
    def compute_alpha(self, velocity, target, position):
        # Angle between velocity and vector to target

        angle = velocity.angle_to(target - position.p)

        # Range between -180 and 180:
        while angle > 180 or angle < -180:
            if angle > 180:
                angle -= 360
            elif angle < -180:
                angle += 360

        return angle
        
    def compute_deceleration_distance(self, x2, v0, v2):
        # Distance between the car and the target

        xd = x2 - (v0**2 + 2 * self.alpha * x2 - v2**2) / (4 * self.alpha)
        deceleration_distance = x2 - xd + self.track.track_width

        return deceleration_distance
