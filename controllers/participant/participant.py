# Copyright 1996-2023 Cyberbotics Ltd.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""
Demonstrates the gait manager (inverse kinematics + simple ellipsoid path).
"""

from controller import Robot
import sys
sys.path.append('..')
# Eve's locate_opponent() is implemented in this module:
from utils.image_processing import ImageProcessing as IP
from utils.fall_detection import FallDetection
from utils.accelerometer import Accelerometer
from utils.gait import Gait_manager
from utils.camera import Camera


class Fatima (Robot):
    SMALLEST_TURNING_RADIUS = 0.1
    SAFE_ZONE = 0.75

    def __init__(self):
        Robot.__init__(self)
        self.time_step = int(self.getBasicTimeStep())

        self.camera = Camera(self)
        self.gps = self.getDevice("gps")
        self.gps.enable(self.time_step)
        self.accelerometer = Accelerometer(
            self.getDevice('accelerometer'), self.time_step)
        self.fall_detector = FallDetection(self.time_step, self)
        self.gait_manager = Gait_manager(self, self.time_step)
        self.heading_angle = 3.14 / 2
        self.k = 0

    def run(self):
        while self.step(self.time_step) != -1:
            # We need to update the internal theta value of the gait manager at every step:
            t = self.getTime()
            self.gait_manager.update_theta()
            if 0.3 < t < 2:
                self.start_sequence()
            elif t > 2:
                self.fall_detector.check()
                self.walk()

    def start_sequence(self):
        """At the beginning of the match, the robot walks forwards to move away from the edges."""
        self.gait_manager.command_to_motors(heading_angle=0)

    def walk(self):
        """Walk towards the opponent like a homing missile."""
        normalized_x = self._get_normalized_opponent_x()
        # We set the desired radius such that the robot walks towards the opponent.
        # If the opponent is close to the middle, the robot walks straight (turning radius very high).
        desired_radius = self.SMALLEST_TURNING_RADIUS / \
            normalized_x if abs(normalized_x) > 1e-3 else 1e3
        [x, y, _] = self.gps.getValues()
        if (abs(x) > self.SAFE_ZONE or abs(y) > self.SAFE_ZONE) and self.k == 0:
            # if the robot is close to the edge, it switches dodging direction
            self.heading_angle = - self.heading_angle
            # we disable the safe zone check for a second to avoid the robot to get stuck in a loop
            self.k = 1000 / self.time_step
        elif self.k > 0:
            self.k -= 1
        self.gait_manager.command_to_motors(
            desired_radius=desired_radius, heading_angle=self.heading_angle)

    def _get_normalized_opponent_x(self):
        """Locate the opponent in the image and return its horizontal position in the range [-1, 1]."""
        img = self.camera.get_image()
        _, _, horizontal_coordinate = IP.locate_opponent(img)
        if horizontal_coordinate is None:
            return 0
        return horizontal_coordinate * 2 / img.shape[1] - 1


# create the Robot instance and run main loop
wrestler = Fatima()
wrestler.run()
