# Copyright 1996-2022 Cyberbotics Ltd.
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

from controller import Robot, Motion

import sys
sys.path.append('..')
from utils.routines import Fall_detection
from utils.motion import Current_motion_manager
from utils.sensors import Accelerometer
from utils.gait import Gait_manager
# Eve's locate_opponent() is implemented in this module:
import utils.image


class Fatima (Robot):
    def __init__(self):
        Robot.__init__(self)
        self.time_step = int(self.getBasicTimeStep())

        self.camera = self.getDevice("CameraTop")
        self.camera.enable(self.time_step)
        self.accelerometer = Accelerometer(self.getDevice('accelerometer'), self.time_step)
        self.fall_detector = Fall_detection(self.time_step, self)
        self.gait_manager = Gait_manager(self, self.time_step)
        self.current_motion = Current_motion_manager()
        self.current_motion.set(Motion('../motions/Stand.motion'))

    def run(self):
        i = 0
        while self.step(self.time_step) != -1:
            if self.current_motion.is_over():
                self.fall_detector.check()
                self.gait_manager.update_theta()
                self.walk()

    def walk(self):
        """Walk towards the opponent like a homing missile."""
        x_pos_normalized = self._get_normalized_opponent_x()
        # We set the desired radius such that the robot walks towards the opponent.
        desired_radius = 0.1 / x_pos_normalized if abs(x_pos_normalized) > 1e-3 else 1e3
        self.gait_manager.command_to_motors(desired_radius)

    def _get_normalized_opponent_x(self):
        """Locate the opponent in the image and return its horizontal position in the range [-1, 1]."""
        img = utils.image.get_cv_image_from_camera(self.camera)
        _, _, horizontal = utils.image.locate_opponent(img)
        if horizontal is None:
            return 0
        return horizontal * 2/img.shape[1] - 1

# create the Robot instance and run main loop
wrestler = Fatima()
wrestler.run()
