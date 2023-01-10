# Humanoid Robot Wrestling Controller Example

[![webots.cloud - Competition](https://img.shields.io/badge/webots.cloud-Competition-007ACC)][1]

## Fatima controller

Demonstrates the gait manager (inverse kinematics + simple ellipsoid path).

Beats [Eve](https://github.com/cyberbotics/wrestling-charlie) by homing on her and pushing her down.

Here is the [participant.py](./controllers/participant/participant.py) file:

``` Python
from controller import Robot, Motion
import sys
sys.path.append('..')
from utils.routines import Fall_detection
from utils.motion import Current_motion_manager
from utils.sensors import Accelerometer
from utils.gait import Gait_manager
import utils.image # Eve's locate_opponent() is implemented in this module

try:
    import numpy as np
    np.set_printoptions(suppress=True)
except ImportError:
    sys.exit("Warning: 'numpy' module not found. Please check the Python modules installation instructions " +
             "at 'https://www.cyberbotics.com/doc/guide/using-python'.")
try:
    import cv2
except ImportError:
    sys.exit("Warning: 'cv2' module not found. Please check the Python modules installation instructions " +
             "at 'https://www.cyberbotics.com/doc/guide/using-python'.")


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
        largest_contour, vertical, horizontal = utils.image.locate_opponent(img)
        output = img.copy()
        if largest_contour is not None:
            cv2.drawContours(output, [largest_contour], 0, (255, 255, 0), 1)
            output = cv2.circle(output, (horizontal, vertical), radius=2,
                                color=(0, 0, 255), thickness=-1)
        # utils.image.send_image_to_robot_window(self, output)
        if horizontal is None:
            return 0
        return horizontal * 2/img.shape[1] - 1

# create the Robot instance and run main loop
wrestler = Fatima()
wrestler.run()
```

<!-- [Grace](https://github.com/cyberbotics/wrestling-grace) is a more advanced robot controller able to win against Fatima. -->

[1]: https://webots.cloud/run?version=R2022b&url=https%3A%2F%2Fgithub.com%2Fcyberbotics%2Fwrestling%2Fblob%2Fmain%2Fworlds%2Fwrestling.wbt&type=competition "Leaderboard"
