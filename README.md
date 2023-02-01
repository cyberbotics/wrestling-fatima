# Humanoid Robot Wrestling Controller Example

[![webots.cloud - Competition](https://img.shields.io/badge/webots.cloud-Competition-007ACC)][1]

## Fatima controller

Demonstrates the gait manager (inverse kinematics + simple ellipsoid path).

This controller needs Numpy, OpenCV, Scipy and AHRS therefore the [Dockerfile](controllers/Dockerfile#L1-L12) needs to be updated:

```Dockerfile
FROM cyberbotics/webots.cloud:R2023a-ubuntu20.04-numpy

# Additional dependencies
RUN apt-get update && apt-get install -y \
    python3-pip \
    && rm -rf /var/lib/apt/lists/*
RUN pip3 install --upgrade pip && \
    pip3 install --no-cache-dir \
    opencv-python \
    scipy \
    ahrs
```

Beats [Eve](https://github.com/cyberbotics/wrestling-eve) by having a higher coverage.

Here is the [participant.py](./controllers/participant/participant.py) file:

``` Python
from controller import Robot
import sys
sys.path.append('..')
# Eve's locate_opponent() is implemented in this module:
from utils.image_processing import ImageProcessing as IP
from utils.fall_detection import FallDetection
from utils.gait_manager import GaitManager
from utils.camera import Camera


class Fatima (Robot):
    SMALLEST_TURNING_RADIUS = 0.1
    SAFE_ZONE = 0.75
    TIME_BEFORE_DIRECTION_CHANGE = 200  # 8000 ms / 40 ms

    def __init__(self):
        Robot.__init__(self)
        self.time_step = int(self.getBasicTimeStep())

        self.camera = Camera(self)
        self.fall_detector = FallDetection(self.time_step, self)
        self.gait_manager = GaitManager(self, self.time_step)
        self.heading_angle = 3.14 / 2
        # Time before changing direction to stop the robot from falling off the ring
        self.counter = 0

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
        """Dodge the opponent robot by taking side steps."""
        normalized_x = self._get_normalized_opponent_x()
        # We set the desired radius such that the robot walks towards the opponent.
        # If the opponent is close to the middle, the robot walks straight.
        desired_radius = (self.SMALLEST_TURNING_RADIUS / normalized_x) if abs(normalized_x) > 1e-3 else None
        # TODO: position estimation so that if the robot is close to the edge, it switches dodging direction
        if self.counter > self.TIME_BEFORE_DIRECTION_CHANGE:
            self.heading_angle = - self.heading_angle
            self.counter = 0
        self.counter += 1
        self.gait_manager.command_to_motors(desired_radius=desired_radius, heading_angle=self.heading_angle)

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
```

<!-- [Grace](https://github.com/cyberbotics/wrestling-grace) is a more advanced robot controller able to win against Fatima. -->

[1]: https://webots.cloud/run?version=R2022b&url=https%3A%2F%2Fgithub.com%2Fcyberbotics%2Fwrestling%2Fblob%2Fmain%2Fworlds%2Fwrestling.wbt&type=competition "Leaderboard"
