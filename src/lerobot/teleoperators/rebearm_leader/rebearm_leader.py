#!/usr/bin/env python

# Copyright 2025 The HuggingFace Inc. team. All rights reserved.
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

import logging
import time

from lerobot.errors import DeviceAlreadyConnectedError, DeviceNotConnectedError
from lerobot.motors import Motor, MotorCalibration, MotorNormMode
from lerobot.motors.lewansol import (
    LewansoulMotorsBus,
)

from ..teleoperator import Teleoperator
from .config_rebearm_leader import REBEARMLeaderConfig

logger = logging.getLogger(__name__)


class REBEARMLeader(Teleoperator):
    """
    Rebearm Leader Arm designed by TheRobotStudio and Hugging Face.
    """

    config_class = REBEARMLeaderConfig
    name = "rebearm_leader"

    def __init__(self, config: REBEARMLeaderConfig):
        super().__init__(config)
        self.config = config
        self.bus = LewansoulMotorsBus(
            port=self.config.port,
            motors={
                "shoulder_pan": Motor(1, "lx16a", MotorNormMode.RANGE_M100_100),
                "shoulder_lift": Motor(2, "lx16a", MotorNormMode.RANGE_M100_100),
                "elbow_flex": Motor(3, "lx16a", MotorNormMode.RANGE_M100_100),
                "wrist_flex": Motor(4, "lx16a", MotorNormMode.RANGE_M100_100),
                "wrist_roll": Motor(5, "lx16a", MotorNormMode.RANGE_M100_100),
                "gripper": Motor(6, "lx16a", MotorNormMode.RANGE_0_100),
            },
            calibration=self.calibration,
        )
        
    @property
    def action_features(self) -> dict[str, type]:
        return {f"{motor}.pos": float for motor in self.bus.motors}

    @property
    def feedback_features(self) -> dict[str, type]:
        return {}

    @property
    def is_connected(self) -> bool:
        return self.bus.is_connected

    def connect(self, calibrate: bool = True) -> None:
        if self.is_connected:
            raise DeviceAlreadyConnectedError(f"{self} already connected")

        self.bus.connect()
        if not self.is_calibrated and calibrate:
            self.calibrate()

        self.configure()
        logger.info(f"{self} connected.")

    @property
    def is_calibrated(self) -> bool:
        return self.bus.is_calibrated

    def calibrate(self) -> None:
        logger.info(f"\nRunning calibration of {self}")
        print(
            "Just torque off, don't run calibration "
        )
        self.bus.disable_torque()

    def configure(self) -> None:
        print(
            "leader->configure"
        )
        self.bus.disable_torque(motors=self.bus.motors)

    def setup_motors(self) -> None:
        for motor in reversed(self.bus.motors):
            input(f"Connect the controller board to the '{motor}' motor only and press enter.")
            self.bus.setup_motor(motor, initial_baudrate = 115200, initial_id=6)
            print(f"'{motor}' motor id set to {self.bus.motors[motor].id}")

    def get_action(self) -> dict[str, int]:
        print("leader->getaction")
        action = {}
        
        for motor in self.bus.motors:
            action_each = self.bus.send("Present_Position", motor=motor, value=None)
            #print(f'Motor {motor} - action_each: {action_each}')
            if action_each is not None:
                action.update(action_each) 
            else:
                print(f"Warning: Got None response for motor {motor}")
        
        # Add .pos suffix to all keys
        action = {f"{motor}.pos": val for motor, val in action.items()}
        print('final action with .pos suffix', action)
        
        return action

    def send_feedback(self, feedback: dict[str, float]) -> None:
        # TODO(rcadene, aliberts): Implement force feedback
        raise NotImplementedError

    def disconnect(self) -> None:
        if not self.is_connected:
            DeviceNotConnectedError(f"{self} is not connected.")

        self.bus.disconnect()
        logger.info(f"{self} disconnected.")
