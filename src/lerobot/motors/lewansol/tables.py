# Copyright 2024 The HuggingFace Inc. team. All rights reserved.
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

# TODO(Steven): Consider doing the following:
# from enum import Enum
# class MyControlTableKey(Enum):
#   ID = "ID"
#   GOAL_SPEED = "Goal_Speed"
#   ...
#
# MY_CONTROL_TABLE ={
#   MyControlTableKey.ID.value: (5,1)
#   MyControlTableKey.GOAL_SPEED.value: (46, 2)
#   ...
# }
# This allows me do to:
# bus.write(MyControlTableKey.GOAL_SPEED, ...)
# Instead of:
# bus.write("Goal_Speed", ...)
# This is important for two reasons:
# 1. The linter will tell me if I'm trying to use an invalid key, instead of me realizing when I get the RunTimeError
# 2. We can change the value of the MyControlTableKey enums without impacting the client code


# {data_name: (address, size_byte)}
# https://emanual.robotis.com/docs/en/dxl/x/{MODEL}/#control-table
CONTROL_TABLE = {
    "Model_Number": (0, 1),
    "Model_Information": (0, 1),
    "Firmware_Version": (0, 1),
    "ID": (0, 1),
    "Baud_Rate": (0, 1),
    "Return_Delay_Time": (0, 1),
    "Drive_Mode": (10, 1),
    "Operating_Mode": (11, 1),
    "Secondary_ID": (12, 1),
    "Protocol_Type": (13, 1),
    "Homing_Offset": (20, 4),
    "Moving_Threshold": (24, 4),
    "Temperature_Limit": (31, 1),
    "Max_Voltage_Limit": (32, 2),
    "Min_Voltage_Limit": (34, 2),
    "PWM_Limit": (36, 2),
    "Current_Limit": (38, 2),
    "Acceleration_Limit": (40, 4),
    "Velocity_Limit": (44, 4),
    "Max_Position_Limit": (48, 4),
    "Min_Position_Limit": (52, 4),
    "Shutdown": (63, 1),
    "Torque_Enable": (64, 1),
    "LED": (65, 1),
    "Status_Return_Level": (68, 1),
    "Registered_Instruction": (69, 1),
    "Hardware_Error_Status": (70, 1),
    "Velocity_I_Gain": (76, 2),
    "Velocity_P_Gain": (78, 2),
    "Position_D_Gain": (80, 2),
    "Position_I_Gain": (82, 2),
    "Position_P_Gain": (84, 2),
    "Feedforward_2nd_Gain": (88, 2),
    "Feedforward_1st_Gain": (90, 2),
    "Bus_Watchdog": (98, 1),
    "Goal_PWM": (100, 2),
    "Goal_Current": (102, 2),
    "Goal_Velocity": (104, 4),
    "Profile_Acceleration": (108, 4),
    "Profile_Velocity": (112, 4),
    "Goal_Position": (116, 4),
    "Realtime_Tick": (120, 2),
    "Moving": (122, 1),
    "Moving_Status": (123, 1),
    "Present_PWM": (124, 2),
    "Present_Current": (126, 2),
    "Present_Velocity": (128, 4),
    "Present_Position": (28, 1),
    "Velocity_Trajectory": (136, 4),
    "Position_Trajectory": (140, 4),
    "Present_Input_Voltage": (144, 2),
    "Present_Temperature": (146, 1),
}

# https://emanual.robotis.com/docs/en/dxl/x/{MODEL}/#baud-rate8
BAUDRATE_TABLE = {
    115_200: 0,
}

# {data_name: size_byte}
ENCODING_TABLE = {
    "Homing_Offset": CONTROL_TABLE["Homing_Offset"][1],
    "Goal_PWM": CONTROL_TABLE["Goal_PWM"][1],
    "Goal_Current": CONTROL_TABLE["Goal_Current"][1],
    "Goal_Velocity": CONTROL_TABLE["Goal_Velocity"][1],
    "Present_PWM": CONTROL_TABLE["Present_PWM"][1],
    "Present_Current": CONTROL_TABLE["Present_Current"][1],
    "Present_Velocity": CONTROL_TABLE["Present_Velocity"][1],
}

MODEL_ENCODING_TABLE = {
    "lx16a": ENCODING_TABLE,
}

# {model: model_resolution}
# https://emanual.robotis.com/docs/en/dxl/x/{MODEL}/#specifications
MODEL_RESOLUTION = {
    "lx16a": 4096,
}

# {model: model_number}
# https://emanual.robotis.com/docs/en/dxl/x/{MODEL}/#control-table-of-eeprom-area
MODEL_NUMBER_TABLE = {
    "lx16a": 1000,
}

# {model: available_operating_modes}
# https://emanual.robotis.com/docs/en/dxl/x/{MODEL}/#operating-mode11
MODEL_OPERATING_MODES = {
    "lx16a": [0, 1, 3, 4, 5, 16],
}

MODEL_CONTROL_TABLE = {
    "lx16a": CONTROL_TABLE,
}

MODEL_BAUDRATE_TABLE = {
    "lx16a": BAUDRATE_TABLE,
}

AVAILABLE_BAUDRATES = [
    115_200,
]
