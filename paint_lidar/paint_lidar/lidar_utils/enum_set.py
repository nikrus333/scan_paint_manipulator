from enum import Enum, unique


class SelectModeWork(Enum):
    SERVO_LIDAR_SIMULATION = False    
    MANIPULATOR_SIMULATION = False
    LIDAR_SIMULATION = False
    VISUALIZATION = False

@unique
class ModeWork(Enum):
    SCAN_AND_PAINT = 1
    CALCULATE_RANGE = 2
    ONLY_PAINT = 3


@unique
class ChooseStartManip(Enum):
    START_MANIPULATOR = 1
    SKIP_TRAJECTORY = 2


class ParametrsManipulator(Enum):
    SPHERE_RADIUS = 0.8
    MIN_DISTATION = 0.475
    MAX_DISTATION = 0.65
    TRANS_PLANE = 0.0
    DIST_PAINT = 0.75 + 0.30 
