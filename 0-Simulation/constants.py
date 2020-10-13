import sys
import math

BIOLOID = "BIOLOID"
PHANTOMX = "PHANTOMX"
PHANTOMX_SIMULATION = "PHANTOMX_SIMULATION"
ARM_SIMULATION = "ARM_SIMULATION"
AX12 = "AX12"
MOTOR_TYPE = AX12
ROBOT_TYPE = ARM_SIMULATION
USING_SIMPLE_ROBOT = True
USE_RADS = False

if ROBOT_TYPE == PHANTOMX:
    constL1 = 54.8
    constL2 = 65.3
    constL3 = 133
    theta2Correction = 16.0
    theta2ExtraCorrection = 0
    theta3Correction = 43.76  # -theta2Correction ??
    THETA3_DK_SIGN = 1
    THETA2_IK_SIGN = -1
    USE_RADS = False
    Z_DIRECTION = -1
elif ROBOT_TYPE == BIOLOID:
    constL1 = 51
    constL2 = 63.7
    constL3 = 93
    # Angle to match the theory with reality for theta 2 (measures of the triangle are 22.5, 60.7, 63.7). => Angle =  -20.69
    theta2Correction = -20.69
    # Same goes for theta 3 : +90 - 20.69 - a. Where a = asin(8.2/93) = 5.06
    theta3Correction = 90 + theta2Correction - 5.06
    THETA3_DK_SIGN = -1
    THETA2_IK_SIGN = 1
    USE_RADS = False
    Z_DIRECTION = -1
elif ROBOT_TYPE == PHANTOMX_SIMULATION:
    constL1 = 0.054
    constL2 = 0.0645
    constL3 = 0.1248
    theta2Correction = -16.0 * math.pi / 180.0
    theta3Correction = -43.76 * math.pi / 180.0 + theta2Correction
    THETA3_DK_SIGN = -1
    THETA2_IK_SIGN = 1
    USE_RADS = True
    Z_DIRECTION = 1
elif ROBOT_TYPE == ARM_SIMULATION:
    constL1 = 0.085
    constL2 = 0.185
    constL3 = 0.250
    theta2Correction = 0
    theta3Correction = 0
    THETA3_DK_SIGN = 1
    THETA2_IK_SIGN = 1
    USE_RADS = True
    Z_DIRECTION = 1
else:
    print("ERROR: Unknwon ROBOT_TYPE '{}'".format(ROBOT_TYPE))
    sys.exit()
