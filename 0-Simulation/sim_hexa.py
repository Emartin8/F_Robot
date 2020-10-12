#!/usr/bin/env python
import math
import sys
import os
import time
import argparse
import pybullet as p
from onshape_to_robot.simulation import Simulation
import kinematics

# from squaternion import Quaternion
from scipy.spatial.transform import Rotation


def to_pybullet_quaternion(roll, pitch, yaw, degrees=False):
    # q = Quaternion.from_euler(roll, pitch, yaw, degrees=degrees)
    # return [q[1], q[2], q[3], q[0]]

    # Create a rotation object from Euler angles specifying axes of rotation
    rot = Rotation.from_euler("xyz", [roll, pitch, yaw], degrees=degrees)

    # Convert to quaternions and print
    rot_quat = rot.as_quat()
    # print(rot_quat)
    return rot_quat


# m_friction
parser = argparse.ArgumentParser()
parser.add_argument("--mode", "-m", type=str, default="direct", help="test")
args = parser.parse_args()
controls = {}
robotPath = "phantomx_description/urdf/phantomx.urdf"
sim = Simulation(robotPath, gui=True, panels=True, useUrdfInertia=False)
# sim.setFloorFrictions(lateral=0, spinning=0, rolling=0)
pos, rpy = sim.getRobotPose()
sim.setRobotPose([0, 0, 0.5], [0, 0, 0, 1])

leg_center_pos = [0.1248, -0.06164, 0.001116 + 0.5]


if args.mode == "frozen-direct":
    crosses = []
    for i in range(4):
        crosses.append(p.loadURDF("target2/robot.urdf"))
    for name in sim.getJoints():
        print(name)
        if "c1" in name or "thigh" in name or "tibia" in name:
            controls[name] = p.addUserDebugParameter(name, -math.pi, math.pi, 0)
elif args.mode == "direct":
    for name in sim.getJoints():
        print(name)
        if "c1" in name or "thigh" in name or "tibia" in name:
            controls[name] = p.addUserDebugParameter(name, -math.pi, math.pi, 0)
elif args.mode == "inverse":
    cross = p.loadURDF("target2/robot.urdf")
    controls["target_x"] = p.addUserDebugParameter("target_x", -1, 1, 0.4)
    controls["target_y"] = p.addUserDebugParameter("target_y", -1, 1, 0)
    controls["target_z"] = p.addUserDebugParameter("target_z", -1, 1, 0.25)
    controls["cross_x"] = p.addUserDebugParameter("cross_x", -1, 1, 0.4)
    controls["cross_y"] = p.addUserDebugParameter("cross_y", -1, 1, 0)
    controls["cross_z"] = p.addUserDebugParameter("cross_z", -1, 1, 0.25)


while True:
    targets = {}
    for name in sim.getJoints():
        if "c1" in name or "thigh" in name or "tibia" in name:
            targets[name] = 0
    if args.mode == "frozen-direct":
        leg_angle = -math.pi / 4
        for name in controls.keys():
            targets[name] = p.readUserDebugParameter(controls[name])
        points = kinematics.computeDKDetailed(
            targets["j_c1_rf"], targets["j_thigh_rf"], targets["j_tibia_rf"]
        )
        i = -1
        for T in points:
            # Drawing each step of the DK calculation
            i += 1
            T = kinematics.rotaton_2D(T[0], T[1], T[2], leg_angle)
            T[0] += leg_center_pos[0]
            T[1] += leg_center_pos[1]
            T[2] += leg_center_pos[2]
            # print("Drawing cross {} at {}".format(i, T))
            p.resetBasePositionAndOrientation(
                crosses[i], T, to_pybullet_quaternion(0, 0, leg_angle)
            )
        # Temp
        sim.setRobotPose([0, 0, 0.5], to_pybullet_quaternion(0, 0, 0))
        # sim.setRobotPose(
        #     leg_center_pos, to_pybullet_quaternion(0, 0, 0),
        # )
        state = sim.setJoints(targets)
    elif args.mode == "direct":
        for name in controls.keys():
            targets[name] = p.readUserDebugParameter(controls[name])
        state = sim.setJoints(targets)
    elif args.mode == "inverse":
        x = p.readUserDebugParameter(controls["target_x"])
        y = p.readUserDebugParameter(controls["target_y"])
        z = p.readUserDebugParameter(controls["target_z"])
        alphas = kinematics.computeIKOriented(x, y, z, 4)

        # print(
        #     "Asked IK for x:{}, y:{}, z{}, got theta1:{}, theta2:{}, theta3:{}".format(
        #         x, y, z, alphas[0], alphas[1], alphas[2]
        #     )
        # )
        targets["j_c1_rf"] = alphas[0]
        targets["j_thigh_rf"] = -alphas[1]
        targets["j_tibia_rf"] = alphas[2]
        # targets = {
        #     "j_c1_rf": -alphas[0],
        #     "j_thigh_rf": -alphas[1],
        #     "j_tibia_rf": alphas[2],
        # }
        state = sim.setJoints(targets)
        # Temp
        sim.setRobotPose([0, 0, 0.5], [0, 0, 0, 1])

        cross_x = p.readUserDebugParameter(controls["cross_x"])
        cross_y = p.readUserDebugParameter(controls["cross_y"])
        cross_z = p.readUserDebugParameter(controls["cross_z"])
        p.resetBasePositionAndOrientation(
            cross, [cross_x, cross_y, cross_z], [0, 0, 0, 1]
        )

    sim.tick()

