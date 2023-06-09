#!/usr/bin/env python
import math
from math import degrees
import sys
import os
import time
import argparse
import pybullet as p
from onshape_to_robot.simulation import Simulation
import kinematics
import pypot.dynamixel
#ports = pypot.dynamixel.get_available_ports()
#dxl_io = pypot.dynamixel.DxlIO(ports[0],baudrate=1000000)

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
leg_angle = -math.pi / 4


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
    alphas = kinematics.computeDK(0, 0, 0, use_rads=True)
    controls["target_x"] = p.addUserDebugParameter("target_x", -0.4, 0.4, alphas[0])
    controls["target_y"] = p.addUserDebugParameter("target_y", -0.4, 0.4, alphas[1])
    controls["target_z"] = p.addUserDebugParameter("target_z", -0.4, 0.4, alphas[2])
elif args.mode == "triangle":
    controls["triangle_x"] = p.addUserDebugParameter("triangle_x", 0.01, 0.8, 0.01)
    controls["triangle_z"] = p.addUserDebugParameter("triangle_z", -0.2, 0.3, 0)
    controls["triangle_h"] = p.addUserDebugParameter("triangle_h", 0.01, 0.3, 0.1)
    controls["triangle_w"] = p.addUserDebugParameter("triangle_w", 0.01, 0.3, 0.05)
    controls["direction"] = p.addUserDebugParameter("direction", 0, math.pi * 2, math.pi/2)
    controls["duration"] = p.addUserDebugParameter("duration", 1, 15, 3)
elif args.mode == "cercle":
    controls["cercle_x"] = p.addUserDebugParameter("cercle_x", 0.01, 1.0, 0.5)
    controls["cercle_z"] = p.addUserDebugParameter("cercle_z", -0.2, 0.3, 0)
    controls["rotation"] = p.addUserDebugParameter("rotation", -1, 1, 0)
    controls["cercle_w"] = p.addUserDebugParameter("cercle_w", 0.01, 0.3, 0.05)
    #controls["direction"] = p.addUserDebugParameter("direction", 0, math.pi * 2, math.pi/2)
    controls["duration"] = p.addUserDebugParameter("duration", 1, 15, 3)

while True:
    targets = {}
    for name in sim.getJoints():
        if "c1" in name or "thigh" in name or "tibia" in name:
            targets[name] = 0
    if args.mode == "frozen-direct":
        for name in controls.keys():
            targets[name] = p.readUserDebugParameter(controls[name])
        points = kinematics.computeDKDetailed(
            targets["j_c1_rf"],
            targets["j_thigh_rf"],
            targets["j_tibia_rf"],
            use_rads=True,
        )
        i = -1
        T = []
        for pt in points:
            # Drawing each step of the DK calculation
            i += 1
            
            T.append(kinematics.rotation_2D(pt[0], pt[1], pt[2], leg_angle))
            T[-1][0] += leg_center_pos[0]
            T[-1][1] += leg_center_pos[1]
            T[-1][2] += leg_center_pos[2]
            # print("Drawing cross {} at {}".format(i, T))
            p.resetBasePositionAndOrientation(
                crosses[i], T[-1], to_pybullet_quaternion(0, 0, leg_angle)
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
        # alphas = kinematics.computeIKOriented(x, y, z, 0)
        alphas = kinematics.computeIK(x, y, z, verbose=True, use_rads=True)

        dk0 = kinematics.computeDK(0, 0, 0, use_rads=True)
        targets["j_c1_rf"] = alphas[0]
        targets["j_thigh_rf"] = alphas[1]
        targets["j_tibia_rf"] = alphas[2]

        state = sim.setJoints(targets)
        # Temp
        sim.setRobotPose([0, 0, 0.5], [0, 0, 0, 1])

        T = kinematics.rotation_2D(x, y, z, leg_angle)
        T[0] += leg_center_pos[0]
        T[1] += leg_center_pos[1]
        T[2] += leg_center_pos[2]
        # print("Drawing cross {} at {}".format(i, T))
        p.resetBasePositionAndOrientation(
            cross, T, to_pybullet_quaternion(0, 0, leg_angle)
        )
        
    # elif args.mode == "triangle":
    #     x = p.readUserDebugParameter(controls["triangle_x"])
    #     z = p.readUserDebugParameter(controls["triangle_z"])
    #     h = p.readUserDebugParameter(controls["triangle_h"])
    #     w = p.readUserDebugParameter(controls["triangle_w"])
        
    #     alphas_1 = kinematics.triangle(x, z, h, w, sim.t)

    #     alphas_2 = kinematics.triangle(x, z, h, w, sim.t + 1.5)
        

         
    #     for name in sim.getJoints():

    #         if "rf" in name:
    #             alphas = alphas_1
    #         if "rm" in name:
    #             alphas = alphas_2
    #         if 'rr' in name :
    #             alphas = alphas_1


    #         if "lf" in name:
    #             alphas = alphas_2

    #         if "lm" in name:
    #             alphas = alphas_1

    #         if 'lr' in name :
    #             alphas = alphas_2


    #         if "c1" in name :
    #             targets[name] = alphas[0]

    #         if "thigh" in name:
    #             targets[name] = alphas[1]

    #         if "tibia" in name:
    #             targets[name] = alphas[2]


    #     state = sim.setJoints(targets)

    # elif args.mode == "triangle":
    #     x = p.readUserDebugParameter(controls["triangle_x"])
    #     z = p.readUserDebugParameter(controls["triangle_z"])
    #     h = p.readUserDebugParameter(controls["triangle_h"])
    #     w = p.readUserDebugParameter(controls["triangle_w"])
        
    #     # alphas_1 = kinematics.triangle(x, z, h, w, sim.t)

    #     # alphas_2 = kinematics.triangle(x, z, h, w, sim.t + 1.5)
        

         
    #     for name in sim.getJoints():

    #         if "rf" in name:
    #             alphas = kinematics.triangle(x, z, h, w, sim.t + 1.5, -math.pi/4)
    #         if "rm" in name:
    #             alphas = kinematics.triangle(x, z, h, w, sim.t, 0)
    #         if 'rr' in name :
    #             alphas = kinematics.triangle(x, z, h, w, sim.t + 1.5, math.pi/4)
    #         if "lf" in name:
    #             alphas = kinematics.triangle(x, z, h, w, sim.t, math.pi/4)
    #         if "lm" in name:
    #             alphas = kinematics.triangle(x, z, h, w, sim.t + 1.5, 0)
    #         if 'lr' in name :
    #             alphas = kinematics.triangle(x, z, h, w, sim.t, -math.pi/4)


    #         if "c1" in name :
    #             targets[name] = alphas[0]

    #         if "thigh" in name:
    #             targets[name] = alphas[1]

    #         if "tibia" in name:
    #             targets[name] = alphas[2]


    #     state = sim.setJoints(targets)
    #     sim.setRobotPose([0, 0, 0.5], to_pybullet_quaternion(0, 0, 0))

    # sim.tick()


    elif args.mode == "triangle":
        x = p.readUserDebugParameter(controls["triangle_x"])
        z = p.readUserDebugParameter(controls["triangle_z"])
        h = p.readUserDebugParameter(controls["triangle_h"])
        w = p.readUserDebugParameter(controls["triangle_w"])
        direction = p.readUserDebugParameter(controls["direction"])
        duration = p.readUserDebugParameter(controls["duration"])
        
        # alphas_1 = kinematics.triangle(x, z, h, w, sim.t)

        # alphas_2 = kinematics.triangle(x, z, h, w, sim.t + 1.5)
        

         
        for name in sim.getJoints():

            if "rf" in name:
                alphas = kinematics.triangle(x, z, h, w, duration, sim.t + 1.5,1, direction)
            if "rm" in name:
                alphas = kinematics.triangle(x, z, h, w, duration, sim.t,2, direction)
            if 'rr' in name :
                alphas = kinematics.triangle(x, z, h, w, duration, sim.t + 1.5, 3, direction)
            if "lf" in name:
                alphas = kinematics.triangle(x, z, h, w, duration, sim.t, 0, direction)
            if "lm" in name:
                alphas = kinematics.triangle(x, z, h, w, duration, sim.t + 1.5, 5, direction)
            if 'lr' in name :
                alphas = kinematics.triangle(x, z, h, w, duration, sim.t, 4, direction)


            if "c1" in name :
                targets[name] = alphas[0]

            if "thigh" in name:
                targets[name] = alphas[1]

            if "tibia" in name:
                targets[name] = alphas[2]


        state = sim.setJoints(targets)
        # sim.setRobotPose([0, 0, 0.5], to_pybullet_quaternion(0, 0, 0))


        #ajout tourner sur lui même
    elif args.mode == "cercle":
        x = p.readUserDebugParameter(controls["cercle_x"])
        z = p.readUserDebugParameter(controls["cercle_z"])
        h = p.readUserDebugParameter(controls["rotation"])
        w = p.readUserDebugParameter(controls["cercle_w"])
        #direction = p.readUserDebugParameter(controls["direction"])
        duration = p.readUserDebugParameter(controls["duration"])
        
        # alphas_1 = kinematics.triangle(x, z, h, w, sim.t)

        # alphas_2 = kinematics.triangle(x, z, h, w, sim.t + 1.5)
        

         
        for name in sim.getJoints():

            if "rf" in name:
                alphas = kinematics.circle(x, z, h, w, duration)
            if "rm" in name:
                alphas = kinematics.circle(x, z, h, w, duration)
            if 'rr' in name :
                alphas = kinematics.circle(x, z, h, w, duration)
            if "lf" in name:
                alphas = kinematics.circle(x, z, h, w, duration)
            if "lm" in name:
                alphas = kinematics.circle(x, z, h, w, duration)
            if 'lr' in name :
                alphas = kinematics.circle(x, z, h, w, duration)

            if "c1" in name :
                targets[name] = alphas[0]




        state = sim.setJoints(targets)
            
        # sim.setRobotPose([0, 0, 0.5], to_pybullet_quaternion(0, 0, 0))

    sim.tick()