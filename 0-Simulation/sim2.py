import math
import time
import numpy as np
from onshape_to_robot import simulation
from colorama import Fore, Back, Style
from transforms3d.quaternions import mat2quat, quat2mat
import pybullet as p
import pygame
import argparse
import kinematics

sim = simulation.Simulation("rrr/robot.urdf", fixed=True, panels=True)
parser = argparse.ArgumentParser()
parser.add_argument("--mode", "-m", type=str, default="direct")
args = parser.parse_args()
sliders = {}
target = None
joints = sim.getJoints()
bx = 0.07
bz = 0.25

if args.mode == "direct":
    target = p.loadURDF("target2/robot.urdf")
    for joint in joints:
        sliders[joint] = p.addUserDebugParameter(joint, -math.pi, math.pi, 0.0)
elif args.mode == "inverse" or args.mode == "inverse-iterative":
    target = p.loadURDF("target2/robot.urdf")
    sliders["target_x"] = p.addUserDebugParameter("target_x", -1, 1, 0.4)
    sliders["target_y"] = p.addUserDebugParameter("target_y", -1, 1, 0)
    sliders["target_z"] = p.addUserDebugParameter("target_z", -1, 1, 0.25)
elif args.mode == "triangle" or args.mode == "triangle-points":
    sliders["triangle_x"] = p.addUserDebugParameter("triangle_x", 0.01, 0.8, 0.4)
    sliders["triangle_z"] = p.addUserDebugParameter("triangle_z", -0.2, 0.3, 0)
    sliders["triangle_h"] = p.addUserDebugParameter("triangle_h", 0.01, 0.3, 0.1)
    sliders["triangle_w"] = p.addUserDebugParameter("triangle_w", 0.01, 0.3, 0.2)
elif args.mode == "circle" or args.mode == "circle-points":
    sliders["circle_x"] = p.addUserDebugParameter("circle_x", -1, 1, 0.4)
    sliders["circle_z"] = p.addUserDebugParameter("circle_z", -1, 1, 0.1)
    sliders["circle_r"] = p.addUserDebugParameter("circle_r", 0.01, 0.3, 0.1)
    sliders["circle_duration"] = p.addUserDebugParameter("circle_duration", 0.01, 10, 3)

lastLine = time.time()
lastInverse = 0
targets = {"motor" + str(k + 1): 0 for k in range(3)}

while True:
    if sim.t > 1.0:
        if args.mode == "direct":
            for joint in joints:
                targets[joint] = p.readUserDebugParameter(sliders[joint])

            T = kinematics.computeDK(
                -targets["motor1"], -targets["motor2"], targets["motor3"]
            )
            # T = model.direct(targets)
            T[0] += bx
            T[2] += bz

            p.resetBasePositionAndOrientation(target, T, [0, 0, 0, 1])

        elif args.mode == "inverse" or args.mode == "inverse-iterative":
            x = p.readUserDebugParameter(sliders["target_x"])
            y = p.readUserDebugParameter(sliders["target_y"])
            z = p.readUserDebugParameter(sliders["target_z"])
            p.resetBasePositionAndOrientation(target, [x + bx, y, z + bz], [0, 0, 0, 1])

            if args.mode == "inverse":
                alphas = kinematics.computeIK(x, y, z)
                print(
                    "Asked IK for x:{}, y:{}, z{}, got theta1:{}, theta2:{}, theta3:{}".format(
                        x, y, z, alphas[0], alphas[1], alphas[2]
                    )
                )
                targets = {
                    "motor1": -alphas[0],
                    "motor2": -alphas[1],
                    "motor3": alphas[2],
                }
            elif args.mode == "inverse-iterative":
                if (time.time() - lastInverse) > 0.1:
                    alphas = kinematics.inverseIterative(x, y, z)
                    targets = {
                        "motor1": -alphas[0],
                        "motor2": -alphas[1],
                        "motor3": alphas[2],
                    }

        elif args.mode == "triangle":
            x = p.readUserDebugParameter(sliders["triangle_x"])
            z = p.readUserDebugParameter(sliders["triangle_z"])
            h = p.readUserDebugParameter(sliders["triangle_h"])
            w = p.readUserDebugParameter(sliders["triangle_w"])

            alphas = kinematics.triangle(x, z, h, w, sim.t)
            targets = {
                "motor1": -alphas[0],
                "motor2": -alphas[1],
                "motor3": alphas[2],
            }
            pos = kinematics.computeDK(alphas[0], alphas[1], alphas[2])
            pos[0] += bx
            pos[2] += bz
            sim.addDebugPosition(pos, duration=3)

        elif args.mode == "circle":
            x = p.readUserDebugParameter(sliders["circle_x"])
            z = p.readUserDebugParameter(sliders["circle_z"])
            r = p.readUserDebugParameter(sliders["circle_r"])
            duration = p.readUserDebugParameter(sliders["circle_duration"])
            alphas = kinematics.circle(x, z, r, sim.t, duration)

            targets = {
                "motor1": -alphas[0],
                "motor2": -alphas[1],
                "motor3": alphas[2],
            }
            pos = kinematics.computeDK(alphas[0], alphas[1], alphas[2])
            pos[0] += bx
            pos[2] += bz
            sim.addDebugPosition(pos, duration=3)

        sim.setJoints(targets)

    sim.tick()
