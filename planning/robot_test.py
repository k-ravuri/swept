from motion import *
from klampt.math import vectorops,so3
from klampt import vis
import math
import time
import numpy as np

def spin(t):
    start_time = time.time()
    while time.time() - start_time < t:
        vis.show()
        time.sleep(0.1)

def wait_for_config(robot, target, mode, timeout=10, tol = 0.01):
    start_time = time.time()
    while time.time() - start_time < timeout:
        if mode == "robot":
            config = robot.getConfig()
        elif mode == "left":
            config = robot.getConfig()[7:13]
        elif mode == "right":
            mode = None
        err = np.linalg.norm(np.array(config) - np.array(target))
        if err < tol:
            break
        time.sleep(0.1)

def reset_arms(motionapi):
    leftTuckedConfig = [0.7934980392456055, -2.541288038293356, -2.7833543555, 4.664876623744629, -0.049166981373, 0.09736919403076172]
    rightTuckedConfig = motionapi.mirror_arm_config(leftTuckedConfig)
    target_config = [0]*7 + leftTuckedConfig + [0, 0] + rightTuckedConfig + [0]
    motionapi.setLeftLimbPositionLinear(leftTuckedConfig,5)
    motionapi.setRightLimbPositionLinear(rightTuckedConfig,5)
    wait_for_config(motionapi.robot_model, target_config, "robot")

def ik_solve(robot, target_position, first_time = False):
    orig_config = robot.getConfig()
    left_ee_link = robot.link(13)
    left_active_dofs = [7, 8, 9, 10, 11, 12]

    ee_local_pos = [0, 0, 0]
    h = 0.1
    local = [ee_local_pos, vectorops.madd(ee_local_pos, (1, 0, 0), -h)]
    world = [target_position, vectorops.madd(target_position, (0, 0, 1), h)]
    goal = ik.objective(left_ee_link, local=local, world=world)

    if first_time:
        solved = ik.solve_global(goal, activeDofs = left_active_dofs, startRandom=True)
    else:
        solved = ik.solve(goal, activeDofs = left_active_dofs)
    new_config = robot.getConfig()[7:13]
    robot.setConfig(orig_config)

    if solved:
        print("solve success")
        return new_config
    else:
        print("Solve unsuccessful")
        return None

motionapi = Motion(mode = 'Kinematic', codename = "anthrax")
motionapi.startup()
robot = motionapi.robot_model

world = motionapi.getWorld()
vis.add("world",world)
vis.show()

reset_arms(motionapi)

positions = [(0.5, 0.5), (0.5, 0.4), (0.5, 0.3), (0.5, 0.2), (0.5, 0.1), (0.5, 0.0), (0.5, -0.1)]
first_time = True

# run the simulation (indefinitely)
for e in positions:
    rv = ik_solve(robot, [e[0], e[1], 0.65], first_time)
    if rv is not None:
        motionapi.setLeftLimbPositionLinear(rv, 2)
        wait_for_config(robot, rv, mode = "left")
    vis.show()
    first_time = False
vis.run()
##use EE velocity drive

motionapi.shutdown()
vis.kill()
