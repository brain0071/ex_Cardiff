#!/usr/bin/env python3

import os
from pyrep import PyRep
from pyrep.robots.arms.panda import Panda
from pyrep.robots.end_effectors.panda_gripper import PandaGripper
from pyrep.objects.shape import Shape
from pyrep.objects.dummy import Dummy

import math
import time

rootDir = "/home/"
fileToSearch = "franka_vrep_demo_w_cam.ttt"

for relPath, dirs, files in os.walk(rootDir):
    if fileToSearch in files:
        fullPath = os.path.join(rootDir, relPath, fileToSearch)
        print(fullPath)


DELTA = 0.01
pr = PyRep()
pr.launch(fullPath, headless=False)
# Launch the application with a scene file that contains a robot
pr.start()  # Start the simulation

arm = Panda()  # Get the panda from the scene
gripper = PandaGripper()  # Get the panda gripper from the scene

cup = Shape("Cup")
waypoints = [Dummy("waypoint%d" % i) for i in range(5)]
pos, quat = arm.get_tip().get_position(), arm.get_tip().get_quaternion()

# input('Press any key to start...')
time.sleep(3)
path = arm.get_path(
    position=pos,
    euler=[0, math.radians(90), 0],
    ignore_collisions=True,
)
done = False
while not done:
    done = path.step()
    pr.step()

print("Moving to pre-grasp ...")
path = arm.get_path(
    position=waypoints[0].get_position(), quaternion=quat, ignore_collisions=True
)
done = False
while not done:
    done = path.step()
    pr.step()

print("Moving to grasp ...")
path = arm.get_path(
    position=waypoints[1].get_position(), quaternion=quat, ignore_collisions=True
)
done = False
while not done:
    done = path.step()
    pr.step()


print("Closing gripper ...")
while not gripper.actuate(0.0, 0.4):
    pr.step()
gripper.grasp(cup)

print("Moving to post-grasp ...")
path = arm.get_path(
    position=waypoints[0].get_position(), quaternion=quat, ignore_collisions=True
)
done = False
while not done:
    done = path.step()
    pr.step()

print("Moving to pre-place ...")
path = arm.get_path(position=waypoints[3].get_position(), quaternion=quat)
done = False
while not done:
    done = path.step()
    pr.step()

print("Moving to place ...")
path = arm.get_path(position=waypoints[2].get_position(), quaternion=quat)
done = False
while not done:
    done = path.step()
    pr.step()


print("Opening gripper ...")
while not gripper.actuate(1.0, 0.4):
    pr.step()
gripper.release()

print("Moving to post-place ...")
path = arm.get_path(position=waypoints[3].get_position(), quaternion=quat)
done = False
while not done:
    done = path.step()
    pr.step()

print("Moving to home ...")
path = arm.get_path(
    position=waypoints[4].get_position(), quaternion=quat, ignore_collisions=False
)
done = False
while not done:
    done = path.step()
    pr.step()


# input('Press enter to finish ...')
print("Finish :-) ...")
pr.stop()
pr.shutdown()
