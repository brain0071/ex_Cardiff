#!/usr/bin/env python3

import os
from pyrep import PyRep
from pyrep.robots.arms.panda import Panda
from pyrep.robots.end_effectors.panda_gripper import PandaGripper
from pyrep.objects.shape import Shape
from pyrep.objects.dummy import Dummy
from pyrep.objects.vision_sensor import VisionSensor
from pyrep.const import ConfigurationPathAlgorithms as Algos
import open3d as o3d
import math
import time
import numpy as np
import matplotlib.pyplot as plt


rootDir = "/home/"
fileToSearch = "franka_vrep_demo_w_cam2.ttt"

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

# cup = Shape("Cup")
waypoints = [Dummy("waypoint%d" % i) for i in range(5)]
pos, quat = arm.get_tip().get_position(), arm.get_tip().get_quaternion()
vision = VisionSensor("Vision_sensor")
vision.set_explicit_handling(value=1)


# input("Press any key to start...")
print("Moving to pre-grasp ...")
path = arm.get_path(
    algorithm=Algos.RRTConnect,
    position=waypoints[4].get_position(),
    euler=[math.radians(-180), math.radians(0), math.radians(-90)],
    ignore_collisions=True,
)
done = False
while not done:
    done = path.step()
    pr.step()

vision.handle_explicitly()
depth = vision.capture_depth(in_meters=True)
rgb = vision.capture_rgb()
i = 0
while i != 1000:
    pr.step()
    i = i + 1

#print(depth)
#print(len(depth))
#print(rgb)
#print(len(rgb))

img_depth = o3d.geometry.Image((depth * 255).astype(np.uint8))
o3d.io.write_image("img_d.png", img_depth)
o3d.visualization.draw_geometries([img_depth])

img_rgb = o3d.geometry.Image((rgb * 255).astype(np.uint8))
o3d.io.write_image("img_r.png", img_rgb)
o3d.visualization.draw_geometries([img_rgb])

rgbd_image = o3d.geometry.RGBDImage.create_from_color_and_depth(
    img_rgb, img_depth, convert_rgb_to_intensity=True
)

plt.subplot(1, 2, 1)
plt.title("grayscale image")
plt.imshow(rgbd_image.color)
plt.subplot(1, 2, 2)
plt.title("depth image")
plt.imshow(rgbd_image.depth)
plt.show()

# print(vision.get resolution())
w = vision.get_resolution()[0]
h = vision.get_resolution()[1]
fl_x = vision.get_intrinsic_matrix()[0][0]
fl_y = vision.get_intrinsic_matrix()[1][1]
c_x = vision.get_intrinsic_matrix()[0][2]
c_y = vision.get_intrinsic_matrix()[1][2]

intrinsic = o3d.camera.PinholeCameraIntrinsic(
    width=w, height=h, fx=fl_x, fy=fl_y, cx=c_x, cy=c_y
)
pcd = o3d.geometry.PointCloud.create_from_rgbd_image(rgbd_image, intrinsic)
pcd.transform([[-1, 0, 0, 0], [0, 1, 0, 0], [0, 0, -1, 0], [0, 0, 0, 1]])
o3d.visualization.draw_geometries([pcd])

# input('Press enter to finish ...')
print("Finish :-) ...")
pr.stop()
pr.shutdown()