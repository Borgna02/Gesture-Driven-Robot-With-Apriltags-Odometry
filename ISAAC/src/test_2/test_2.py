# Copyright (c) 2020-2021, NVIDIA CORPORATION.  All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto.  Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.

import argparse
from omni.isaac.kit import SimulationApp
import numpy as np
import omni
from time import sleep
from PIL import Image

# This sample loads a usd stage and starts simulation
CONFIG = {"width": 1280, "height": 720, "sync_loads": True, "headless": False, "renderer": "RayTracedLighting"}


# Set up command line arguments
parser = argparse.ArgumentParser("Usd Load sample")
parser.add_argument("--headless", default=False, action="store_true", help="Run stage headless")

args, unknown = parser.parse_known_args()
# Start the omniverse application
CONFIG["headless"] = args.headless
simulation_app = SimulationApp(launch_config=CONFIG)

from omni.isaac.core import World
from omni.isaac.wheeled_robots.robots import WheeledRobot

# open stage
omni.usd.get_context().open_stage("src/scene_jetbot_apriltag.usd")

# wait two frames so that stage starts loading
simulation_app.update()
simulation_app.update()

print("Loading stage...")
from omni.isaac.core.utils.stage import is_stage_loading

while is_stage_loading():
    simulation_app.update()
print("Loading Complete")

world = World(stage_units_in_meters=1.0)

# robot = world.scene.add(Robot(prim_path="/World/jetbot", name="robot"))

jetbot = world.scene.add(
    WheeledRobot(
        prim_path="/World/jetbot",
        name="my_jetbot",
        wheel_dof_names=["left_wheel_joint", "right_wheel_joint"]
    )
)
from omni.isaac.sensor import Camera
from omni.isaac.wheeled_robots.controllers.differential_controller import DifferentialController

my_controller = DifferentialController(name="simple_control", wheel_radius=0.03, wheel_base=0.1125)


camera = Camera(prim_path="/World/jetbot/chassis/rgb_camera/jetbot_camera", resolution=(1024, 819))
camera.initialize()

world.reset()

while simulation_app.is_running():
    world.step(render=not args.headless)

    # deal with pause/stop
    if world.is_playing():
        if world.current_time_step_index == 0:
            world.reset()
        velvec = np.load("src/test_2/velvec.npy")
        jetbot.apply_wheel_actions(my_controller.forward(command=[velvec[0], velvec[1]]))
        
        rgba_array = camera.get_current_frame()["rgba"]
        image = Image.fromarray(rgba_array.astype('uint8'))
        image.save("src/test_2/test.png")
        
        sleep(0.05)
        

simulation_app.close()
