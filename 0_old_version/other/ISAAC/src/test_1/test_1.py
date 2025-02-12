# Questo programma crea un'istanza di isaac con un jetbot. Quindi gli da una velocità leggendo il file velvec.npy.
# Il file viene modificato tramite lo script manual_control.py

import argparse

from omni.isaac.kit import SimulationApp

parser = argparse.ArgumentParser()
parser.add_argument("--test", default=False, action="store_true", help="Run in test mode")
args, unknown = parser.parse_known_args()


simulation_app = SimulationApp({"headless": False})

import carb
import numpy as np
from omni.isaac.core import World
from omni.isaac.core.utils.nucleus import get_assets_root_path
from omni.isaac.wheeled_robots.controllers.differential_controller import DifferentialController
from omni.isaac.wheeled_robots.robots import WheeledRobot

my_world = World(stage_units_in_meters=1.0)
assets_root_path = get_assets_root_path()
if assets_root_path is None:
    carb.log_error("Could not find Isaac Sim assets folder")
jetbot_asset_path = assets_root_path + "/Isaac/Robots/Jetbot/jetbot.usd"
my_jetbot = my_world.scene.add(
    WheeledRobot(
        prim_path="/World/Jetbot",
        name="my_jetbot",
        wheel_dof_names=["left_wheel_joint", "right_wheel_joint"],
        create_robot=True,
        usd_path=jetbot_asset_path,
        position=np.array([0, 0.0, 2.0]),
    )
)
my_world.scene.add_default_ground_plane()
my_controller = DifferentialController(name="simple_control", wheel_radius=0.03, wheel_base=0.1125)
my_world.reset()

i = 0
while simulation_app.is_running():
    my_world.step(render=True)
    if my_world.is_playing():
        if my_world.current_time_step_index == 0:
            my_world.reset()
            my_controller.reset()
            i += 1
        # if i >= 0 and i < 1000:
        #     # forward
        #     my_jetbot.apply_wheel_actions(my_controller.forward(command=[0.05, 0]))
        #     print(my_jetbot.get_linear_velocity())
        # elif i >= 1000 and i < 1300:
        #     # rotate
        #     my_jetbot.apply_wheel_actions(my_controller.forward(command=[0.0, np.pi / 12]))
        #     print(my_jetbot.get_angular_velocity())
        # elif i >= 1300 and i < 2000:
        #     # forward
        #     my_jetbot.apply_wheel_actions(my_controller.forward(command=[0.05, 0]))
        # elif i == 2000:
        #     i = 0
        # i += 1
        velvec = np.load("src/velvec.npy")
        my_jetbot.apply_wheel_actions(my_controller.forward(command=[velvec[0], velvec[1]]))
    if args.test is True:
        break


simulation_app.close()
