from omni.isaac.kit import SimulationApp
import numpy as np
import omni
from time import sleep
from PIL import Image
import paho.mqtt.client as mqtt
import cv2
import pyapriltags


class Constants:
    CONFIG = {"width": 1280, "height": 720, "sync_loads": True, "headless": False, "renderer": "RayTracedLighting"}
    SCENE_PATH = "src/camera_calibration/camera_calibration_scene.usd"
    JETSON_PRIM_PATH = "/World/jetbot"
    CAMERA_PRIM_PATH = JETSON_PRIM_PATH + "/chassis/rgb_camera/jetbot_cam"
    IMAGE_PATH = "src/camera_calibration/image"



sim = SimulationApp(launch_config=Constants.CONFIG)

# I seguenti import funzionano solo DOPO che la simulazione è stata lanciata
from omni.isaac.core import World
from omni.isaac.wheeled_robots.robots import WheeledRobot
from omni.isaac.core.utils.stage import is_stage_loading
from omni.isaac.sensor import Camera
from omni.isaac.wheeled_robots.controllers.differential_controller import DifferentialController
from pxr import Gf

class ImageUtils:
    def array_to_image(array):
        # Creazione di un'immagine vuota di dimensione 1024x1024 con 4 canali (RGBA)
        image = np.zeros((1024, 1024, 4), dtype=np.uint8)

        # Assegnazione dei valori RGBA all'immagine
        image[:, :, 0] = array[:, :, 0]  # Red
        image[:, :, 1] = array[:, :, 1]  # Green
        image[:, :, 2] = array[:, :, 2]  # Blue
        image[:, :, 3] = array[:, :, 3]  # Alpha

        return image

class SimHandler:
    
    def __init__(self, simulation: SimulationApp):
        self.sim = simulation
              
        
        # open stage
        omni.usd.get_context().open_stage(Constants.SCENE_PATH)

        # wait two frames so that stage starts loading
        self.sim.update()
        self.sim.update()

        print("Loading stage...")

        while is_stage_loading():
            self.sim.update()
        print("Loading Complete")

        self.world = World(stage_units_in_meters=1.0)
        
    def start_program(self, my_jetbot):
        i = 0
        while self.sim.is_running():
            self.world.step(render=not Constants.CONFIG["headless"])

            # deal with pause/stop
            if self.world.is_playing():
                if self.world.current_time_step_index == 0:
                    self.world.reset()
                    
                if(i % 100 == 0):
                    image = my_jetbot.read_camera()
                    cv2.imwrite(Constants.IMAGE_PATH + str(i) + ".png", image)
                    
                i += 1
                
                
class MyJetbot:
    
    def __init__(self, world: World):

        self.jetbot = world.scene.add(
            WheeledRobot(
                prim_path=Constants.JETSON_PRIM_PATH,
                # name="my_jetbot",
                wheel_dof_names=["left_wheel_joint", "right_wheel_joint"]
            )
        )
        
        self.controller = DifferentialController(name="simple_control", wheel_radius=0.03, wheel_base=0.1125)
        self.camera = Camera(prim_path=Constants.CAMERA_PRIM_PATH, resolution=(1024, 1024))
        self.camera.initialize()
        
    
        
    def read_camera(self):
        rgba_array = self.camera.get_current_frame()["rgba"]
        image = ImageUtils.array_to_image(rgba_array)
        # Visualizzazione dell'immagine
        return image



            
            

if __name__ == "__main__":
    sim_handler = SimHandler(sim)
    my_jetbot = MyJetbot(sim_handler.world)

        
    sim_handler.world.reset()
    sim_handler.start_program(my_jetbot)

    sim_handler.sim.close()
