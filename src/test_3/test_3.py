from omni.isaac.kit import SimulationApp
import numpy as np
import omni
from time import sleep
from PIL import Image
import paho.mqtt.client as mqtt


class Constants:
    CONFIG = {"width": 1280, "height": 720, "sync_loads": True, "headless": False, "renderer": "RayTracedLighting"}
    SCENE_PATH = "src/scene_jetbot_apriltag.usd"
    JETSON_PRIM_PATH = "/World/jetbot"
    CAMERA_PRIM_PATH = JETSON_PRIM_PATH + "/chassis/rgb_camera/jetbot_camera"
    COMMAND_MANUAL_PATH = "src/test_3/velvec.npy"
    IMAGE_PATH = "src/test_3/test.png"

sim = SimulationApp(launch_config=Constants.CONFIG)

# I seguenti import funzionano solo DOPO che la simulazione è stata lanciata
from omni.isaac.core import World
from omni.isaac.wheeled_robots.robots import WheeledRobot
from omni.isaac.core.utils.stage import is_stage_loading
from omni.isaac.sensor import Camera
from omni.isaac.wheeled_robots.controllers.differential_controller import DifferentialController

class SimHandler():
    
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
        
    def start_program(self, my_jetbot, mqtt_client):
        while self.sim.is_running():
            self.world.step(render=not Constants.CONFIG["headless"])

            # deal with pause/stop
            if self.world.is_playing():
                if self.world.current_time_step_index == 0:
                    self.world.reset()
                    
                mqtt_client.check_messages()
                my_jetbot.read_camera()
                
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
        
    def do_action(self, message):
        velvec = None
        if message == "su":
            velvec = [1, 0]
        elif message == "giù":
            velvec = [-1, 0]
        elif message == "sinistra":
            velvec = [0, 1]
        elif message == "destra": 
            velvec = [0, -1]
        else:
            velvec = [0, 0]
        self.jetbot.apply_wheel_actions(self.controller.forward(command=[velvec[0], velvec[1]]))
        
    def read_camera(self):
        rgba_array = self.camera.get_current_frame()["rgba"]
        image = Image.fromarray(rgba_array.astype('uint8'))
        image.save(Constants.IMAGE_PATH)

class MqttManager:
    
    def __init__(self, jetbot: MyJetbot, broker_address="localhost", port=1883):
        self.client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION1)
        self.client.connect(broker_address, port)
        self.client.on_message = self.on_message
        self.client.subscribe("command")
        self.jetbot = jetbot
        self.client.loop_start() # diverso da loop_forever perché lo esegue su thread separato
        self.is_reading = False  # Variabile semaforo per non sovrapporre la lettura dei messaggi

        

    def on_message(self, client, userdata, message):
        print(message.payload.decode('utf-8'))
        self.jetbot.do_action(message.payload.decode('utf-8'))

    def check_messages(self):
        if not self.is_reading:
            self.is_reading = True
            self.client.loop(timeout=0.01)  # Controlla i messaggi per un breve periodo di tempo
            self.is_reading = False


if __name__ == "__main__":
    sim_handler = SimHandler(sim)
    my_jetbot = MyJetbot(sim_handler.world)
    mqtt_client = MqttManager(my_jetbot)
    
    sim_handler.world.reset()
    sim_handler.start_program(my_jetbot, mqtt_client)

    sim_handler.sim.close()
