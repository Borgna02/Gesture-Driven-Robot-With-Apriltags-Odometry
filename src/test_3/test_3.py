# Importante che sim_start sia per primo perché se il simulatore non è avviato non posso eseguire gli altri import
from sim_start import sim, CONFIG
from omni.isaac.kit import SimulationApp
from pxr import Usd, UsdGeom, Gf
import numpy as np
import paho.mqtt.client as mqtt
import cv2

import pyapriltags
from omni.usd import get_context, get_world_transform_matrix, get_local_transform_matrix
from omni.isaac.wheeled_robots.controllers.differential_controller import DifferentialController
from omni.isaac.sensor import Camera
from omni.isaac.core.utils.stage import is_stage_loading
from omni.isaac.wheeled_robots.robots import WheeledRobot
from omni.isaac.core import World


SCENE_PATH = "src/scene_warehouse.usd"
JETSON_PRIM_PATH = "/World/jetbot"
CAMERA_PRIM_PATH = JETSON_PRIM_PATH + "/chassis/rgb_camera/jetbot_cam"
IMAGE_PATH = "src/test_3/test.png"
IMAGE_RESOLUTION = (1920, 1200)



class Instrinsics:
    def __init__(self, camera: Camera):

        # Recupero gli instrinsics dal simulatore
        ((self.fx, _, self.cx), (_, self.fy, self.cy),
         (_, _, _)) = camera.get_intrinsics_matrix()


    def to_list(self):
        return [self.fx, self.fy, self.cx, self.cy]


# I seguenti import funzionano solo DOPO che la simulazione è stata lanciata


class ImageUtils:
    @staticmethod
    def array_to_image(array):
        # Assicurati che 'array' sia un array di numpy per evitare errori
        array = np.asarray(array)

        # Creazione di un'immagine vuota con la stessa forma e tipo di dato di 'array'
        image = np.zeros(array.shape, dtype=array.dtype)

        # Copia dei dati da 'array' a 'image'
        image[:] = array

        return image


class MyJetbot:

    def __init__(self, world: World):

        self.jetbot: WheeledRobot = world.scene.add(
            WheeledRobot(
                prim_path=JETSON_PRIM_PATH,
                # name="my_jetbot",
                wheel_dof_names=["left_wheel_joint", "right_wheel_joint"]
            )
        )

        self.controller = DifferentialController(
            name="simple_control", wheel_radius=0.03, wheel_base=0.1125)

        self.camera = Camera(prim_path=CAMERA_PRIM_PATH,
                             resolution=IMAGE_RESOLUTION,
                             )
        self.camera.initialize()
        print(self.camera.get_intrinsics_matrix())
        self.intrinsics = Instrinsics(self.camera)

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
        self.jetbot.apply_wheel_actions(
            self.controller.forward(command=[velvec[0], velvec[1]]))

    def read_camera(self):
        rgba_array = self.camera.get_current_frame()["rgba"]
        image = ImageUtils.array_to_image(rgba_array)
        image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
        # Salvataggio dell'immagine
        cv2.imwrite(IMAGE_PATH, image)
        return image


class MqttManager:

    def __init__(self, jetbot: MyJetbot, broker_address="localhost", port=1883):
        self.client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2)
        self.client.connect(broker_address, port)
        self.client.on_message = self.on_message
        self.client.subscribe("command")
        self.jetbot = jetbot
        self.client.loop_start()  # diverso da loop_forever perché lo esegue su thread separato
        self.is_reading = False  # Variabile semaforo per non sovrapporre la lettura dei messaggi

    def on_message(self, client, userdata, message):
        print(message.payload.decode('utf-8'))
        self.jetbot.do_action(message.payload.decode('utf-8'))

    def check_messages(self):
        if not self.is_reading:
            self.is_reading = True
            # Controlla i messaggi per un breve periodo di tempo
            self.client.loop(timeout=0.01)
            self.is_reading = False


class AprilTagsManager:

    def __init__(self):
        self.detector = pyapriltags.Detector()

    def detect(self, image, my_jetbot: MyJetbot):

        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

        # Esegui la detection
        result = self.detector.detect(
            gray, estimate_tag_pose=True, camera_params=my_jetbot.intrinsics.to_list(), tag_size=0.05)

        stage: Usd.Stage = get_context().get_stage()
        prim: Usd.Prim = stage.GetPrimAtPath(CAMERA_PRIM_PATH)
        matrix = get_world_transform_matrix(prim)
        local_matrix = get_local_transform_matrix(prim)
        print("\nCamera position: " + str(matrix) + "\n\n")
        print("\nLocal camera transform: " + str(local_matrix) + "\n\n")
        exit()
        translate = matrix.ExtractTranslation()

        tag_center_pos = (0.21546, 0.0, 0.0001)
        tag_top_sx_pos = (0.23549, 0.01632, 0.0001)

        distanza_centro = np.linalg.norm(
            np.array(translate) - np.array(tag_center_pos))
        distanza_top_sx = np.linalg.norm(
            np.array(translate) - np.array(tag_top_sx_pos))

        # Stampa i risultati
        for tag in result:

            (ptA, ptB, ptC, ptD) = tag.corners
            ptB = (int(ptB[0]), int(ptB[1]))
            ptC = (int(ptC[0]), int(ptC[1]))
            ptD = (int(ptD[0]), int(ptD[1]))
            ptA = (int(ptA[0]), int(ptA[1]))
            # draw the bounding box of the AprilTag detection
            cv2.line(image, ptA, ptB, (0, 255, 0), 2)
            cv2.line(image, ptB, ptC, (0, 255, 0), 2)
            cv2.line(image, ptC, ptD, (0, 255, 0), 2)
            cv2.line(image, ptD, ptA, (0, 255, 0), 2)
            # draw the center (x, y)-coordinates of the AprilTag
            (cX, cY) = (int(tag.center[0]), int(tag.center[1]))
            cv2.circle(image, (cX, cY), 5, (0, 0, 255), -1)

            cv2.imwrite("src/test_3/test_tag.png", image)

            norm = round(np.linalg.norm(tag.pose_t), 3)
            print(
                f"ID: {tag.tag_id}, Norm: {norm}, Dist calcolata manualmente: centro : {distanza_centro}, topsx: {distanza_top_sx}")
            # print(f"ID: {tag.tag_id}, Norm: {norm} m")

            break


class SimHandler:

    def __init__(self, simulation: SimulationApp):
        self.sim = simulation

        # open stage
        get_context().open_stage(SCENE_PATH)

        # wait two frames so that stage starts loading
        self.sim.update()
        self.sim.update()

        print("Loading stage...")

        while is_stage_loading():
            self.sim.update()
        print("Loading Complete")

        self.world = World(stage_units_in_meters=1.0)

    def start_program(self, my_jetbot: MyJetbot, mqtt_client: MqttManager, at_manager: AprilTagsManager):
        while self.sim.is_running():
            self.world.step(render=not CONFIG["headless"])

            # deal with pause/stop
            if self.world.is_playing():
                if self.world.current_time_step_index == 0:
                    self.world.reset()

                mqtt_client.check_messages()
                image = my_jetbot.read_camera()
                at_manager.detect(image, my_jetbot)


if __name__ == "__main__":
    sim_handler = SimHandler(sim)
    my_jetbot = MyJetbot(sim_handler.world)
    mqtt_client = MqttManager(my_jetbot)
    at_manager = AprilTagsManager()

    sim_handler.world.reset()
    sim_handler.start_program(my_jetbot, mqtt_client, at_manager)

    sim_handler.sim.close()
