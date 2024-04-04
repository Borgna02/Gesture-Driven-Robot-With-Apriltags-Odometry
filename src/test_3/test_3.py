
import numpy as np
import omni
from time import sleep
from PIL import Image
import paho.mqtt.client as mqtt
import cv2
import pyapriltags
from omni.isaac.kit import SimulationApp


class Constants:
    CONFIG = {"width": 1280, "height": 720, "sync_loads": True,
              "headless": False, "renderer": "RayTracedLighting"}
    SCENE_PATH = "src/scene_warehouse.usd"
    JETSON_PRIM_PATH = "/World/jetbot"
    CAMERA_PRIM_PATH = JETSON_PRIM_PATH + "/chassis/rgb_camera/jetbot_cam"
    IMAGE_PATH = "src/test_3/test.png"
    IMAGE_RESOLUTION = (256,256)


class Instrinsics:
    def __init__(self, camera):

        # https://forums.developer.nvidia.com/t/change-intrinsic-camera-parameters/180309/4

        width, height = camera.get_resolution()

        # Matrice ricavata dalla calibrazione con la scacchiera
        # camera_matrix = [[433.028, 0.0, 541.725], [0.0, 415.378, 659.105], [0.0, 0.0, 1.0]]
        camera_matrix = [[958.8, 0.0, 957.8], [
            0.0, 956.7, 589.5], [0.0, 0.0, 1.0]]

        ((self.fx, _, self.cx), (_, self.fy, self.cy), (_, _, _)) = camera_matrix

        # in mm, 3 microns is a common pixel size for high resolution cameras
        pixel_size = 3 * 1e-3
        # f_stop = 1.8            # f-number, the ratio of the lens focal length to the diameter of the entrance pupil
        # f-number, the ratio of the lens focal length to the diameter of the entrance pupil
        f_stop = 0.0
        focus_distance = 0.6    # in meters, the distance from the camera to the object plane

        # The aperture size in mm
        horizontal_aperture = pixel_size * width
        vertical_aperture = pixel_size * height
        focal_length_x = self.fx * pixel_size
        focal_length_y = self.fy * pixel_size
        focal_length = (focal_length_x + focal_length_y) / \
            2         # The focal length in mm

        # Set the camera parameters, note the unit conversion between Isaac Sim sensor and Kit
        # Convert from mm to cm (or 1/10th of a world unit)
        camera.set_focal_length(focal_length / 10.0)
        # The focus distance in meters
        camera.set_focus_distance(focus_distance)
        # Convert the f-stop to Isaac Sim units
        camera.set_lens_aperture(f_stop * 100.0)
        # Convert from mm to cm (or 1/10th of a world unit)
        camera.set_horizontal_aperture(horizontal_aperture / 10.0)
        camera.set_vertical_aperture(vertical_aperture / 10.0)

        camera.set_clipping_range(0.05, 1.0e5)

    def to_list(self):
        return [self.fx, self.fy, self.cx, self.cy]


sim = SimulationApp(launch_config=Constants.CONFIG)

# I seguenti import funzionano solo DOPO che la simulazione è stata lanciata
from omni.isaac.core import World
from omni.isaac.wheeled_robots.robots import WheeledRobot
from omni.isaac.core.utils.stage import is_stage_loading
from omni.isaac.sensor import Camera
from omni.isaac.wheeled_robots.controllers.differential_controller import DifferentialController
from pxr import Gf
import omni.isaac.core.utils.numpy.rotations as rot_utils


class ImageUtils:
    def array_to_image(array):
        # Creazione di un'immagine vuota di dimensione Constants.IMAGE_RESOLUTION con 4 canali (RGBA)
        image = np.zeros((Constants.IMAGE_RESOLUTION[0], Constants.IMAGE_RESOLUTION[1], 4), dtype=np.uint8)

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

    def start_program(self, my_jetbot, mqtt_client, at_manager):
        while self.sim.is_running():
            self.world.step(render=not Constants.CONFIG["headless"])

            # deal with pause/stop
            if self.world.is_playing():
                if self.world.current_time_step_index == 0:
                    self.world.reset()

                mqtt_client.check_messages()
                image = my_jetbot.read_camera()
                at_manager.detect(image, my_jetbot)


class MyJetbot:

    def __init__(self, world: World):

        self.jetbot = world.scene.add(
            WheeledRobot(
                prim_path=Constants.JETSON_PRIM_PATH,
                # name="my_jetbot",
                wheel_dof_names=["left_wheel_joint", "right_wheel_joint"]
            )
        )

        self.controller = DifferentialController(
            name="simple_control", wheel_radius=0.03, wheel_base=0.1125)
        
        #! Capire bene come posizionare nel punto giusto la camera perché così risulta sbagliata sia l'angolazione che la posizione
        self.camera = Camera(prim_path=Constants.CAMERA_PRIM_PATH,
                             position=np.array([0.025, 0.0, 0.0]),
                             frequency=20,
                             resolution=Constants.IMAGE_RESOLUTION,
                             orientation=rot_utils.euler_angles_to_quats(
                                 np.array([-45, 90, -45]), degrees=True),
                             )
        self.camera.initialize()
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
        cv2.imwrite(Constants.IMAGE_PATH, image)
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

        # stage = omni.usd.get_context().get_stage()
        # prim = stage.GetPrimAtPath(Constants.CAMERA_PRIM_PATH)
        # matrix: Gf.Matrix4d = omni.usd.get_world_transform_matrix(prim)
        # translate: Gf.Vec3d = matrix.ExtractTranslation()

        # tag_pos = (0.21546, 0.0, 0.0001)

        # distanza = np.linalg.norm(np.array(translate) - np.array(tag_pos))

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
            # print(f"ID: {tag.tag_id}, Norm: {norm}, Dist calcolata manualmente: {distanza}")
            print(f"ID: {tag.tag_id}, Norm: {norm} m")

            break


if __name__ == "__main__":
    sim_handler = SimHandler(sim)
    my_jetbot = MyJetbot(sim_handler.world)
    mqtt_client = MqttManager(my_jetbot)
    at_manager = AprilTagsManager()

    sim_handler.world.reset()
    sim_handler.start_program(my_jetbot, mqtt_client, at_manager)

    sim_handler.sim.close()
