from coppeliasim_zmqremoteapi_client import RemoteAPIClient
import numpy as np
import cv2


client = RemoteAPIClient()
sim = client.getObject('sim')
sim.startSimulation()
camera_handle = sim.getObject('./rgb')


i = 0
while True:
    try:

        input("Press Enter to capture image...")
        byte_data, resolution = sim.getVisionSensorImg(camera_handle)

        # Verifica che la lunghezza dei dati sia corretta
        expected_size = resolution[0] * resolution[1] * 3  # 3 canali per RGB
        if len(byte_data) != expected_size:
            raise ValueError(
                f"Dimensione dei dati non corretta: {len(byte_data)} != {expected_size}")

        # Converti i byte in un array numpy
        np_array = np.frombuffer(byte_data, np.uint8)

        # Reshape l'array numpy in un'immagine
        # height, width, channels
        image = np_array.reshape((resolution[1], resolution[0], 3))
        image = cv2.flip(image, 0)

        # Mostra l'immagine
        cv2.imwrite(f"skrobot/camera_calibration/image{i}.png", image)
        cv2.waitKey(0)
        i += 1

    except KeyboardInterrupt:
        break

sim.stopSimulation()
