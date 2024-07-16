#!PySketch/executor.py

###############################################################################
##
# Copyright (C) Daniele Di Ottavio (aka Tetsuo)
# Contact: tetsuo.tek (at) gmail (dot) com
##
# This program is free software; you can redistribute it and/or
# modify it under the terms of the GNU General Public License
# as published by the Free Software Foundation; either version 3
# of the License, or (at your option) any later version.
##
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
##
# You should have received a copy of the GNU General Public License
# along with this program; if not, write to the Free Software
# Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA  02111-1307, USA.
##
###############################################################################

# Calcolo intrinsics

# int width = thresholded_image_gray.size().width;
# int height = thresholded_image_gray.size().height;
# double f = (width/2) / 0.6032; //tan(fov_h/2);


# detection.getRelativeTranslationRotation(tagSize, f, f, width / 2, height / 2, translation, rotation);

import json
import math

from coppeliasim_zmqremoteapi_client import RemoteAPIClient
import cv2
from PySketch.abstractflow import FlowChannel
from PySketch.flowproto import Variant_T, Flow_T
from PySketch.flowsat import FlowSat

from PySketch.elapsedtime import ElapsedTime

###############################################################################

import argparse
import numpy as np
from timing import TICK_LEN

###############################################################################
# CLASSES

HEIGHT = 240
WIDTH = 320


class CameraController:

    def __init__(self):

        print("Connecting to simulator...")
        self._cSim_client = RemoteAPIClient(host="localhost")
        self._sim = self._cSim_client.getObject('sim')
        print("Connected to SIM")
        self._camera_handle = self._sim.getObject('./rgb')
        self._camera_height = self._sim.getObjectPosition(self._camera_handle)[
            2]
        self._colorView = self._sim.floatingViewAdd(0.69, 0.9, 0.2, 0.2, 0)
        self._sim.adjustView(self._colorView, self._camera_handle, 64)

        # Larghezza immagine
        self._width = self._sim.getObjectInt32Param(
            self._camera_handle, self._sim.visionintparam_resolution_x)
        # Altezza immagine
        self._height = self._sim.getObjectInt32Param(
            self._camera_handle, self._sim.visionintparam_resolution_y)

        print("Res: ", self._width, self._height)

        # fov in radianti
        self._fov = self._sim.getObjectFloatParam(
            self._camera_handle, self._sim.visionfloatparam_perspective_angle)

        # # Calcola gli intrinsics
        fy = self._height / (2 * math.tan(self._fov/2))
        fx = fy * (self._width/self._height)
        cx = self._width / 2
        cy = self._height / 2

        self._intrinsics = (fx, fy, cx, cy)

        self._sim.startSimulation()

    def read_image(self):
        byte_data, resolution = self._sim.getVisionSensorImg(
            self._camera_handle)

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

        # Converti l'immagine in scala di grigi
        image_bn = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

        # Applica la sogliatura
        image_thresh = cv2.threshold(image_bn, 127, 255, cv2.THRESH_BINARY)[1]

        # Trasforma l'immagine in un array numpy
        array_tresh = image_thresh.reshape((resolution[0], resolution[1],  1))

        # Estrai i bit dalla matrice
        bits = [1 if x == 255 else 0 for x in array_tresh.flatten().tolist()]

        # Raggruppa la lista modificata in byte (sottoliste di 8 elementi)
        bytes_list = [bits[i:i + 8] for i in range(0, len(bits), 8)]

        # Converti ogni gruppo di 8 bit in un byte
        bytes_output = [int("".join(map(str, byte)), 2) for byte in bytes_list]

        # Converti la lista di byte in un bytearray
        bytearr = bytearray(bytes_output)

        # Mostra l'immagine
        # cv2.imshow("Camera", image_thresh)
        cv2.waitKey(1)

        return bytearr


###############################################################################
# SKETCH

sat = FlowSat()
timer = sat._timer
chronoCameraPub = ElapsedTime()

cameraChanName = "cam"
cameraChan = None

controller = None


def setup():
    print("[SETUP] ..")

    global controller
    controller = CameraController()

    parser = argparse.ArgumentParser(description="Nao publisher")
    parser.add_argument('sketchfile', help='Sketch program file')
    parser.add_argument(
        '--user', help='Flow-network username', default='guest')
    parser.add_argument(
        '--password', help='Flow-network password', default='password')
    args = parser.parse_args()

    sat.setLogin(args.user, args.password)
    sat.setNodeName("Camera")

    t = TICK_LEN  # seconds
    sat.setTickTimer(t, t * 50)

    sat.setNewChanCallBack(onChannelAdded)
    sat.setDelChanCallBack(onChannelRemoved)

    sat.setStartChanPubReqCallBack(onStartChanPub)
    sat.setStopChanPubReqCallBack(onStopChanPub)

    sat.setGrabDataCallBack(onDataGrabbed)

    ok = sat.connect()  # uses the env-var ROBOT_ADDRESS

    if ok:
        # Per non mostrare il monitor degli errori
        sat.setSpeedMonitorEnabled(True)

        print("[LOOP] ..")
        props = {
            "resolution": f"{controller._width}x{controller._height}",
            "intrinsics": json.dumps(controller._intrinsics),
            "camera_height": controller._camera_height,
            "fov": controller._fov
        }

        sat.addStreamingChannel(
            Flow_T.FT_BLOB, Variant_T.T_BYTEARRAY, cameraChanName, props=props)

    return ok


def loop():

    if (cameraChan):
        if (chronoCameraPub.stop() > 0.1):
            image_bytes = controller.read_image()

            sat.publish(cameraChan.chanID, image_bytes)

            chronoCameraPub.start()

    sat.tick()
    return sat.isConnected()

###############################################################################
# CALLBACKs


def onChannelAdded(ch: FlowChannel):

    global cameraChan

    if (ch.name == f"guest.{cameraChanName}"):
        print("Channel ADDED: {}".format(ch.name))
        cameraChan = ch


def onChannelRemoved(ch):
    print("Channel REMOVED: {}".format(ch.name))


def onStartChanPub(ch):
    print("Publish START required: {}".format(ch.name))


def onStopChanPub(ch):
    print("Publish STOP required: {}".format(ch.name))


def onDataGrabbed(chanID, data):
    pass

###############################################################################
