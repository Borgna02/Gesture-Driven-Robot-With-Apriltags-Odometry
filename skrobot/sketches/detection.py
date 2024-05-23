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
import struct

import cv2
from PySketch.abstractflow import FlowChannel
from PySketch.flowsync import FlowSync
from PySketch.flowproto import FlowChanID, Variant_T, Flow_T
from PySketch.flowsat import FlowSat

from PySketch.elapsedtime import ElapsedTime

###############################################################################

import argparse
import numpy as np
from timing import TICK_LEN

###############################################################################
# CLASSES


class DetectionController:

    def __init__(self):
        self._intrinsics = None
        self._height = 0
        self._width = 0

    def detect(self, image):
        if (not self._intrinsics):
            return None

        # # Call the detect_apriltags function in the detect method
        # tags = detect_apriltags(image)
        # return tags

    @staticmethod
    def bytearray_to_image(bytearr, resolution):
        # Converti il bytearray in una lista di byte
        bytes_list = list(bytearr)

        # Converti ogni byte in una lista di 8 bit
        bits = []
        for byte in bytes_list:
            bits.extend([int(bit) for bit in format(byte, '08b')])

        # Crea un array numpy dai bit
        bit_array = np.array(bits, dtype=np.uint8)

        # Reshape l'array numpy nella dimensione originale dell'immagine
        image_shape = (resolution[0], resolution[1])
        image_array = bit_array.reshape(image_shape)

        # Converti i bit in valori di pixel (0 o 255) per ottenere l'immagine in bianco e nero
        image_bn = np.where(image_array == 1, 255, 0).astype(np.uint8)

        return image_bn


###############################################################################
# SKETCH

sat = FlowSat()
timer = sat._timer
chronoSensePub = ElapsedTime()

cameraChanName = "cam"
cameraChan = None



controller = None


def setup():
    print("[SETUP] ..")

    global controller
    controller = DetectionController()

    parser = argparse.ArgumentParser(description="Nao publisher")
    parser.add_argument('sketchfile', help='Sketch program file')
    parser.add_argument(
        '--user', help='Flow-network username', default='guest')
    parser.add_argument(
        '--password', help='Flow-network password', default='password')
    args = parser.parse_args()

    sat.setLogin(args.user, args.password)

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
        sat.setSpeedMonitorEnabled(True, "detection")

        print("[LOOP] ..")

    return ok


def loop():

    sat.tick()
    return sat.isConnected()

###############################################################################
# CALLBACKs


def onChannelAdded(ch: FlowChannel):

    global cameraChan
    if (ch.name == f"guest.{cameraChanName}"):
        print("Channel ADDED: {}".format(ch.name))
        cameraChan = ch

        sat.subscribeChannel(ch.chanID)
        
        sync = sat.newSyncClient()
        sync.setCurrentDbName(cameraChan.name)
        controller._width = int(sync.getVariable("resolution").split("x")[0])
        controller._height = int(sync.getVariable("resolution").split("x")[1])

        controller._intrinsics = sync.getVariable("intrinsics")
        sync.close()
        

  


def onChannelRemoved(ch: FlowChannel):
    print("Channel REMOVED: {}".format(ch.name))


def onStartChanPub(ch: FlowChannel):
    print("Publish START required: {}".format(ch.name))


def onStopChanPub(ch: FlowChannel):
    print("Publish STOP required: {}".format(ch.name))


def onDataGrabbed(chanID, data):
    if (chanID == cameraChan.chanID and controller._width != 0 and controller._height != 0):
        image = DetectionController.bytearray_to_image(
            data, (controller._height, controller._width))

        cv2.imshow('image', image)
        cv2.waitKey(1)



###############################################################################
