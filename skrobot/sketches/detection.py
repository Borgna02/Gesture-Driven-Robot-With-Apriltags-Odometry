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


import csv
import json
import math
import struct

from coppeliasim_zmqremoteapi_client import RemoteAPIClient
import cv2
import pandas as pd
from PySketch.abstractflow import FlowChannel
from PySketch.flowsync import FlowSync
from PySketch.flowproto import FlowChanID, Variant_T, Flow_T
from PySketch.flowsat import FlowSat

from PySketch.elapsedtime import ElapsedTime

###############################################################################

import argparse
import numpy as np
from timing import TICK_LEN
from dt_apriltags import Detection, Detector
import pickle
import zipfile
import matplotlib.pyplot as plt
import os
import csv


###############################################################################
# CLASSES

RADIANS_COEFF = 180/3.14159265358979323846


class DetectionController:

    def __init__(self):
        self._intrinsics = None
        self._height = 0
        self._width = 0
        self._camera_height = 0
        self._fov = 0

    def detect(self, image):
        if (not self._intrinsics):
            return None
        
        
        # Dewarping
        black = np.zeros((self._height, self._width, 3), np.uint8)
        # cv2.imshow('black', black)
        
        pts_src = np.array([[0, 0], [self._width, 0], [self._width, self._height], [0, self._height]])
        pts_dst = np.array([[0, 0], [self._width, 0], [500, self._height], [140, self._height]])
        
        h, _ = cv2.findHomography(pts_src, pts_dst)
        
        dewarped_image = cv2.warpPerspective(image, h, (black.shape[1], black.shape[0]))
        
        # cv2.imshow('dewarped', dewarped_image)
        cv2.imwrite('dewarped.png', dewarped_image)

        at_detector = Detector(families='tag36h11',
                               nthreads=1,
                               quad_decimate=1.0,
                               quad_sigma=0.0,
                               refine_edges=1,
                               decode_sharpening=0.25,
                               debug=0)

        results: list[Detection] = at_detector.detect(
            image, True, self._intrinsics, 0.06)

        image = cv2.cvtColor(image, cv2.COLOR_GRAY2BGR)

        # loop over the AprilTag detection results
        for tag in results:
            # extract the bounding box (x, y)-coordinates for the AprilTag
            # and convert each of the (x, y)-coordinate pairs to integers
            for idx in range(len(tag.corners)):
                cv2.line(image, tuple(
                    tag.corners[idx-1, :].astype(int)), tuple(tag.corners[idx, :].astype(int)), (0, 255, 0))

            # draw the center (x, y)-coordinates of the AprilTag
            (cX, cY) = (int(tag.center[0]), int(tag.center[1]))
            cv2.circle(image, (cX, cY), 5, (0, 0, 255), -1)

          

            ###################### Calcolo della distanza ################################

            # Vettore di traslazione, ovvero coordinate del tag rispetto alla camera
            translation_vector = tag.pose_t

            # Calcolo della distanza tramite la libreria
            norm = round(np.linalg.norm(translation_vector), 2)

            # Calcolo della proiezione della distanza sul piano orizzontale
            plane_distance_sq = norm ** 2 - self._camera_height ** 2
            if plane_distance_sq < 0:
                plane_distance_sq = 0  # Imposta a zero se il valore è negativo

            plane_distance = round(math.sqrt(plane_distance_sq), 2)

            # Calcolo della distanza tramite le coordinate globali del tag e della camera
            tag_coords = np.array(
                [*sim.getObjectPosition(sim.getObject(f"./tag{tag.tag_id}")), 1])
            cam_coords = np.array(
                [*sim.getObjectPosition(sim.getObject("./rgb")), 1])
            
            plane_distance_manual_sq = np.linalg.norm(
                tag_coords - cam_coords) ** 2 - self._camera_height ** 2
            if plane_distance_manual_sq < 0:
                plane_distance_manual_sq = 0  # Imposta a zero se il valore è negativo

            plane_distance_manual = round(
                math.sqrt(plane_distance_manual_sq), 2)
            
            # Calcolo della distanza tramite il modello di ML
            DISTANCE = round(model.predict(pd.DataFrame({"distanza_calcolata": [
                plane_distance], "tag_center_x": [cX], "tag_center_y": [cY]}))[0], 2)
            
            # print(f"Distanza: {plane_distance}, Manual dist: {plane_distance_manual}, Pred. dist: {DISTANCE}, Diff: {round(abs(plane_distance - plane_distance_manual), 2)}, Diff con predetto: {round(abs(plane_distance_manual - DISTANCE), 2)}")


            ######################## Calcolo dello yaw ####################################
            
            
            
            tags_in_dewarped = at_detector.detect(dewarped_image, False, tag_size=0.06)
            
            if not tags_in_dewarped:
                return image, -1, -1, -1
            
            corners = list(tags_in_dewarped[0].corners)

            
            
            
            # Vettore da corners[0] (bottom_sx) a corners[1] (bottom_dx), ovvero parallelo all'asse x dell'arena
            vector1 = np.array([corners[0][0] - corners[1][0], corners[0][1] - corners[1][1]])

            # Versore opposto all'asse y dell'immagine, ovvero la visuale del robot
            vector2 = np.array([0, 10])
            
            # Calcola il prodotto scalare dei vettori
            dot_product = np.dot(vector1, vector2)

            # Calcola la lunghezza dei vettori
            vector1_length = np.linalg.norm(vector1)
            vector2_length = np.linalg.norm(vector2)

            # Calcola il coseno dell'angolo usando la formula del prodotto scalare
            cos_theta = dot_product / (vector1_length * vector2_length)

            # Calcola l'angolo in radianti
            abs_yaw = np.degrees(np.arccos(cos_theta))

            # Determinazione del segno dello yaw
            yaw_sign = 1 if corners[0][0] < corners[1][0] else -1

            # Calcolo dello yaw
            YAW = yaw_sign * abs_yaw
            
            
            real_orientation = math.degrees(sim.getObjectOrientation(sim.getObject("./PioneerP3DX"))[2])
         
            with open('orientation_error_with_dewarping.csv', 'a', newline='') as csvfile:
                writer = csv.writer(csvfile)
                writer.writerow([YAW, real_orientation])

            return image, tag.tag_id, DISTANCE, YAW


        return image, -1, -1, -1

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


cameraChanName = "cam"
cameraChan = None

distYawPhiChanName = "dist_yaw_phi"
distYawPhiChan = None


controller = None
sim = None

model = None


def setup():
    print("[SETUP] ..")
    parser = argparse.ArgumentParser(description="Nao publisher")
    parser.add_argument('sketchfile', help='Sketch program file')
    parser.add_argument(
        '--user', help='Flow-network username', default='guest')
    parser.add_argument(
        '--password', help='Flow-network password', default='password')
    args = parser.parse_args()

    global controller
    global sim
    controller = DetectionController()

    print("Connecting to simulator...")
    sim = RemoteAPIClient(host="localhost").getObject('sim')
    print("Connected to SIM")

    global model

    # Percorso dell'archivio ZIP che contiene il modello
    zip_path = 'modello.zip'

    # Estrarre il file modello.pkl dall'archivio ZIP
    with zipfile.ZipFile(zip_path, 'r') as zipf:
        zipf.extract('modello.pkl', path='.')

    # Caricare il modello dal file modello.pkl
    with open('modello.pkl', 'rb') as file:
        model = pickle.load(file)

    # Rimozione del file modello.pkl dopo l'uso
   
    os.remove('modello.pkl')

    sat.setLogin(args.user, args.password)
    sat.setAppName("Detection")

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
        sat.addStreamingChannel(
            Flow_T.FT_BLOB, Variant_T.T_BYTEARRAY, distYawPhiChanName)

    return ok


def loop():

    sat.tick()
    return sat.isConnected()

###############################################################################
# CALLBACKs


def onChannelAdded(ch: FlowChannel):

    global cameraChan
    global distYawPhiChan
    if (ch.name == f"guest.{cameraChanName}"):
        print("Channel ADDED: {}".format(ch.name))
        cameraChan = ch

        sat.subscribeChannel(ch.chanID)

        sync = sat.newSyncClient()
        sync.setCurrentDbName(cameraChan.name)
        controller._width = int(sync.getVariable("resolution").split("x")[0])
        controller._height = int(sync.getVariable("resolution").split("x")[1])
        controller._camera_height = float(sync.getVariable("camera_height"))

        controller._intrinsics = sync.getVariable("intrinsics")
        controller._fov = float(sync.getVariable("fov"))

        # Converto la stringa in una lista di float
        controller._intrinsics = [float(x) for x in controller._intrinsics.replace(
            "[", "").replace("]", "").split(",")]
        sync.close()
    if (ch.name == f"guest.{distYawPhiChanName}"):
        print("Channel ADDED: {}".format(ch.name))
        distYawPhiChan = ch


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

        image, ID, DISTANCE, YAW = controller.detect(image)

        
        data = struct.pack("<bff", ID, DISTANCE, YAW)
        sat.publish(distYawPhiChan.chanID, data)

        cv2.imshow('image', image)
        cv2.waitKey(1)


###############################################################################
