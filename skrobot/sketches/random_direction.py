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

import json
import math
import random
import struct

import cv2
from PySketch.abstractflow import FlowChannel
from PySketch.flowsync import FlowSync
from PySketch.flowproto import FlowChanID, Variant_T, Flow_T
from PySketch.flowsat import FlowSat
from PySketch.elapsedtime import ElapsedTime
from PySketch.databuffer import DataBuffer

###############################################################################

from enum import Enum
import time
import argparse
import numpy as np
from timing import TICK_LEN
import mediapipe as mp

###############################################################################
# CLASSES


class Command(Enum):
    LEFT = 0
    FRONTLEFT = 1
    FRONT = 2
    FRONTRIGHT = 3
    RIGHT = 4
    STOP = -1


class Mode(Enum):
    MANUAL = 1
    AUTO = 2


class Constants:
    # Numero di elementi utilizzati per la media
    MEAN_DIMENSION = 20
    # Somma di tutti gli elementi usati per la media pesata
    MEAN_DENOM = sum(range(1, MEAN_DIMENSION + 1))
    # Rate di refresh della camera
    CAM_REFRESH_TIME = 0.05
    # Dimensioni della finestra con la webcam
    WINDOW_WIDTH = 500


class MathUtils:

    @staticmethod
    def get_slope(point_1, point_2):
        slope = 300  # valore molto alto per simulare l'infinito

        if point_1[0] - point_2[0] != 0:
            slope = (point_1[1] - point_2[1]) / (point_1[0] - point_2[0])

        return slope

    @staticmethod
    def distance(point_1, point_2):

        return math.sqrt((point_2[0] - point_1[0])**2 + (point_2[1] - point_1[1])**2)


class handTracker():
    def __init__(self, mode=False, maxHands=1, detectionCon=0.5, modelComplexity=1, trackCon=0.5):
        # Inizializzazione del tracker con i parametri forniti
        self.mode = mode
        self.maxHands = maxHands
        self.detectionCon = detectionCon
        self.modelComplex = modelComplexity
        self.trackCon = trackCon
        self.current_lm_list = []

        # Inizializzazione di Mediapipe per il rilevamento delle mani
        self.mpHands = mp.solutions.hands
        self.hands = self.mpHands.Hands(self.mode, self.maxHands, self.modelComplex,
                                        self.detectionCon, self.trackCon)
        self.mpDraw = mp.solutions.drawing_utils

    def handsFinder(self, image, draw=True):
        # Converte l'immagine da BGR a RGB
        imageRGB = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
        # Processa l'immagine per rilevare le mani
        self.results = self.hands.process(imageRGB)
        if self.results.multi_hand_landmarks:
            for handLms in self.results.multi_hand_landmarks:
                # Disegna i landmark e le connessioni delle mani sull'immagine
                if draw:
                    self.mpDraw.draw_landmarks(
                        image, handLms, self.mpHands.HAND_CONNECTIONS)
        return image

    def compute_landmarks(self, image):
        # A ogni iterazione svuoto la lista dei landmarks
        self.current_lm_list = []

        # Trova e disegna le posizioni dei landmark delle mani sull'immagine
        self.positionFinder(image)

    # restituisce la lista di landmarks sulla mano
    def positionFinder(self, image, handNo=0, draw=True):

        if self.results.multi_hand_landmarks:
            Hand = self.results.multi_hand_landmarks[handNo]
            for id, lm in enumerate(Hand.landmark):
                # Ottiene le coordinate normalizzate dei landmark e le converte in pixel
                h, w, _ = image.shape
                cx, cy = int(lm.x * w), int(lm.y * h)
                self.current_lm_list.append([id, cx, cy])
                # Disegna un cerchio intorno al landmark sull'immagine
                if draw:
                    cv2.circle(image, (cx, cy), 10, (255, 0, 255), cv2.FILLED)
                    # Scrive l'ID del landmark all'interno del cerchio
                    cv2.putText(image, str(id), (cx - 5, cy + 5),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 2)

    def get_lm_coords(self, id):
        return (self.current_lm_list[id][1], self.current_lm_list[id][2])


class GestureController:

    def __init__(self):
        self._current_mode = Mode.AUTO 


    def publish_manual_operation(self, operation: Command):
        # Invio l'operazione solo se è diversa da quella precedente
            print("New operation: " + str(operation))
            sat.publishInt8(gestureManualChan.chanID,
                            operation.value)

    def change_mode(self):
        self._changing_mode = True
        if (self._current_mode == Mode.MANUAL):
            self._current_mode = Mode.AUTO
        else:
            self._current_mode = Mode.MANUAL
            self._mean_counter = 0
            self._mean_pos = Command.STOP

        print("Changing Mode in " + self._current_mode.name)
        
        
        sat.setCurrentDbName(gestureModeChan.name)
        sat.setVariable("mode", self._current_mode.value)
        sat.setCurrentDbName(sat._userName)
  
        


        


class ImageUtils:
    @staticmethod
    def write_on_image(image, string, pos, orientation, mode: Mode):
        if isinstance(string, Command):
            string = string.name.lower()
        pos_string = f"Position: {round(pos[0],3)}, {round(pos[1],3)}"
        orient_string = f"Orientation: {round(orientation, 3)} deg"
        # Scrive la stringa sull'immagine
        cv2.putText(image, string, (50, 50),
                    cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 0), 4)
        cv2.putText(image, mode.name, (image.shape[1] - 150, 50),
                    cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 0), 4)

        cv2.rectangle(
            image, (48, image.shape[0] - 58), (320, image.shape[0] - 15), (255, 255, 255), -1)

        cv2.putText(image, pos_string, (50, image.shape[0] - 40),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 0), 1)
        cv2.putText(image, orient_string, (50, image.shape[0] - 20),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 0), 1)

    @staticmethod
    def capture_image(device: cv2.VideoCapture, gesture_controller: GestureController, is_from_phone: bool):
        _, image = device.read()

        # Ho bisogno di sapere se l'immagine viene da DroidCAM perché in tal caso la devo ribaltare
        if (is_from_phone):
            image = cv2.rotate(image, cv2.ROTATE_90_CLOCKWISE)
        # Riflette l'immagine a specchio
        image = cv2.flip(image, 1)  # 1 indica il riflesso orizzontale

        # Rileva e disegna le mani sull'immagine
        image = gesture_controller._tracker.handsFinder(image)

        height, width, _ = image.shape
        new_height = int(Constants.WINDOW_WIDTH * height / width)
        image = cv2.resize(image, (Constants.WINDOW_WIDTH, new_height))

        gesture_controller._tracker.compute_landmarks(image)

        gesture_controller._image = image

    @staticmethod
    def show_image(image):
        # Mostra l'immagine risultante con le mani e i landmark rilevati
        cv2.imshow("Video", image)
        cv2.waitKey(1)


###############################################################################
# SKETCH
sat = FlowSat()
timer = sat._timer


chronoImage = ElapsedTime()
# Canali da gesture


gestureManualChanName = "gesture_manual"
gestureManualChan = None

gestureModeChanName = "gesture_mode"
gestureModeChan = None



controller = None
device = None


def setup():
    print("[SETUP] ..")

    # Inizializzazione della videocamera
    global device

    device = cv2.VideoCapture(0)

    if not device.isOpened():
        print("CANNOT open the camera [ID: {}]".format(0))
        return False


    global controller
    controller = GestureController()

    parser = argparse.ArgumentParser(description="Nao publisher")
    parser.add_argument('sketchfile', help='Sketch program file')
    parser.add_argument(
        '--user', help='Flow-network username', default='guest')
    parser.add_argument(
        '--password', help='Flow-network password', default='password')
    args = parser.parse_args()

    sat.setLogin(args.user, args.password)
    sat.setAppName("Random direction")

    t = TICK_LEN
    sat.setTickTimer(t, t * 20)  # Intervallo di check

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
        # sat.addServiceChannel(serviceAutoName)
        sat.addStreamingChannel(
            Flow_T.FT_BLOB, Variant_T.T_BYTEARRAY, gestureManualChanName)
        sat.addStreamingChannel(
            Flow_T.FT_BLOB, Variant_T.T_BYTEARRAY, gestureModeChanName)

    return ok


def loop():

    if gestureManualChan and gestureModeChan:
        if (chronoImage.stop() > 5):
       
            current_operation = Command(random.randint(0, 4))
            controller.publish_manual_operation(current_operation)
           

            chronoImage.start()

    sat.tick()
    return sat.isConnected()


def close():
    # Rilascia la risorsa della videocamera e chiude tutte le finestre
    device.release()
    cv2.destroyAllWindows()

###############################################################################
# CALLBACKs


def onChannelAdded(ch: FlowChannel):
    global gestureManualChan
    global gestureModeChan



    if (ch.name == f"guest.{gestureManualChanName}"):
        print("Channel ADDED: {}".format(ch.name))
        gestureManualChan = ch
    elif (ch.name == f"guest.{gestureModeChanName}"):
        print("Channel ADDED: {}".format(ch.name))
        gestureModeChan = ch
        controller.change_mode()
   
  


def onChannelRemoved(ch):
    print("Channel REMOVED: {}".format(ch.name))


def onStartChanPub(ch):
    print("Publish START required: {}".format(ch.name))


def onStopChanPub(ch):
    print("Publish STOP required: {}".format(ch.name))


def onDataGrabbed(chanID, data):
    pass


###############################################################################
