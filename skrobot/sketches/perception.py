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
import struct
from PySketch.abstractflow import FlowChannel
from PySketch.flowsync import FlowSync
from PySketch.flowproto import FlowChanID, Variant_T, Flow_T
from PySketch.flowsat import FlowSat

###############################################################################

from enum import Enum
import time
import argparse
import numpy as np
from timing import TICK_LEN

###############################################################################
# CLASSES


LONG_DISTANCE = 0.5
MEDIUM_DISTANCE = 0.3
SHORT_DISTANCE = 0.15


class Directions(Enum):
    LEFT = 0
    FRONTLEFT = 1
    FRONT = 2
    FRONTRIGHT = 3
    RIGHT = 4
    STOP = -1


class PerceptionsController:

    def __init__(self):
        self._sensors = [Directions.LEFT, Directions.FRONTLEFT,
                         Directions.FRONT, Directions.FRONTRIGHT, Directions.RIGHT]

        self._old_perceptions = {sensor.name: True for sensor in self._sensors}

    def percept(self, sensor_data: dict):

        # Crea il dizionario values
        values = {sensor: sensor_data[sensor.name] for sensor in self._sensors}

        # Inizializzo le nuove perceptions completamente a True
        new_perceptions = {sensor.name: True for sensor in self._sensors}

        # Se uno dei sensori FRONTALI ha un valore minore di LONG_DISTANCE, blocco solo quella direzione
        for sensor in self._sensors:
            if sensor == Directions.LEFT or sensor == Directions.RIGHT:
                continue
            if values[sensor] < LONG_DISTANCE:
                new_perceptions[sensor.name] = False

        # Se uno dei sensori ha un valore minore di MEDIUM_DISTANCE, blocco quella direzione e le due adiacenti (se esistono entrambe)
        for sensor in self._sensors:
            if (values[sensor] < MEDIUM_DISTANCE):
                new_perceptions[sensor.name] = False
                if (sensor.value - 1 >= 0):
                    new_perceptions[Directions(sensor.value - 1).name] = False
                if (sensor.value + 1 <= 4):
                    new_perceptions[Directions(sensor.value + 1).name] = False

        # Se uno dei sensori ha un valore minore di SHORT_DISTANCE, blocco tutte le direzioni frontali. Se il sensore in questione è diverso da FRONT, blocco anche la direzione laterale (LEFT, RIGHT) più vicina
        for sensor in self._sensors:
            if values[sensor] < SHORT_DISTANCE:

                new_perceptions[Directions.FRONTLEFT.name] = False
                new_perceptions[Directions.FRONT.name] = False
                new_perceptions[Directions.FRONTRIGHT.name] = False

                if sensor == Directions.LEFT or sensor == Directions.FRONTLEFT:
                    new_perceptions[Directions.LEFT.name] = False
                if sensor == Directions.RIGHT or sensor == Directions.FRONTRIGHT:
                    new_perceptions[Directions.RIGHT.name] = False
             
        # Se tutte le perceptions sono False, sblocco la direzione LEFT perché altrimenti il robot rimarrebbe bloccato    
        if all(value == False for value in new_perceptions.values()):
            new_perceptions[Directions.LEFT.name] = True

        return new_perceptions


###############################################################################
# SKETCH
sat = FlowSat()
timer = sat._timer

senseChanName = "sense"
senseChan = None

perceptionChanName = "perception"
perceptionChan = None

controller = None


def setup():
    print("[SETUP] ..")

    global controller
    controller = PerceptionsController()

    parser = argparse.ArgumentParser(description="Nao publisher")
    parser.add_argument('sketchfile', help='Sketch program file')
    parser.add_argument(
        '--user', help='Flow-network username', default='guest')
    parser.add_argument(
        '--password', help='Flow-network password', default='password')
    args = parser.parse_args()

    sat.setLogin(args.user, args.password)
    sat.setAppName("Perception")

    t = TICK_LEN
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
            Flow_T.FT_BLOB, Variant_T.T_BYTEARRAY, perceptionChanName)

    return ok


def loop():

    sat.tick()
    return sat.isConnected()

###############################################################################
# CALLBACKs


def onChannelAdded(ch):
    global perceptionChan
    global senseChan

    if (ch.name == f"guest.{perceptionChanName}"):
        print("Channel ADDED: {}".format(ch.name))
        perceptionChan = ch
    elif (ch.name == f"guest.{senseChanName}"):
        print("Channel SUBSCRIBED: {}".format(ch.name))
        senseChan = ch
        sat.subscribeChannel(senseChan.chanID)


def onChannelRemoved(ch):
    print("Channel REMOVED: {}".format(ch.name))


def onStartChanPub(ch):
    print("Publish START required: {}".format(ch.name))


def onStopChanPub(ch):
    print("Publish STOP required: {}".format(ch.name))


def onDataGrabbed(chanID, data):

    if (chanID == senseChan.chanID):

        # Spacchetta la stringa JSON
        sensor_data = struct.unpack("<fffff", data)
        sensor_data = {sensor.name: value for sensor,
                       value in zip(controller._sensors, sensor_data)}

        # Calcolo le perceptions basandomi sui valori dei sensori
        new_perceptions = controller.percept(sensor_data)

        if (perceptionChan and controller._old_perceptions != new_perceptions):

            print(new_perceptions)
            bits = [1 if val == True else 0 for val in new_perceptions.values()]

            byte = 0

            # Gli ultimi cinque bit del byte rappresentano le perceptions (1 = Libero, 0 = Occupato)
            for bit in bits:
                byte = (byte << 1) | bit

            sat.publishInt8(perceptionChan.chanID, byte)
            controller._old_perceptions = new_perceptions

###############################################################################
