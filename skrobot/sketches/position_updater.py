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



###############################################################################
# CLASSES



ROBOT_RADIUS = 0.2

class TagHandler:

    def __init__(self):
        self._arena_size = (4,4) # width, height (meters)
        self._space_between_tags = 0.8 
        
        # Inizializzo il dizionario dei tag
        self._tags = []
        # Itero sulle righe
        i = -self._arena_size[0]/2
        while i <= self._arena_size[0]/2:
            
            # Itero sulle colonne
            j = -self._arena_size[1]/2
            while j <= self._arena_size[1]/2:
                self._tags.append({
                    'x': j,
                    'y': i,
                    'dist': 0,
                    'yaw': 0,
                    'phi': 0,
                    'is_visible': False
                })
                j += self._space_between_tags
            i += self._space_between_tags
            
        # print(self._tags)

    

       

    def update_tags(self, id, dist, yaw, phi):
        
        # Se non vedo nessun tag, id è -1 e quindi nessun tag viene posto come visibile
        new_tags_list = []
        for i, tag in enumerate(self._tags):
            new_tag = {'x': tag['x'], 'y': tag['y'], 'dist': None,
                       'yaw': None, 'phi': None, 'is_visible': False}
            if i == id:
                new_tag['dist'] = dist
                new_tag['yaw'] = yaw
                new_tag['phi'] = phi
                new_tag['is_visible'] = True
                
            new_tags_list.append(new_tag)

        self._tags = new_tags_list

    def update_pos_and_orient(self):

        # Restituisce il tag visibile
        visible_tags= [tag for tag in self._tags if tag['is_visible'] == True]
        
        if not visible_tags:
            self._my_pos = None
            self._my_orientation = None
            return
        
        tag = visible_tags[0]

        ##
        # Calcolo posizione della camera
        ##

        # Aggiungo la distanza tra la fotocamera e il centro del robot
        if (tag['dist'] == None):
            return

        dist = round(tag['dist'], 3)
        yaw = round(tag['yaw'], 3)
        camera_x = tag['x']
        camera_y = tag['y']

        # Calcolo dell'angolo vicino all'origine del triangolo rettangolo
        beta = (abs(yaw) % 90)
        if (beta != 0):
            # Calcolo dei cateti
            a = dist * math.cos(math.radians(beta))
            b = math.sqrt(dist ** 2 - a ** 2)
            if (yaw >= -90 and yaw <= 90):
                camera_x += math.copysign(1, yaw) * b
                camera_y += a
            else:
                camera_x += math.copysign(1, yaw) * a
                camera_y -= b
        else:
            # tratto separatamente i casi limite
            if (yaw == 0):
                camera_y += dist
            elif (yaw == 90):
                camera_x += dist
            elif (abs(yaw) == 180):
                camera_y -= dist
            else:
                camera_x -= dist

        camera_x = round(camera_x, 3)
        camera_y = round(camera_y, 3)

        ##
        # Calcolo orientamento
        ##
        my_or = 0
        phi = tag['phi']
        gamma = abs(yaw)
        if (abs(yaw) % 90 != 0):
            if (gamma > 90):
                gamma = 180 - gamma
            theta = 90 - gamma

            if (yaw > -180 and yaw < -90):
                my_or = -theta - phi
            elif (yaw > 90 and yaw < 180):
                my_or = -180 + theta - phi
            elif (yaw > -90 and yaw < 0):
                my_or = theta - phi
            elif (yaw > 0 and yaw < 90):
                my_or = 180 - theta - phi
            else:
                print("Caso errato, verificare il problema", flush=True)
        # tratto separatamente i casi limite
        else:
            if (yaw == 0):
                my_or = 90 - phi
            elif (yaw == 90):
                # a seconda del segno di phi parto da 180 o -180
                my_or = phi/abs(phi) * 180 - phi
            elif (yaw == 180 or yaw == -180):
                my_or = -90 - phi
            elif (yaw == -90):
                my_or = -phi
            else:
                print(
                    "Caso limite errato, verificare il problema", flush=True)

        my_or = round(my_or, 3)
        self._my_orientation = my_or
        
        self._my_pos = (round(camera_x, 3),
                        round(camera_y, 3))



        # TODO Comunico a gesture la nuova posizione in modo da stamparla sull'interfaccia
        # self._mqtt_manager._client.publish(
        #     "/position", dumps({"position": self._my_pos, "orientation": self._my_orientation}))


###############################################################################
# SKETCH

sat = FlowSat()
timer = sat._timer


posOrientChanName = "position_orientation"
posOrientChan = None

distYawPhiChanName = "dist_yaw_phi"
distYawPhiChan = None


handler = None
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

    global handler
    handler = TagHandler()

    global sim
    print("Connecting to simulator...")
    sim = RemoteAPIClient(host="localhost").getObject('sim')
    print("Connected to SIM")

    



    sat.setLogin(args.user, args.password)
    sat.setAppName("Position Updater")

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
            Flow_T.FT_BLOB, Variant_T.T_BYTEARRAY, posOrientChanName)

    return ok


def loop():

    sat.tick()
    return sat.isConnected()

###############################################################################
# CALLBACKs


def onChannelAdded(ch: FlowChannel):

    global posOrientChan
    global distYawPhiChan
    if (ch.name == f"guest.{posOrientChanName}"):
        print("Channel ADDED: {}".format(ch.name))
        posOrientChan = ch

        
    if (ch.name == f"guest.{distYawPhiChanName}"):
        print("Channel ADDED: {}".format(ch.name))
        distYawPhiChan = ch
        sat.subscribeChannel(distYawPhiChan.chanID)


def onChannelRemoved(ch: FlowChannel):
    print("Channel REMOVED: {}".format(ch.name))


def onStartChanPub(ch: FlowChannel):
    print("Publish START required: {}".format(ch.name))


def onStopChanPub(ch: FlowChannel):
    print("Publish STOP required: {}".format(ch.name))


def onDataGrabbed(chanID, data):
    if (chanID == distYawPhiChan.chanID ):
        
        id, dist, yaw, phi = struct.unpack('<Bfff', data)
        dist, yaw, phi = round(dist, 2), round(yaw, 2), round(phi, 2)
        
        
        handler.update_tags(id, dist, yaw, phi)        

        # Con i dati aggiornati posso aggiornare posizione ed orientamento
        handler.update_pos_and_orient()
        
        # Calciolo la distanza tra la posizione calcolata tramite apriltag e la posizione reale nel simulatore
        cam_coords = np.array(
                [*sim.getObjectPosition(sim.getObject("./rgb")), 1])
        
        print(f"Position: {handler._my_pos}, Orientation: {handler._my_orientation}, Error: {round(np.linalg.norm(cam_coords[:2] - handler._my_pos), 2) if handler._my_pos else None}")
###############################################################################
