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
        k = 0 # variabile per l'id
        while i <= self._arena_size[0]/2:
            
            # Itero sulle colonne
            j = -self._arena_size[1]/2
            while j <= self._arena_size[1]/2:
                self._tags.append({
                    'id': k,
                    'x': j,
                    'y': i,
                    'dist': 0,
                    'yaw': 0,
                    'is_visible': False
                })
                j += self._space_between_tags
                k += 1
            i += self._space_between_tags
            
        # print(self._tags)

    

       

    def update_tags(self, id, dist, yaw):
        
        # Se non vedo nessun tag, id è -1 e quindi nessun tag viene posto come visibile
        new_tags_list = []
        for i, tag in enumerate(self._tags):
            new_tag = {'id' : i, 'x': tag['x'], 'y': tag['y'], 'dist': None,
                       'yaw': None, 'is_visible': False}
            if i == id:
                new_tag['dist'] = dist
                new_tag['yaw'] = yaw
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
        # Calcolo orientamento
        ##
        
        self._my_orientation = tag['yaw']

        ##
        # Calcolo posizione della camera
        ##

        # Aggiungo la distanza tra la fotocamera e il centro del robot
        if (tag['dist'] == None):
            return

        dist = round(tag['dist'], 3)
        yaw = round(tag['yaw'], 3)
     
        
        # Estrai le coordinate del punto T
        Tx = tag['x']
        Ty = tag['y']
        
        # Converti l'angolo yaw da gradi a radianti
        yaw = math.radians(yaw)
        
        # Calcola le coordinate del punto C
        x_delta = dist * math.cos(yaw)
        Cx = Tx - x_delta
        y_delta = dist * math.sin(yaw)
        Cy = Ty - y_delta
        
        # Recupero gli x_delta e y_delta reali
        cam_coords = np.array(
                [*sim.getObjectPosition(sim.getObject("./rgb")), 1])

        # Calcola la distanza tra cam_coords e C
        error = np.linalg.norm(cam_coords[:2] - np.array([Cx, Cy]))
        
        # Salva questi dati su due file csv separati, uno per x e uno per y
        with open('error_with_dewarping.csv', mode='a') as file:
            writer = csv.writer(file)
            writer.writerow([Cx, Cy, cam_coords[0], cam_coords[1], error])
            
       
        
        camera_x = round(Cx, 3)
        camera_y = round(Cy, 3)

       
        
        self._my_pos = (round(camera_x, 3),
                        round(camera_y, 3))



        data = struct.pack("<fff", *self._my_pos, self._my_orientation)
        sat.publish(posOrientChan.chanID, data)



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
        
        id, dist, yaw = struct.unpack('<bff', data)
        dist, yaw = round(dist, 2), round(yaw, 2)
        
        
        
        handler.update_tags(id, dist, yaw)        

        # Con i dati aggiornati posso aggiornare posizione ed orientamento
        handler.update_pos_and_orient()
        
        # Calciolo la distanza tra la posizione calcolata tramite apriltag e la posizione reale nel simulatore
        cam_coords = np.array(
                [*sim.getObjectPosition(sim.getObject("./rgb")), 1])
        
        print(f"Position: {handler._my_pos}, Orientation: {handler._my_orientation}, Error: {round(np.linalg.norm(cam_coords[:2] - handler._my_pos), 2) if handler._my_pos else None}")
###############################################################################
