#!/usr/local/bin/pysketch-executor
###############################################################################
##
## Copyright (C) Daniele Di Ottavio (aka Tetsuo)
## Contact: tetsuo.tek (at) gmail (dot) com
##
## This program is free software; you can redistribute it and/or
## modify it under the terms of the GNU General Public License
## as published by the Free Software Foundation; either version 3
## of the License, or (at your option) any later version.
##
## This program is distributed in the hope that it will be useful,
## but WITHOUT ANY WARRANTY; without even the implied warranty of
## MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
## GNU General Public License for more details.
##
## You should have received a copy of the GNU General Public License
## along with this program; if not, write to the Free Software
## Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA  02111-1307, USA.
##
###############################################################################

import os
import random
import struct
from PySketch.abstractflow  import FlowChannel
from PySketch.flowsync      import FlowSync
from PySketch.flowproto     import FlowChanID, Variant_T, Flow_T
from PySketch.flowsat       import FlowSat
from PySketch.log           import msg, dbg, wrn, err, cri, printPair
from PySketch.elapsedtime   import ElapsedTime

###############################################################################

import argparse
from coppeliasim_zmqremoteapi_client import RemoteAPIClient

###############################################################################
# GLOBALS

TICK_LEN = float(os.getenv('TICK_LEN', 0.010))
senseChanName = "sense"
senseChan = None
controller = None
COPPELIASIM_API = os.getenv("COPPELIASIM_API")


###############################################################################
# CLASSES

class SenseController:

    def __init__(self):
        self._sensors = ["ultrasonicSensor[1]",
           "ultrasonicSensor[2]",
           "ultrasonicSensor[3]",
           "ultrasonicSensor[4]",
           "ultrasonicSensor[5]"]

        print("Connecting to simulator...")
        self._cSim_client = RemoteAPIClient(host=COPPELIASIM_API)
        self._sim = self._cSim_client.getObject('sim')
        print("Connected to SIM")
        self._sensors_handles = [self._sim.getObject(
            "./"+sensor) for sensor in self._sensors]

        self._sim.startSimulation()

        self._old_sense = ""


    def sense(self):
        front_sensor_dists = []
        for handle in self._sensors_handles:  # ottengo le distanze dai sensori e filtro eventuali zeri
            dist = self._sim.readProximitySensor(handle)[1]
            if (dist == 0):
                dist = 100
            front_sensor_dists.append(round(dist, 3))



        return front_sensor_dists


###############################################################################
# SKETCH

sat = FlowSat()
chronoSensePub = ElapsedTime()


def setup():
    wrn("[SETUP] ..")
    
    global controller
    controller = SenseController()

    parser = argparse.ArgumentParser(description="Sense")
    parser.add_argument('sketchfile', help='Sketch program file')
    parser.add_argument('--user', help='Flow-network username', default='guest')
    parser.add_argument('--password', help='Flow-network password', default='password')
    args = parser.parse_args()
    
    sat.setLogin(args.user, args.password)
    sat.setNodeName("Sense")


    t = TICK_LEN # seconds
    sat.setTickTimer(t, t * 50)

    sat.setNewChanCallBack(onChannelAdded)
    sat.setDelChanCallBack(onChannelRemoved)
    
    sat.setStartChanPubReqCallBack(onStartChanPub)
    sat.setStopChanPubReqCallBack(onStopChanPub)

    sat.setGrabDataCallBack(onDataGrabbed)

    ok = sat.connect() # uses the env-var ROBOT_ADDRESS

    if ok:
        sat.setSpeedMonitorEnabled(True)
        wrn("[LOOP] ..")
        sat.addStreamingChannel(
            Flow_T.FT_BLOB, Variant_T.T_BYTEARRAY, senseChanName, "")
    
    return ok

def loop():
    try:
        if controller._sim.getSimulationState() == 0:
            msg("Simulation non avviata, avvio...")
            controller._sim.startSimulation()
    except Exception as e:
        err("Errore nel controllo dell'avvio della simulazione: " + str(e))
    
    if (senseChan):
        if (chronoSensePub.stop() > 0.3):
            new_sense = controller.sense()
            if controller._old_sense != new_sense:
                msg(new_sense)
                data = struct.pack("<fffff", *new_sense)
                sat.publish(senseChan.chanID, data)
                controller._old_sense = new_sense
            chronoSensePub.start()
    else:
        err("No sense channel")
            
    sat.tick()
    return sat.isConnected()

###############################################################################
# CALLBACKs

def onChannelAdded(ch):
    global senseChan
    if (ch.name == f"guest.{senseChanName}"):
        msg("Channel ADDED: {}".format(ch.name))
        senseChan = ch


def onChannelRemoved(ch):
    msg("Channel REMOVED: {}".format(ch.name))

def onStartChanPub(ch):
    msg("Publish START required: {}".format(ch.name))

def onStopChanPub(ch):
    msg("Publish STOP required: {}".format(ch.name))

def onDataGrabbed(chanID, data):
    pass
    
###############################################################################
