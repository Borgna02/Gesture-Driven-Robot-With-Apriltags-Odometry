#!PySketch/executor.py

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

import struct
from PySketch.abstractflow import FlowChannel
from PySketch.flowsync import FlowSync
from PySketch.flowproto import FlowChanID, Variant_T, Flow_T
from PySketch.flowsat import FlowSat

###############################################################################

import time
import argparse
import numpy as np
from coppeliasim_zmqremoteapi_client import RemoteAPIClient
from enum import Enum
from timing import TICK_LEN

###############################################################################
# CLASSES

class Command(Enum):
    LEFT = 0
    FRONTLEFT = 1
    FRONT = 2
    FRONTRIGHT = 3
    RIGHT = 4
    STOP = -1


class Controller:

    
    def __init__(self, left_motor_name: str, right_motor_name: str):
        
        print("Connecting to simulator...")
        self._cSim_client = RemoteAPIClient(host="localhost")        
        self._sim = self._cSim_client.getObject('sim')
        print("Connected to SIM")
        self._sim.startSimulation()
        
        self._left_motor_handle = self._sim.getObject("./" + left_motor_name)
        self._right_motor_handle = self._sim.getObject("./" + right_motor_name)

    def set_speeds(self, right_speed, left_speed):
        self._sim.setJointTargetVelocity(self._left_motor_handle, left_speed)
        self._sim.setJointTargetVelocity(self._right_motor_handle, right_speed)
      
    def do_action(self, command: Command):
         
        match command:
            case Command.FRONT:
                self.set_speeds(2, 2)
            case Command.FRONTLEFT:
                self.set_speeds(2, 1)
            case Command.FRONTRIGHT:
                self.set_speeds(1, 2)
            case Command.RIGHT:
                self.set_speeds(-0.5, 0.5)
            case Command.LEFT:
                self.set_speeds(0.5, -0.5)
            case Command.STOP:
                self.set_speeds(0, 0)

            case _:
                raise Exception(f"Unknown command {command}")

        print("Azione eseguita: " + command.name, flush=True)

###############################################################################
# SKETCH

sat = FlowSat()
timer = sat._timer

chanName = "action"
chan = None

controller = None

def setup():
    print("[SETUP] ..")
    
    global controller
    controller = Controller("leftMotor", "rightMotor")

    parser = argparse.ArgumentParser(description="Nao publisher")
    parser.add_argument('sketchfile', help='Sketch program file')
    parser.add_argument('--user', help='Flow-network username', default='guest')
    parser.add_argument('--password', help='Flow-network password', default='password')
    args = parser.parse_args()
    
    sat.setLogin(args.user, args.password)
    sat.setAppName("Action")

    t = TICK_LEN
    sat.setTickTimer(t, t * 50)

    sat.setNewChanCallBack(onChannelAdded)
    sat.setDelChanCallBack(onChannelRemoved)
    
    sat.setStartChanPubReqCallBack(onStartChanPub)
    sat.setStopChanPubReqCallBack(onStopChanPub)

    sat.setGrabDataCallBack(onDataGrabbed)

    ok = sat.connect() # uses the env-var ROBOT_ADDRESS

    if ok:
        sat.setSpeedMonitorEnabled(True)
        

        print("[LOOP] ..")
    
    return ok

def loop():
    sat.tick()
    
    return sat.isConnected()

def close():
    global controller
    # Fermo la connessione
    controller._sim.stopSimulation()
    
###############################################################################
# CALLBACKs

def onChannelAdded(ch):
    global chan
        
    if ch.name == "guest.action":
        chan = ch 
        sat.subscribeChannel(chan.chanID)
        controller._sim.startSimulation()
        
def onChannelRemoved(ch):
    print("Channel REMOVED: {}".format(ch.name))

def onStartChanPub(ch):
    print("Publish START required: {}".format(ch.name))

def onStopChanPub(ch):
    print("Publish STOP required: {}".format(ch.name))

def onDataGrabbed(chanID, data):
    global controller
    
    if controller:
        # Per vedere come decodificare il messaggio andare in socketdevice.py
              
        val, = struct.unpack('<b', data)
        print(val)
        controller.do_action(Command(val))
   
###############################################################################

