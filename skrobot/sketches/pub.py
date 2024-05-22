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

from PySketch.abstractflow import FlowChannel
from PySketch.flowsync import FlowSync
from PySketch.flowproto import FlowChanID, Variant_T, Flow_T
from PySketch.flowsat import FlowSat

###############################################################################

import time
import argparse
import numpy as np
from timing import TICK_LEN

###############################################################################
# SKETCH

sat = FlowSat()
timer = sat._timer

chanName = "CHN"
chan = None

def setup():
    print("[SETUP] ..")

    parser = argparse.ArgumentParser(description="Nao publisher")
    parser.add_argument('sketchfile', help='Sketch program file')
    parser.add_argument('--user', help='Flow-network username', default='guest')
    parser.add_argument('--password', help='Flow-network password', default='password')
    args = parser.parse_args()
    
    sat.setLogin(args.user, args.password)

    t = TICK_LEN
    sat.setTickTimer(t, t * 50)

    sat.setNewChanCallBack(onChannelAdded)
    sat.setDelChanCallBack(onChannelRemoved)
    
    sat.setStartChanPubReqCallBack(onStartChanPub)
    sat.setStopChanPubReqCallBack(onStopChanPub)

    sat.setGrabDataCallBack(onDataGrabbed)

    ok = sat.connect() # uses the env-var ROBOT_ADDRESS

    if ok:
        sat.setSpeedMonitorEnabled(False) # Per non mostrare il monitor degli errori
        

        print("[LOOP] ..")
        sat.addStreamingChannel(Flow_T.FT_BLOB, Variant_T.T_BYTEARRAY, chanName)
    
    return ok

i = 0
j = 0
def loop():
    global i
    global j
    if chan and i % 100 == 0:
        if (j % 2 == 0):
           sat.publishInt8(chan.chanID, 0)
        else: 
           sat.publishInt8(chan.chanID, 4)
        
        j += 1
        
            
    i += 1
    
    sat.tick()
    return sat.isConnected()

###############################################################################
# CALLBACKs

def onChannelAdded(ch):
    global chan
    if (ch.name == "guest.CHN"):
        print("Channel ADDED: {}".format(ch.name))  
        chan = ch 

def onChannelRemoved(ch):
    print("Channel REMOVED: {}".format(ch.name))

def onStartChanPub(ch):
    print("Publish START required: {}".format(ch.name))

def onStopChanPub(ch):
    print("Publish STOP required: {}".format(ch.name))

def onDataGrabbed(chanID, data):
    pass
    
###############################################################################
