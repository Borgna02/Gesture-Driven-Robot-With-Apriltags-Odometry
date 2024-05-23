#!PySketch/sketch-executor.py

####################################################
#SKETCH

from PySketch.abstractflow import FlowChannel
from PySketch.flowproto import FlowChanID
from PySketch.flowsat import FlowSat

import argparse

sat = None
serviceName = ""

args = None

parser = argparse.ArgumentParser(description="Send a service channel request and obtain an answer")
parser.add_argument('sketchfile', help='Sketch program file')
parser.add_argument('--user', help='Flow-network username', default='guest')
parser.add_argument('--password', help='Flow-network password', default='password')
parser.add_argument('--service-chan', help='Flow-network service', default='User1.ServiceTestOnClient')

def setup() -> bool:
    global sat
    global args
    global serviceName

    args = parser.parse_args()
    
    sat = FlowSat() 
    sat._userName = args.user
    sat._passwd = args.password
    
    if not sat.connect():
        return False
    
    serviceName = args.service_chan

    sat.setNewChanCallBack(onChannelAdded)
    sat.setResponseCallBack(onServiceResponse)

    return True

def loop() -> bool:
    global sat

    sat.tick()
    return sat.isConnected()

####################################################
#CALLBACKs

def onChannelAdded(ch: FlowChannel):
    global sat
    global serviceName

    if ch.name == serviceName:
        val = {"test3" : "a string", "test4": True}

        # THIS IS A NON-BLOCKING (asychronous) CALL
        sat.sendServiceRequest(ch.chanID, "testCmd", val)

def onServiceResponse(val):
    print("Service response RECEIVED: {}".format(val))
    
####################################################