#!PySketch/sketch-executor.py

####################################################
#SKETCH
from PySketch.abstractflow import FlowChannel
from PySketch.flowproto import FlowChanID
from PySketch.flowsat import FlowSat

import argparse

sat = None
service = None
serviceGlobalName = ""

args = None

parser = argparse.ArgumentParser(description="Build a service channel and accepting commands (only echo for test)")
parser.add_argument('sketchfile', help='Sketch program file')
parser.add_argument('--user', help='Flow-network username', default='User1')
parser.add_argument('--password', help='Flow-network password', default='password')

def setup() -> bool:
    global sat
    global args
    global serviceGlobalName

    args = parser.parse_args()

    sat = FlowSat()
    sat._userName = args.user
    sat._passwd = args.password
    
    if not sat.connect():
        return False
    
    sat.setNewChanCallBack(onChannelAdded)
    sat.setRequestCallBack(onServiceRequest)

    serviceName = "ServiceTestOnClient"

    if not sat.addServiceChannel("ServiceTestOnClient"):
        return False

    serviceGlobalName = "{}.ServiceTestOnClient".format(sat._userName)
    return True

def loop() -> bool:
    global sat

    sat.tick()
    return sat.isConnected()

####################################################
#CALLBACKs

def onChannelAdded(ch: FlowChannel):
    global sat
    global service
    global serviceGlobalName

    if ch.name == serviceGlobalName:
        service = ch
        print("ServiceChannel is READY: {}".format(service.name))

def onServiceRequest(chanID: FlowChanID, hash: str, cmdName: str, val):
    global sat
    global service

    # Sat could build more service-channels
    if service.chanID == chanID:
        #data echo test
        print("ServiceChannel request RECEIVED [cmdName: {}; hash: {}]: {}".format(cmdName, hash, val))
        sat.sendServiceResponse(chanID, hash, val)

    
    
####################################################