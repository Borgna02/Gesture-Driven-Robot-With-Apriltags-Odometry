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
import struct

from coppeliasim_zmqremoteapi_client import RemoteAPIClient
from PySketch.abstractflow import FlowChannel
from PySketch.flowsync import FlowSync
from PySketch.flowproto import FlowChanID, Variant_T, Flow_T
from PySketch.flowsat import FlowSat
from PySketch.databuffer import DataBuffer
from PySketch.elapsedtime import ElapsedTime

###############################################################################

from enum import Enum
import time
import argparse
import numpy as np
from timing import TICK_LEN


###############################################################################
# SKETCH
sat = FlowSat()
timer = sat._timer

chronoPositionUpdate = ElapsedTime()

# Canali da gesture

# gestureAutoChanName = "gesture_auto"
# gestureAutoChan = None

serviceAutoName = "gesture_auto"
serviceAuto = None

gestureManualChanName = "gesture_manual"
gestureManualChan = None

gestureModeChanName = "gesture_mode"
gestureModeChan = None

# Canali verso gesture

gestureConfirmChanName = "gesture_confirm"
gestureConfirmChan = None

gesturePositionChanName = "gesture_position"
gesturePositionChan = None

# Canale perceptions

perceptionChanName = "perception"
perceptionChan = None

# Canale actions

actionChanName = "action"
actionChan = None

# Canale distYawPhi

posOrientChanName = "position_orientation"
posOrientChan = None


controller = None


###############################################################################
# CLASSES


class Mode(Enum):
    MANUAL = 1
    AUTO = 2


class Command(Enum):
    LEFT = 0
    FRONTLEFT = 1
    FRONT = 2
    FRONTRIGHT = 3
    RIGHT = 4
    STOP = -1


class Constants:
    MY_SIM_HOST = "localhost"
    CLOSE_ENOUGH_THRESHOLD = 0.05  # m


class Controller:

    def __init__(self):
        self._mode = Mode.MANUAL
        self._free_spaces = dict()
        self._last_action = ""
        self._last_received_auto_cmnd = None

        # se diversa da "", allora significa che sto in una sessione di obstacle avoidance
        self._last_avoiding_command = ""

        # Invio della prima operazione all'avvio (stop)
        self._last_command = Command.STOP
        self.exec_command(Command.STOP)

        # Recupero degli handler dal simulatore
        self.connect_to_sim()

        # Recupero le posizioni dei targets dal simulatore
        self._targets = dict()
        self.get_targets()

        # Invio della prima operazione all'avvio (stop)
        self._last_command = Command.STOP
        self.exec_command(Command.STOP)
        
        self._response_sent = False

    def connect_to_sim(self):
        print("Connecting to simulator...", flush=True)
        client = RemoteAPIClient(host=Constants.MY_SIM_HOST)
        self._sim = client.require('sim')
        print('Connected', flush=True)
        self._robot_handler = self._sim.getObject("./PioneerP3DX")

    def update_pos_and_orient(self, pos=None, orient=None, pos_real=None, orient_real=None):
        # Imposto i valori calcolati come posizione e orientamento
        self._my_pos = pos
        self._my_orientation = orient

        # I valori reali (recuperati dal simulatore) li invio al modulo gesture solo per stamparli sull'interfaccia
        if (gesturePositionChan):
            data = DataBuffer()
            data.setFloat(
                [*self._my_pos, self._my_orientation, *pos_real, orient_real])
            sat.publish(gesturePositionChan.chanID, data)
            
# TODO capire perché non va la guida automatica
    def get_targets(self):
        for i in range(1, 6):
            # Ottengo la posizione (in metri) degli obiettivi relativa all'origine della scena
            x, y, _ = self._sim.getObjectPosition(
                self._sim.getObject("./Disc"+str(i)), -1)
            # in alcuni casi la coordinata 0 non viene restituita come 0 ma come un numero infinitamente piccolo
            x = 0 if x < 0.00001 and x > -0.0001 else x
            y = 0 if y < 0.00001 and y > -0.0001 else y
            self._targets[str(i)] = x, y

    # ricava l'azione dall'ultimo comando dato e la pubblica se è diversa da quella attuale

    def exec_command(self, command):

        # pubblicazione del messaggio solo se la nuova azione è diversa da quella attuale
        if (self._last_action != command and actionChan):
            print("Azione eseguita: " + command.name, flush=True)
            sat.publishInt8(actionChan.chanID, command.value)
            self._last_action = command

    def check_free_spaces(self, cmnd1: Command, cmnd2: Command, cmnd3: Command, cmnd4: Command):
        ##
        # In questa funzione vengono passati i comandi nell'ordine in cui voglio che siano valutati
        # Ogni comando ha un ordine diverso, per questo motivo è necessario parametrizzare.
        # Se il comando è front, allora controllo frontLeft, frontRight, left e right
        # Se il comando è frontLeft, allora controllo Left, front, frontRight e right (speculare per frontRight)
        # Se il comando è left, allora controllo frontLeft, front, frontRight e right (speculare per right)
        ##

        sorted_commands = [cmnd1, cmnd2, cmnd3, cmnd4]
        for cmnd in sorted_commands:
            if (self._free_spaces[cmnd]):
                self._last_avoiding_command = cmnd
                return cmnd
        return Command.STOP

    def avoid_obstacles(self):

        # L'obstacle avoidance non viene fatta se l'ultimo comando è STOP
        if (self._last_command == Command.STOP):
            return self._last_command

        # L'obstacle avoidance non viene fatta se gli spazi vuoti non sono inizializzati
        if (not self._free_spaces):
            return self._last_command

        # L'obstacle avoidance non viene fatta se la direzione in cui sto andando è libera
        if (self._free_spaces[self._last_command]):
            self._last_avoiding_command = ""
            return self._last_command

        command = self._last_command
        # Se l'avoiding command è diverso da "", vuol dire che mi trovo nella stessa "sessione" di obstacle avoidance, quindi se
        # quella direzione è libera continuo a seguirla
        if self._last_avoiding_command != "":
            if self._free_spaces[self._last_avoiding_command]:
                return self._last_avoiding_command

            command = self._last_avoiding_command

        # Se arrivo qui vuol dire che command è occupato
        match command:
            case Command.FRONT:
                return self.check_free_spaces(Command.FRONTLEFT, Command.FRONTRIGHT, Command.LEFT, Command.RIGHT)

            case Command.FRONTLEFT:
                if (self._mode == Mode.MANUAL):
                    return self.check_free_spaces(Command.LEFT, Command.FRONT, Command.FRONTRIGHT, Command.RIGHT)
                return self.check_free_spaces(Command.FRONT, Command.FRONTRIGHT, Command.RIGHT, Command.LEFT)

            case Command.FRONTRIGHT:
                if (self._mode == Mode.MANUAL):
                    return self.check_free_spaces(Command.RIGHT, Command.FRONT, Command.FRONTLEFT, Command.LEFT)
                return self.check_free_spaces(Command.FRONT, Command.FRONTLEFT, Command.LEFT, Command.RIGHT)

            case Command.LEFT:
                return self.check_free_spaces(Command.FRONTLEFT, Command.FRONT, Command.FRONTRIGHT, Command.RIGHT)

            case Command.RIGHT:
                return self.check_free_spaces(Command.FRONTRIGHT, Command.FRONT, Command.FRONTLEFT, Command.LEFT)

    def get_dir_to_target(self, pos_to_reach):
        x_to_reach, y_to_reach = pos_to_reach
        my_x, my_y = self._my_pos

        # L'angolo da - FRONT_ANGLE a FRONT_ANGLE viene considerato come dritto
        FRONT_ANGLE = 5
        # L'angolo da - FRONT_SIDE_ANGLE a FRONT_SIDE_ANGLE viene considerato come quasi dritto, quindi il robot utilizzerà le operazioni frontSide (frontLeft, frontRight)
        FRONT_SIDE_ANGLE = 60

        # Calcolo la direzione tra la posizione del robot e il target
        direction_to_reach = math.degrees(
            math.atan2(y_to_reach - my_y, x_to_reach - my_x))

        # aggiungiamo 540 perché in python il modulo di un numero negativo non funziona come vorremmo
        variation = (direction_to_reach -
                     self._my_orientation + 540) % 360 - 180

        # In base all'ampiezza della variazione decido il comando da eseguire
        assert -180 < variation <= 180

        if variation >= FRONT_SIDE_ANGLE:
            self._last_command = Command.LEFT
        elif variation > FRONT_ANGLE and variation < FRONT_SIDE_ANGLE:
            self._last_command = Command.FRONTLEFT
        elif variation < -FRONT_ANGLE and variation > -FRONT_SIDE_ANGLE:
            self._last_command = Command.FRONTRIGHT
        elif variation <= -FRONT_SIDE_ANGLE:
            self._last_command = Command.RIGHT
        else:
            # Consideriamo la direzione dritta anche se c'è un piccolo scarto di +-5 gradi
            self._last_command = Command.FRONT

    def close_enough(self, pos_to_reach):
        x_to_reach, y_to_reach = pos_to_reach
        my_x, my_y = self._my_pos
        # Controllo che la distanza sia minore del threashold
        return (x_to_reach - my_x) ** 2 + (y_to_reach - my_y) ** 2 <= Constants.CLOSE_ENOUGH_THRESHOLD

    def closer_than_obstacle(self, pos_to_reach):
        OBSTACLE_DISTANCE = 0.6
        x_to_reach, y_to_reach = pos_to_reach
        my_x, my_y = self._my_pos
        return (x_to_reach - my_x) ** 2 + (y_to_reach - my_y) ** 2 <= OBSTACLE_DISTANCE

    def handle_manual_cmnd(self, decoded_msg: str):
        # Converto il comando da stringa a Enum
        decoded_msg = Command(int(decoded_msg))

        # Imposto last_command come il comando dato e resetto last_avoiding_command in modo da interrompere un'eventuale sessione di avoiding
        self._last_command = decoded_msg
        self._last_avoiding_command = ""

        # Verifico se bisogna fare eventuali avoiding
        command = self.avoid_obstacles()

        # Eseguo il comando calcolato
        self.exec_command(command)

    def handle_auto_cmnd(self, decoded_msg: str):
        
        if (decoded_msg == Command.STOP.value):
            self._last_command = Command.STOP
            self.exec_command(Command.STOP)

        else:
            # Recupero dal dizionario la posizione da raggiungere
            pos_to_reach = self._targets[str(decoded_msg)]

            # Se mi sono avvicinato abbastanza all'obiettivo, mi fermo
            if (self.close_enough(pos_to_reach)):

                self._last_command = Command.STOP
                self.exec_command(Command.STOP)

                if(not self._response_sent):
                    sat.sendServiceResponse(serviceAuto.chanID, last_hash, "reached")
                    self._response_sent = True 
            else:
                self._response_sent = False
                # Imposta last command come la direzione migliore per avvicinarsi al target
                self.get_dir_to_target(
                    pos_to_reach)

                # Su tale direzione eseguo l'obstacle avoidance
                command = self.avoid_obstacles()

                # Eseguo il comando così calcolato
                self.exec_command(
                    command)

    def handle_perceptions(self, new_perceptions : dict):
        # Aggiorno i freespaces

        # Dato che il json.loads converte le chiavi in stringhe, le trasformo in Command
        for direction, value in new_perceptions.items():
            self._free_spaces[direction] = value

        # Verifico se con le nuove perceptions c'è bisogno di evitare un ostacolo
        command = self.avoid_obstacles()

        # Eseguo l'operazione
        self.exec_command(command)

    def change_mode(self, decoded_msg: str):
        self._mode = Mode(decoded_msg)
        # print("Cambio modalità in " + str(self._mode.name), flush=True)




###############################################################################
# SKETCH


def setup():
    print("[SETUP] ..")

    parser = argparse.ArgumentParser(description="Nao publisher")
    parser.add_argument('sketchfile', help='Sketch program file')
    parser.add_argument(
        '--user', help='Flow-network username', default='guest')
    parser.add_argument(
        '--password', help='Flow-network password', default='password')
    args = parser.parse_args()

    sat.setLogin(args.user, args.password)
    sat.setAppName("Controller")

    t = TICK_LEN
    sat.setTickTimer(t, t * 50)

    sat.setNewChanCallBack(onChannelAdded)
    sat.setDelChanCallBack(onChannelRemoved)

    sat.setStartChanPubReqCallBack(onStartChanPub)
    sat.setStopChanPubReqCallBack(onStopChanPub)

    sat.setGrabDataCallBack(onDataGrabbed)

    sat.setRequestCallBack(onServiceRequest)
    
    ok = sat.connect()  # uses the env-var ROBOT_ADDRESS

    if ok:
        # Per non mostrare il monitor degli errori
        sat.setSpeedMonitorEnabled(True)

        print("[LOOP] ..")
        sat.addStreamingChannel(
            Flow_T.FT_BLOB, Variant_T.T_BYTEARRAY, gestureConfirmChanName)
        sat.addStreamingChannel(
            Flow_T.FT_BLOB, Variant_T.T_BYTEARRAY, gesturePositionChanName)
        sat.addStreamingChannel(
            Flow_T.FT_BLOB, Variant_T.T_BYTEARRAY, actionChanName)
        if not sat.addServiceChannel(serviceAutoName):
            print("Error adding service channel")
            return False

    global controller
    controller = Controller()

    return ok


def loop():
    # if (chronoPositionUpdate.stop() > 1):
    #     controller.update_pos_and_orient()
    #     chronoPositionUpdate.start()

    if (controller._mode == Mode.AUTO and controller._last_received_auto_cmnd):
        controller.handle_auto_cmnd(controller._last_received_auto_cmnd)

    sat.tick()
    return sat.isConnected()

###############################################################################
# CALLBACKs


def onChannelAdded(ch: FlowChannel):
    global serviceAuto
    global gestureManualChan
    global gestureModeChan
    global gestureConfirmChan
    global gesturePositionChan
    global perceptionChan
    global actionChan
    global posOrientChan

    if (ch.name == f"guest.{serviceAutoName}"):
        serviceAuto = ch
        print("ServiceChannel is READY: {}".format(serviceAuto.name))

    elif (ch.name == f"guest.{gestureManualChanName}"):
        print("Channel ADDED: {}".format(ch.name))
        gestureManualChan = ch
        sat.subscribeChannel(gestureManualChan.chanID)

    elif (ch.name == f"guest.{gestureModeChanName}"):
        print("Channel ADDED: {}".format(ch.name))
        gestureModeChan = ch
        sat.subscribeChannel(gestureModeChan.chanID)
        
        sync = sat.newSyncClient()
        sync.setCurrentDbName(gestureModeChan.name)
        print("Var", sync.getVariable("mode"))
        controller._mode = Mode(int(sync.getVariable("mode")))
        sync.close()

    elif (ch.name == f"guest.{gestureConfirmChanName}"):
        print("Channel ADDED: {}".format(ch.name))
        gestureConfirmChan = ch

    elif (ch.name == f"guest.{gesturePositionChanName}"):
        print("Channel ADDED: {}".format(ch.name))
        gesturePositionChan = ch

    elif (ch.name == f"guest.{actionChanName}"):
        print("Channel ADDED: {}".format(ch.name))
        actionChan = ch

    elif (ch.name == f"guest.{perceptionChanName}"):
        print("Channel ADDED: {}".format(ch.name))
        perceptionChan = ch
        sat.subscribeChannel(perceptionChan.chanID)
        
    elif (ch.name == f"guest.{posOrientChanName}"):
        print("Channel ADDED: {}".format(ch.name))
        posOrientChan = ch
        sat.subscribeChannel(posOrientChan.chanID)


def onChannelRemoved(ch):
    print("Channel REMOVED: {}".format(ch.name))


def onStartChanPub(ch):
    print("Publish START required: {}".format(ch.name))


def onStopChanPub(ch):
    print("Publish STOP required: {}".format(ch.name))


def onDataGrabbed(chanID, data):
    
    if (gestureModeChan and chanID == gestureModeChan.chanID):
        data, = struct.unpack('<b', data)
        controller.change_mode(data)

    elif (gestureManualChan and chanID == gestureManualChan.chanID):
        data, = struct.unpack('<b', data)
        print("Command: ", Command(data).name)
        controller.handle_manual_cmnd(data)

    elif (perceptionChan and chanID == perceptionChan.chanID):
        # Estrarre i cinque bit più a destra dall'ultimo (unico) byte
        last_five_bits = int(data[-1]) & 0b11111

        # Convertire i bit in una lista di True e False
        bits = [(last_five_bits >> i) & 1 for i in range(4, -1, -1)]
        bits = [True if bit == 1 else False for bit in bits]

        new_perceptions = {Command.LEFT: bits[0], Command.FRONTLEFT: bits[1],
                           Command.FRONT: bits[2], Command.FRONTRIGHT: bits[3], Command.RIGHT: bits[4]}
        controller.handle_perceptions(new_perceptions)
    elif (posOrientChan and chanID == posOrientChan.chanID):
        x, y, orientation, real_x, real_y, real_orientation = struct.unpack('<ffffff', data)
        controller.update_pos_and_orient((x, y), orientation, (real_x, real_y), real_orientation)
    


last_hash = ""
def onServiceRequest(chanID: FlowChanID, hash: str, cmdName: str, val):
    # Sat could build more service-channels
    if serviceAuto.chanID == chanID:
        
        global last_hash
        last_hash = hash
        print("ServiceChannel request RECEIVED [cmdName: {}; hash: {}]: {}".format(cmdName, hash, val))

        controller._last_received_auto_cmnd = val['value']

###############################################################################
