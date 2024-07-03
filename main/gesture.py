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

import math
import struct

import cv2
from PySketch.abstractflow import FlowChannel
from PySketch.flowproto import Variant_T, Flow_T
from PySketch.flowsat import FlowSat
from PySketch.elapsedtime import ElapsedTime

###############################################################################

from enum import Enum
import argparse
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
        self._current_mode = Mode.MANUAL 

        self._tracker = handTracker()

        self._changing_mode = False
        self._mean_counter = 0
        self._meanPos = 0

        self._last_operation = ""
        self._reached = False
        self._counter = 0
        self._image = None

        self._pos = (0, 0)
        self._orient = 0
        
        self._real_pos = (0, 0)
        self._real_orient = 0

    def get_index_direction(self):

        wirst_x, wirst_y = self._tracker.get_lm_coords(0)
        index_x, index_y = self._tracker.get_lm_coords(8)

        slope = MathUtils.get_slope((wirst_x, wirst_y), (index_x, index_y))

        # Se l'indice si trova in posizione più bassa rispetto al polso, cambio modalità
        if index_y > wirst_y:
            if slope < -2 or slope > 2:
                return -1
            else:
                return Command.STOP

        # Se il coefficiente angolare è compreso tra 1 e -1, differenzio tra sinistra e destra in base a quale x è maggiore tra polso e indice
        if slope < 1 and slope > -1:
            if index_x > wirst_x:
                return Command.RIGHT
            return Command.LEFT

        if slope > 1 and slope < 3:
            return Command.FRONTLEFT

        if slope < -3 or slope > 3:
            return Command.FRONT

        # L'ultimo caso rimasto è slope > -3 and slope < -1
        return Command.FRONTRIGHT

    def get_hand_mean(self):
        ##
        # Essendo l'apertura della mano un'operazione potenzialmente lenta, utilizziamo una media pesata in cui i valori finali hanno un peso più elevato
        # in questo modo si da più importanza ai valori alla fine dell'apertura, che deve impiegare non più di MEAN_DIMENSION/CAM_REFRESH_TIME millisecondi
        ##

        # Caso in cui avevo finito di calcolare la posizione oppure avevo interrotto il calcolo
        if (self._mean_counter == 0 or self._mean_counter == -1):
            self._mean_counter = 1
            self._mean_pos = int(0)
            return Command.STOP

        # Caso in cui sto calcolando la media
        if (self._mean_counter < Constants.MEAN_DIMENSION and self._mean_counter > 0):
            self._mean_pos += self.calculate_number() * self._mean_counter
            self._mean_counter += 1
            return "Calcolando"

        # Caso in cui sono arrivato alla fine del calcolo della media e posso restituire la posizione
        if (self._mean_counter == Constants.MEAN_DIMENSION):
            self._mean_pos = str(
                round(int(self._mean_pos) / Constants.MEAN_DENOM))
            self._mean_counter += 1
            return str(self._mean_pos)

        # Caso in cui ho finito di calcolare la media ma ho ancora la mano aperta (anche se indico un altro numero, viene mantenuto quello calcolato)
        # l'unico modo per ricalcolare un nuovo numero è chiudere la mano
        return str(self._mean_pos)

    def change_mode(self):
        self._changing_mode = True
        if (self._current_mode == Mode.MANUAL):
            self._current_mode = Mode.AUTO
        else:
            self._current_mode = Mode.MANUAL
            self._mean_counter = 0
            self._mean_pos = Command.STOP

        # print("Changing Mode in " + self._current_mode.name)
       
        
        data = struct.pack("<b", self._current_mode.value)
        sat.publish(gestureModeChan.chanID, data)
        
        

        

    def compute_operation(self):

        # Se l'indice si trova sotto al polso, allora cambio modalità
        if (self._tracker.current_lm_list and self.get_index_direction() == -1):
            if (not self._changing_mode):
                self.change_mode()
            return Command.STOP

        else:
            # se "torno" con l'indice sopra al polso cambiare un'altra volta modalità
            if (self._changing_mode):
                self._changing_mode = False

        ##
        # Se sono in modalità manuale
        ##

        if self._current_mode == Mode.MANUAL:
            # Se faccio il pugno in modalità manuale oppure rimuovo le mani dalla finestra il robot si ferma
            if (not self._tracker.current_lm_list or self.calculate_number() == 0):
                return Command.STOP

            # Sennò calcolo e restituisco la direzione
            return self.get_index_direction()

        ##
        # Se sono in modalità automatica
        ##

        # Se chiudo il pugno o rimuovo le mani dalla finestra ho tre possibilità
        if (not self._tracker.current_lm_list or self.calculate_number() == 0):

            # Se ho raggiunto il target e ho la mano chiusa o fuori dall'inquadratura scrivo arrivato
            if (self._reached):
                return "Arrivato"

            # Se torno al pugno chiuso quando ho già finito di calcolare la posizione, continuo a mantenere la posizione calcolata
            if (self._mean_counter == Constants.MEAN_DIMENSION + 1 or self._mean_counter == -1):
                self._mean_counter = -1
                return str(self._mean_pos)

            # Se chiudo la mano prima che la posizione sia stata calcolata, azzero il calcolo
            self._mean_counter = 0
            return Command.STOP

        # Se ho raggiunto il target e ho la mano aperta, reinizializzo il calcolo
        if (self._reached):
            self._mean_counter = 0
            self._reached = False
            return "Arrivato"

        return self.get_hand_mean()

    def calculate_number(self):

        number = 0

        wirst = self._tracker.get_lm_coords(0)

        # l'indice è aperto se la distanza tra la punta e il polso è maggiore della distanza tra la nocca e il polso
        index_tip = self._tracker.get_lm_coords(8)
        index_knuckle = self._tracker.get_lm_coords(6)
        if MathUtils.distance(wirst, index_tip) > MathUtils.distance(wirst, index_knuckle):
            number += 1

        # verifico se il medio è aperto
        middle_tip = self._tracker.get_lm_coords(12)
        middle_knucle = self._tracker.get_lm_coords(10)
        if MathUtils.distance(wirst, middle_tip) > MathUtils.distance(wirst, middle_knucle):
            number += 1

        # verifico se l'anulare è aperto
        ring_tip = self._tracker.get_lm_coords(16)
        ring_knucle = self._tracker.get_lm_coords(14)
        if MathUtils.distance(wirst, ring_tip) > MathUtils.distance(wirst, ring_knucle):
            number += 1

        # verifico se il mignolo è aperto
        pinky_tip = self._tracker.get_lm_coords(20)
        pinky_knucle = self._tracker.get_lm_coords(18)
        if MathUtils.distance(wirst, pinky_tip) > MathUtils.distance(wirst, pinky_knucle):
            number += 1

        # il pollice è aperto se la distanza tra la punta e il landmark 17 è maggiore della distanza tra il landmark 17 e il polso più un certo margine
        # (quest'ultima usata come unità di misura anche se non rappresenta esattamente la distanza del pollice dal centro)
        thumb_tip = self._tracker.get_lm_coords(4)
        lm_17 = self._tracker.get_lm_coords(17)
        if MathUtils.distance(thumb_tip, lm_17) > MathUtils.distance(wirst, lm_17) * 1.1:
            number += 1

        return number

    def publish_manual_operation(self, operation: Command):
        # Invio l'operazione solo se è diversa da quella precedente
        if (self._last_operation != operation):
            # print("New operation: " + str(operation))
            sat.publishInt8(gestureManualChan.chanID,
                            operation.value)
            self._last_operation = operation

    def publish_auto_operation(self, current_operation):
        # Se sono in modalità automatica e l'operazione è diversa da Command.STOP, allora la ripeto all'infinito

        if (current_operation == self._last_operation == Command.STOP):
            # Se l'operazione corrente è STOP e anche l'ultima inviata è STOP, allora non invio nulla
            return
        
        # Se l'operazione è STOP invio il suo valore numerico, altrimenti inv
        if (current_operation != self._last_operation):
            operation_code = current_operation.value if current_operation == Command.STOP else current_operation
            sat.sendServiceRequest(
                serviceAuto.chanID, "cmndAuto", operation_code)
            self._last_operation = current_operation
        


        


class ImageUtils:
    @staticmethod
    def write_on_image(image, string, pos, orientation, mode: Mode, real_pos, real_orient):
        if isinstance(string, Command):
            string = string.name.lower()
            
        if pos != (None,None):
            pos_string = f"Position: {round(pos[0],3)}, {round(pos[1],3)}"
        else: 
            pos_string = f"Position: Nessun tag"
            
        pos_string += f" ({round(real_pos[0], 3)}, {round(real_pos[1], 3)})"
        
        if orientation != None:
            orient_string = f"Orientation: {round(orientation, 2)}"
        else:
            orient_string = f"Orientation: Nessun tag"
        
        orient_string += f" ({round(real_orient, 2)})"
        # Scrive la stringa sull'immagine
        cv2.putText(image, string, (50, 50),
                    cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 0), 4)
        cv2.putText(image, mode.name, (image.shape[1] - 150, 50),
                    cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 0), 4)

        cv2.rectangle(
            image, (48, image.shape[0] - 58), (470, image.shape[0] - 15), (255, 255, 255), -1)

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


controller = None
device = None
is_from_phone = None


def setup():
    print("[SETUP] ..")

    # Inizializzazione della videocamera
    global device
    global is_from_phone

    device = cv2.VideoCapture(0)

    if not device.isOpened():
        print("CANNOT open the camera [ID: {}]".format(0))
        return False

    is_from_phone = False

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
    sat.setAppName("Gesture")

    t = TICK_LEN
    sat.setTickTimer(t, t * 20)  # Intervallo di check

    sat.setNewChanCallBack(onChannelAdded)
    sat.setDelChanCallBack(onChannelRemoved)

    sat.setStartChanPubReqCallBack(onStartChanPub)
    sat.setStopChanPubReqCallBack(onStopChanPub)

    sat.setGrabDataCallBack(onDataGrabbed)

    sat.setResponseCallBack(onServiceResponse)

    ok = sat.connect()  # uses the env-var ROBOT_ADDRESS

    if ok:
        # Per non mostrare il monitor degli errori
        sat.setSpeedMonitorEnabled(True)

        print("[LOOP] ..")
        # sat.addServiceChannel(serviceAutoName)
        sat.addStreamingChannel(
            Flow_T.FT_BLOB, Variant_T.T_BYTEARRAY, gestureManualChanName)
        sat.addStreamingChannel(
            Flow_T.FT_BLOB, Variant_T.T_BYTEARRAY, gestureModeChanName, props={"mode": controller._current_mode.value})

    return ok


def loop():

    if serviceAuto and gestureManualChan and gestureModeChan:
        if (chronoImage.stop() > 0.05):
            global is_from_phone
            # Leggo l'immagine dalla videocamera
            ImageUtils.capture_image(device, controller, is_from_phone)

            # Calcolo l'operazione sulle mani
            current_operation = controller.compute_operation()

            # Scrivo l'operazione calcolata sull'immagine e la mostro
            ImageUtils.write_on_image(controller._image, current_operation,
                                      controller._pos, controller._orient, controller._current_mode, controller._real_pos, controller._real_orient)
            ImageUtils.show_image(controller._image)

            if (current_operation == "Calcolando" or current_operation == "Arrivato"):
                current_operation = Command.STOP
            
            if controller._current_mode == Mode.MANUAL:
                controller.publish_manual_operation(current_operation)
            else:
                controller.publish_auto_operation(current_operation)

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
    global serviceAuto
    global gestureManualChan
    global gestureModeChan
    global gestureConfirmChan
    global gesturePositionChan

    if (ch.name == f"guest.{serviceAutoName}"):
        print("Service ADDED: {}".format(ch.name))
        serviceAuto = ch

    elif (ch.name == f"guest.{gestureManualChanName}"):
        print("Channel ADDED: {}".format(ch.name))
        gestureManualChan = ch
        
    elif (ch.name == f"guest.{gestureModeChanName}"):
        print("Channel ADDED: {}".format(ch.name))
        gestureModeChan = ch               
        print("Current mode: ", controller._current_mode.value)
        
        
        sat.setCurrentDbName(gestureModeChan.name)
        sat.setVariable("mode", controller._current_mode.value)
        sat.setCurrentDbName(sat._userName)
   
        
    elif (ch.name == f"guest.{gestureConfirmChanName}"):
        print("Channel ADDED: {}".format(ch.name))
        gestureConfirmChan = ch
        sat.subscribeChannel(gestureConfirmChan.chanID)
        
    elif (ch.name == f"guest.{gesturePositionChanName}"):
        print("Channel ADDED: {}".format(ch.name))
        gesturePositionChan = ch
        sat.subscribeChannel(gesturePositionChan.chanID)


def onChannelRemoved(ch):
    print("Channel REMOVED: {}".format(ch.name))


def onStartChanPub(ch):
    print("Publish START required: {}".format(ch.name))


def onStopChanPub(ch):
    print("Publish STOP required: {}".format(ch.name))

def onDataGrabbed(chanID, data):

    if (chanID == gesturePositionChan.chanID):

        # x, y, controller._orient, real_x, real_y, controller._real_orient = DataBuffer(data).toFloat(6)
        
        try:
            x, y, controller._orient, real_x, real_y, controller._real_orient = struct.unpack('<ffffff', data)
        except struct.error:
            x, y, controller._orient = None, None, None
            real_x, real_y, controller._real_orient = struct.unpack('<fff', data)
        
        
        controller._pos = (x, y)
        controller._real_pos = (real_x, real_y)


def onServiceResponse(chanID, val):
    # print("Arrivati", flush=True)
    controller._reached = True
    print("Service response RECEIVED: {}".format(val))


###############################################################################
