from pynput import keyboard
import paho.mqtt.client as mqtt

class MQTTClient:
    def __init__(self, broker_address="localhost", port=1883):
        self.client = mqtt.Client()
        self.client.connect(broker_address, port)

    def publish(self, message, topic="command"):
        self.client.publish(topic, message)

client = MQTTClient()

def on_press(key):
    try:
        if key == keyboard.Key.up:
            client.publish("su")
            # Esegui l'operazione per la freccia su
        elif key == keyboard.Key.down:
            client.publish("giù")
            # Esegui l'operazione per la freccia giù
        elif key == keyboard.Key.left:
            client.publish("sinistra")
            # Esegui l'operazione per la freccia sinistra
        elif key == keyboard.Key.right:
            client.publish("destra")
            # Esegui l'operazione per la freccia destra
        elif key == keyboard.Key.alt_gr:
            client.publish("stop")
            # Esegui l'operazione stop
    except AttributeError:
        pass

print("Inizia a guidare!\n")
# Registra la funzione on_press come callback per i press dei tasti
with keyboard.Listener(on_press=on_press) as listener:
    listener.join()
