# -*- coding: utf-8 -*-
"""
Created on Tue Mar 10 15:37:50 2026

@author: oreaj
"""

import paho.mqtt.client as mqtt

# Configuración del broker MQTT
broker = "10.91.115.191"  # IP de tu broker MQTT
port = 1883
topic = "esp32/servo/control"

def enviar_angulo(angulo):
    client = mqtt.Client()
    client.connect(broker, port, 60)
    client.publish(topic, str(angulo))
    client.disconnect()
    print(f"Ángulo {angulo} enviado al tópico {topic}")

if __name__ == "__main__":
    while True:
        angulo = input("Introduce el ángulo para el servo (0-180), o 'salir' para terminar: ")
        if angulo.lower() == 'salir':
            break
        try:
            angulo_int = int(angulo)
            if 0 <= angulo_int <= 180:
                enviar_angulo(angulo_int)
            else:
                print("El ángulo debe estar entre 0 y 180.")
        except ValueError:
            print("Por favor, introduce un número válido.")