# -*- coding: utf-8 -*-
"""
Created on Tue Feb 17 15:21:03 2026

@author: oreaj
"""

import cv2
import numpy as np
import paho.mqtt.client as mqtt


# Abrir cámara
cap = cv2.VideoCapture(0)
client = mqtt.Client()
client.connect("192.168.1.82", 1883, 60)
client.loop_start()

posicion_actual = None
ultima_posicion = None

# Margen de la zona segura: porcentaje del ancho total desde el centro hacia cada lado
MARGEN_ZONA_SEGURA = 0.15  # 15% del ancho a cada lado (zona total = 30% del frame)


while True:
    ret, frame = cap.read()
    if not ret:
        break

    # Convertir a HSV
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    # Rango de azul en HSV
    lower_blue = np.array([100, 150, 0])
    upper_blue = np.array([140, 255, 255])

    # Crear máscara
    mask = cv2.inRange(hsv, lower_blue, upper_blue)

    # Encontrar contornos
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    # Dimensiones del frame
    height, width, _ = frame.shape

    # Límites de la zona segura centrada en el frame
    zona_izq = int(width // 2 - width * MARGEN_ZONA_SEGURA)
    zona_der = int(width // 2 + width * MARGEN_ZONA_SEGURA)

    # Dibujar zona segura con fondo semitransparente verde
    overlay = frame.copy()
    cv2.rectangle(overlay, (zona_izq, 0), (zona_der, height), (0, 255, 0), -1)
    cv2.addWeighted(overlay, 0.15, frame, 0.85, 0, frame)

    # Líneas de borde de la zona segura
    cv2.line(frame, (zona_izq, 0), (zona_izq, height), (0, 255, 0), 2)
    cv2.line(frame, (zona_der, 0), (zona_der, height), (0, 255, 0), 2)

    # Línea central tenue de referencia
    cv2.line(frame, (width // 2, 0), (width // 2, height), (200, 200, 200), 1)

    for contour in contours:
        area = cv2.contourArea(contour)

        if area > 500:
            x, y, w, h = cv2.boundingRect(contour)

            # Centro del objeto detectado
            cx = x + w // 2
            cy = y + h // 2

            # Dibujar rectángulo y centroide
            cv2.rectangle(frame, (x, y), (x+w, y+h), (255, 0, 0), 2)
            cv2.circle(frame, (cx, cy), 5, (0, 0, 255), -1)

            # Determinar posición respecto a la zona segura
            if cx < zona_izq:
                posicion_actual = "IZQ"
            elif cx > zona_der:
                posicion_actual = "DER"
            else:
                posicion_actual = "CENTRO"

            break

    if posicion_actual is not None and posicion_actual != ultima_posicion:
        print(posicion_actual)
        client.publish("esp32/control", posicion_actual)
        ultima_posicion = posicion_actual

    # Mostrar comando actual en pantalla
    if ultima_posicion:
        if ultima_posicion == "CENTRO":
            color_texto = (0, 220, 0)
        else:
            color_texto = (0, 140, 255)
        cv2.putText(frame, ultima_posicion, (10, 40),
                    cv2.FONT_HERSHEY_SIMPLEX, 1.2, color_texto, 3)

    cv2.imshow("Frame", frame)
    cv2.imshow("Mascara Azul", mask)

    if cv2.waitKey(1) & 0xFF == 27:
        break

cap.release()
cv2.destroyAllWindows()
client.loop_stop()
client.disconnect()
