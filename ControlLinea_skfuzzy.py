# -*- coding: utf-8 -*-
"""
Control Difuso para Carro Ackermann 1:10 — Seguidor de Linea
Usa scikit-fuzzy (skfuzzy) para el sistema Mamdani
Detecta linea blanca sobre fondo negro usando Picamera2

@author: oreaj
"""

import cv2
import numpy as np
import skfuzzy as fuzz
from skfuzzy import control as ctrl
import paho.mqtt.client as mqtt
from picamera2 import Picamera2


# ============================================================
# SISTEMA DIFUSO CON SCIKIT-FUZZY
# ============================================================

def crear_sistema_difuso():
    """
    Crea el sistema de control difuso Mamdani.
    Entrada: posicion normalizada [0, 1]
    Salida: angulo del servo [80, 135]
    """

    # --- Variables linguisticas ---
    posicion = ctrl.Antecedent(np.linspace(0, 1, 1001), 'posicion')
    servo = ctrl.Consequent(np.linspace(80, 135, 56), 'servo', defuzzify_method='centroid')

    # --- Funciones de membresia de ENTRADA (posicion) ---
    posicion['muy_izq']  = fuzz.trapmf(posicion.universe, [0.0, 0.0, 0.05, 0.20])
    posicion['med_izq']  = fuzz.trimf(posicion.universe, [0.05, 0.20, 0.35])
    posicion['poco_izq'] = fuzz.trimf(posicion.universe, [0.20, 0.35, 0.50])
    posicion['centro']   = fuzz.trimf(posicion.universe, [0.35, 0.50, 0.65])
    posicion['poco_der'] = fuzz.trimf(posicion.universe, [0.50, 0.65, 0.80])
    posicion['med_der']  = fuzz.trimf(posicion.universe, [0.65, 0.80, 0.95])
    posicion['muy_der']  = fuzz.trapmf(posicion.universe, [0.80, 0.95, 1.0, 1.0])

    # --- Funciones de membresia de SALIDA (servo) ---
    # Asimetrico: 80-113 izquierda (33 grados), 113-135 derecha (22 grados)
    servo['muy_izq']  = fuzz.trimf(servo.universe, [80, 80, 91])
    servo['med_izq']  = fuzz.trimf(servo.universe, [80, 91, 102])
    servo['poco_izq'] = fuzz.trimf(servo.universe, [91, 102, 113])
    servo['centro']   = fuzz.trimf(servo.universe, [102, 113, 120])
    servo['poco_der'] = fuzz.trimf(servo.universe, [113, 120, 127])
    servo['med_der']  = fuzz.trimf(servo.universe, [120, 127, 135])
    servo['muy_der']  = fuzz.trimf(servo.universe, [127, 135, 135])

    # --- Reglas difusas ---
    # 7 reglas directas (si la linea esta a la izquierda, girar a la izquierda, etc.)
    r1 = ctrl.Rule(posicion['muy_izq'], servo['muy_izq'])
    r2 = ctrl.Rule(posicion['med_izq'], servo['med_izq'])
    r3 = ctrl.Rule(posicion['poco_izq'], servo['poco_izq'])
    r4 = ctrl.Rule(posicion['centro'], servo['centro'])
    r5 = ctrl.Rule(posicion['poco_der'], servo['poco_der'])
    r6 = ctrl.Rule(posicion['med_der'], servo['med_der'])
    r7 = ctrl.Rule(posicion['muy_der'], servo['muy_der'])

    # --- Crear sistema de control ---
    sistema = ctrl.ControlSystem([r1, r2, r3, r4, r5, r6, r7])
    simulacion = ctrl.ControlSystemSimulation(sistema)

    return simulacion


# ============================================================
# CONFIGURACION
# ============================================================

BROKER = "127.0.0.1"
PORT = 1883
TOPIC_SERVO = "esp32/servo/control"

SERVO_MIN = 80
SERVO_MAX = 135
SERVO_RECTO = 113

# Deteccion de linea blanca
UMBRAL_BLANCO = 200
MIN_AREA = 300
ROI_PROPORCION = 0.4


def enviar_angulo_servo(client, angulo):
    angulo_int = int(round(angulo))
    angulo_int = max(SERVO_MIN, min(SERVO_MAX, angulo_int))
    client.publish(TOPIC_SERVO, str(angulo_int))
    return angulo_int


def calcular_angulo(simulacion, posicion_norm):
    """Calcula el angulo del servo usando el sistema difuso."""
    posicion_norm = max(0.0, min(1.0, posicion_norm))
    simulacion.input['posicion'] = posicion_norm
    simulacion.compute()
    return simulacion.output['servo']


# ============================================================
# BUCLE PRINCIPAL DE VISION + CONTROL
# ============================================================

def main():
    # Crear sistema difuso
    simulacion = crear_sistema_difuso()
    print("Sistema difuso skfuzzy creado.")

    # Camara CSI con Picamera2
    picam2 = Picamera2()
    config = picam2.create_preview_configuration(main={"size": (640, 480)})
    picam2.configure(config)
    picam2.start()

    # Conexion MQTT
    client = mqtt.Client()
    try:
        client.connect(BROKER, PORT, 60)
        client.loop_start()
    except Exception as e:
        print(f"Advertencia: No se pudo conectar al broker MQTT ({e})")

    ultimo_angulo = SERVO_RECTO
    ultima_posicion = 0.5  # ultima posicion conocida de la linea
    print("Seguidor de linea Ackermann (skfuzzy) iniciado. ESC para salir.")

    while True:
        frame = picam2.capture_array()
        height, width, _ = frame.shape

        # --- Deteccion de linea blanca sobre fondo negro ---
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        # ROI: franja inferior del frame
        roi_y = int(height * (1.0 - ROI_PROPORCION))
        roi_gray = gray[roi_y:height, :]

        # Binarizacion
        _, thresh = cv2.threshold(roi_gray, UMBRAL_BLANCO, 255, cv2.THRESH_BINARY)

        # Reducir ruido
        kernel = np.ones((3, 3), np.uint8)
        thresh = cv2.morphologyEx(thresh, cv2.MORPH_OPEN, kernel)
        thresh = cv2.morphologyEx(thresh, cv2.MORPH_CLOSE, kernel)

        # Buscar contornos
        contours, _ = cv2.findContours(thresh, cv2.RETR_EXTERNAL,
                                       cv2.CHAIN_APPROX_SIMPLE)

        # --- Visualizacion ---
        tercio_izq = width // 3
        tercio_der = 2 * width // 3

        overlay = frame.copy()
        cv2.rectangle(overlay, (tercio_izq, 0), (tercio_der, height),
                      (0, 255, 0), -1)
        cv2.addWeighted(overlay, 0.10, frame, 0.90, 0, frame)

        cv2.line(frame, (tercio_izq, 0), (tercio_izq, height), (0, 255, 0), 1)
        cv2.line(frame, (tercio_der, 0), (tercio_der, height), (0, 255, 0), 1)
        cv2.line(frame, (width // 2, 0), (width // 2, height),
                 (200, 200, 200), 1)

        cv2.line(frame, (0, roi_y), (width, roi_y), (0, 200, 200), 1)
        cv2.putText(frame, "ROI", (5, roi_y - 5),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 200, 200), 1)

        angulo = float(SERVO_RECTO)
        linea_detectada = False
        posicion_norm = 0.5

        if contours:
            mayor = max(contours, key=cv2.contourArea)
            area = cv2.contourArea(mayor)

            if area > MIN_AREA:
                M = cv2.moments(mayor)
                if M["m00"] > 0:
                    cx = int(M["m10"] / M["m00"])
                    cy = int(M["m01"] / M["m00"])

                    posicion_norm = cx / width
                    ultima_posicion = posicion_norm

                    # Control difuso con skfuzzy
                    angulo = calcular_angulo(simulacion, posicion_norm)
                    linea_detectada = True

                    # Dibujar
                    cy_frame = cy + roi_y
                    cv2.drawContours(frame[roi_y:height, :], [mayor], -1,
                                     (255, 0, 0), 2)
                    cv2.circle(frame, (cx, cy_frame), 6, (0, 0, 255), -1)
                    cv2.putText(frame, f"Pos: {posicion_norm:.2f}",
                                (cx + 10, cy_frame - 10),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                                (255, 255, 0), 1)

        # Si pierde la linea, girar full hacia el lado donde la vio por ultima vez
        if not linea_detectada:
            if ultima_posicion < 0.5:
                angulo = float(SERVO_MIN)  # linea se perdio a la izquierda -> full izquierda
            else:
                angulo = float(SERVO_MAX)  # linea se perdio a la derecha -> full derecha

        # Enviar al servo si el cambio es significativo
        if abs(angulo - ultimo_angulo) > 2:
            angulo_enviado = enviar_angulo_servo(client, angulo)
            ultimo_angulo = angulo
            if linea_detectada:
                print(f"Servo -> {angulo_enviado} deg  |  Pos: {posicion_norm:.2f}")
            else:
                print(f"Servo -> {angulo_enviado} deg  |  BUSCANDO LINEA")

        # --- HUD ---
        angulo_int = int(round(angulo))
        if angulo_int < SERVO_RECTO - 3:
            color = (0, 165, 255)
            etiqueta = "IZQUIERDA"
        elif angulo_int > SERVO_RECTO + 3:
            color = (255, 165, 0)
            etiqueta = "DERECHA"
        else:
            color = (0, 220, 0)
            etiqueta = "CENTRO"

        cv2.putText(frame, f"Servo: {angulo_int} deg", (10, 35),
                    cv2.FONT_HERSHEY_SIMPLEX, 1.0, color, 2)
        cv2.putText(frame, etiqueta, (10, 70),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, color, 2)

        if not linea_detectada:
            cv2.putText(frame, "SIN LINEA", (10, 105),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)

        # Barra indicadora inferior
        mitad = width // 2
        rango_izq = SERVO_RECTO - SERVO_MIN
        rango_der = SERVO_MAX - SERVO_RECTO

        if angulo <= SERVO_RECTO:
            barra_x = int(((angulo - SERVO_MIN) / rango_izq) * mitad)
        else:
            barra_x = mitad + int(((angulo - SERVO_RECTO) / rango_der) * mitad)

        cv2.line(frame, (barra_x, height - 30), (barra_x, height),
                 (0, 0, 255), 3)
        cv2.line(frame, (0, height - 15), (width, height - 15),
                 (100, 100, 100), 1)

        for deg, lbl in [(SERVO_MIN, "80"), (SERVO_RECTO, "113"),
                         (SERVO_MAX, "135")]:
            if deg <= SERVO_RECTO:
                px = int(((deg - SERVO_MIN) / rango_izq) * mitad)
            else:
                px = mitad + int(((deg - SERVO_RECTO) / rango_der) * mitad)
            cv2.line(frame, (px, height - 25), (px, height - 5),
                     (200, 200, 200), 1)
            cv2.putText(frame, lbl, (px - 10, height - 28),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.35, (200, 200, 200), 1)

        cv2.imshow("Seguidor de Linea Ackermann (skfuzzy)", frame)
        cv2.imshow("Umbral Linea (ROI)", thresh)

        if cv2.waitKey(1) & 0xFF == 27:
            break

    picam2.stop()
    cv2.destroyAllWindows()
    client.loop_stop()
    client.disconnect()
    print("Sistema finalizado.")


if __name__ == "__main__":
    main()
