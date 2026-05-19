# -*- coding: utf-8 -*-
"""
Control Difuso para Carro Ackermann 1:10 — Seguidor de Linea v2.2
Controlador AGRESIVO: respuesta rapida ante desviaciones pequenas
para compensar el FOV estrecho de la Pi Camera V2 (~62 grados).

Cambios respecto a la version anterior:
  - MF de entrada "centro" mucho mas angosta (±0.07 vs ±0.15)
  - MFs de salida "poco_izq/poco_der" generan correcciones ~45% mas fuertes
  - MF de salida "centro" tambien angosta (±5° vs ±11°)
  - Universo simetrico: [80, 146], centro=113, ±33 grados

@author: oreaj
"""

import cv2
import numpy as np
import skfuzzy as fuzz
from skfuzzy import control as ctrl
import paho.mqtt.client as mqtt
from picamera2 import Picamera2


# ============================================================
# SISTEMA DIFUSO AGRESIVO CON SCIKIT-FUZZY
# ============================================================

def crear_sistema_difuso():
    """
    Crea el sistema de control difuso Mamdani AGRESIVO.
    Entrada: posicion normalizada [0, 1]  (0.5 = centrado)
    Salida: angulo del servo [80, 146]    (113 = recto, simetrico ±33)

    Diferencia clave: la zona "centro" es angosta, asi cualquier
    desviacion pequena activa rapidamente poco_izq/poco_der y produce
    correcciones mas fuertes que la version anterior.
    """

    # --- Variables linguisticas ---
    posicion = ctrl.Antecedent(np.linspace(0, 1, 1001), 'posicion')
    servo = ctrl.Consequent(np.linspace(80, 146, 67), 'servo', defuzzify_method='centroid')

    # --- Funciones de membresia de ENTRADA ---
    # Centro ANGOSTO: [0.43, 0.50, 0.57] (antes era [0.35, 0.50, 0.65])
    # Efecto: el error entra en poco_izq/poco_der mucho mas rapido
    #
    #  Anterior:  centro cubria 0.35 a 0.65 (ancho 0.30)
    #  Ahora:     centro cubre  0.43 a 0.57 (ancho 0.14) → ~53% mas angosto
    #
    posicion['muy_izq']  = fuzz.trapmf(posicion.universe, [0.0, 0.0, 0.05, 0.17])
    posicion['med_izq']  = fuzz.trimf(posicion.universe,  [0.05, 0.17, 0.30])
    posicion['poco_izq'] = fuzz.trimf(posicion.universe,  [0.17, 0.30, 0.45])
    posicion['centro']   = fuzz.trimf(posicion.universe,  [0.43, 0.50, 0.57])
    posicion['poco_der'] = fuzz.trimf(posicion.universe,  [0.55, 0.70, 0.83])
    posicion['med_der']  = fuzz.trimf(posicion.universe,  [0.70, 0.83, 0.95])
    posicion['muy_der']  = fuzz.trapmf(posicion.universe, [0.83, 0.95, 1.0, 1.0])

    # --- Funciones de membresia de SALIDA ---
    # Simetrico [80, 146], centro=113
    #
    # Cambios clave:
    #   poco_izq pico en 97° (antes 102°) → 16° de correccion vs 11° anterior
    #   poco_der pico en 129° (antes 124°) → 16° de correccion vs 11° anterior
    #   centro angosto [108,113,118] (antes [102,113,124]) → zona recta minima
    #
    #   Anterior (suave):    80---91---102---113---124---135---146
    #   Ahora (agresivo):    80---89----97---113---129---137---146
    #
    servo['muy_izq']  = fuzz.trimf(servo.universe, [80,  80,  89])
    servo['med_izq']  = fuzz.trimf(servo.universe, [80,  89,  100])
    servo['poco_izq'] = fuzz.trimf(servo.universe, [86,  97,  110])
    servo['centro']   = fuzz.trimf(servo.universe, [108, 113, 118])
    servo['poco_der'] = fuzz.trimf(servo.universe, [116, 129, 140])
    servo['med_der']  = fuzz.trimf(servo.universe, [126, 137, 146])
    servo['muy_der']  = fuzz.trimf(servo.universe, [137, 146, 146])

    # --- Reglas difusas (7 reglas directas, simetricas) ---
    r1 = ctrl.Rule(posicion['muy_izq'],  servo['muy_izq'])
    r2 = ctrl.Rule(posicion['med_izq'],  servo['med_izq'])
    r3 = ctrl.Rule(posicion['poco_izq'], servo['poco_izq'])
    r4 = ctrl.Rule(posicion['centro'],   servo['centro'])
    r5 = ctrl.Rule(posicion['poco_der'], servo['poco_der'])
    r6 = ctrl.Rule(posicion['med_der'],  servo['med_der'])
    r7 = ctrl.Rule(posicion['muy_der'],  servo['muy_der'])

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
SERVO_MAX = 146
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
    """Calcula el angulo del servo usando el sistema difuso agresivo."""
    posicion_norm = max(0.0, min(1.0, posicion_norm))
    simulacion.input['posicion'] = posicion_norm
    simulacion.compute()
    return simulacion.output['servo']


# ============================================================
# BUCLE PRINCIPAL DE VISION + CONTROL
# ============================================================

def main():
    simulacion = crear_sistema_difuso()
    print("Sistema difuso v2.2 (AGRESIVO) creado.")
    print("  Centro entrada: [0.43, 0.50, 0.57] (angosto)")
    print("  poco_izq salida pico: 97° | poco_der salida pico: 129°")

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
    ultima_posicion = 0.5
    print("Seguidor de linea v2.2 (agresivo) iniciado. ESC para salir.")

    while True:
        frame = picam2.capture_array()
        height, width, _ = frame.shape

        # --- Deteccion de linea blanca sobre fondo negro ---
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        roi_y = int(height * (1.0 - ROI_PROPORCION))
        roi_gray = gray[roi_y:height, :]

        _, thresh = cv2.threshold(roi_gray, UMBRAL_BLANCO, 255, cv2.THRESH_BINARY)

        kernel = np.ones((3, 3), np.uint8)
        thresh = cv2.morphologyEx(thresh, cv2.MORPH_OPEN, kernel)
        thresh = cv2.morphologyEx(thresh, cv2.MORPH_CLOSE, kernel)

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

                    angulo = calcular_angulo(simulacion, posicion_norm)
                    linea_detectada = True

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
                angulo = float(SERVO_MIN)
            else:
                angulo = float(SERVO_MAX)

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
        cv2.putText(frame, "v2.2 AGRESIVO", (10, height - 45),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 200, 255), 1)

        if not linea_detectada:
            cv2.putText(frame, "SIN LINEA", (10, 105),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)

        # Barra indicadora inferior (simetrica: 113 centrado en pantalla)
        mitad = width // 2
        rango_izq = SERVO_RECTO - SERVO_MIN  # 33
        rango_der = SERVO_MAX - SERVO_RECTO   # 33

        if angulo <= SERVO_RECTO:
            barra_x = int(((angulo - SERVO_MIN) / rango_izq) * mitad)
        else:
            barra_x = mitad + int(((angulo - SERVO_RECTO) / rango_der) * mitad)

        cv2.line(frame, (barra_x, height - 30), (barra_x, height),
                 (0, 0, 255), 3)
        cv2.line(frame, (0, height - 15), (width, height - 15),
                 (100, 100, 100), 1)

        for deg, lbl in [(SERVO_MIN, "80"), (SERVO_RECTO, "113"),
                         (SERVO_MAX, "146")]:
            if deg <= SERVO_RECTO:
                px = int(((deg - SERVO_MIN) / rango_izq) * mitad)
            else:
                px = mitad + int(((deg - SERVO_RECTO) / rango_der) * mitad)
            cv2.line(frame, (px, height - 25), (px, height - 5),
                     (200, 200, 200), 1)
            cv2.putText(frame, lbl, (px - 10, height - 28),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.35, (200, 200, 200), 1)

        cv2.imshow("Seguidor de Linea v2.2 (agresivo)", frame)
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
