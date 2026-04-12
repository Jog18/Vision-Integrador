# -*- coding: utf-8 -*-
"""
Control Difuso para Carro Ackermann 1:10 — Seguidor de Linea
Sistema Mamdani con defuzzificacion por Centro de Gravedad
Detecta linea blanca sobre fondo negro usando Picamera2

@author: oreaj
"""

import cv2
import numpy as np
import paho.mqtt.client as mqtt
from picamera2 import Picamera2


# ============================================================
# FUNCIONES DE MEMBRESIA
# ============================================================

def trimf(x, params):
    """Funcion de membresia triangular. params = [a, b, c]"""
    a, b, c = params
    result = np.zeros_like(x, dtype=float)
    if b != a:
        mask = (x >= a) & (x <= b)
        result[mask] = (x[mask] - a) / (b - a)
    if c != b:
        mask = (x > b) & (x <= c)
        result[mask] = (c - x[mask]) / (c - b)
    result[x == b] = 1.0
    return result


def trapmf(x, params):
    """Funcion de membresia trapezoidal. params = [a, b, c, d]"""
    a, b, c, d = params
    result = np.zeros_like(x, dtype=float)
    if b != a:
        mask = (x >= a) & (x < b)
        result[mask] = (x[mask] - a) / (b - a)
    mask = (x >= b) & (x <= c)
    result[mask] = 1.0
    if d != c:
        mask = (x > c) & (x <= d)
        result[mask] = (d - x[mask]) / (d - c)
    return result


def trimf_scalar(val, params):
    return float(trimf(np.array([val]), params)[0])


def trapmf_scalar(val, params):
    return float(trapmf(np.array([val]), params)[0])


# ============================================================
# CLASE CONTROL DIFUSO
# ============================================================

class ControlDifuso:
    """
    Controlador difuso tipo Mamdani para direccion Ackermann.
    Entrada: posicion normalizada de la linea en el frame [0, 1]
        - 0.0 = linea en el borde izquierdo
        - 0.5 = linea centrada
        - 1.0 = linea en el borde derecho
    Salida: angulo del servo [80, 135] grados
        - 80  = giro maximo a la izquierda
        - 113 = linea recta
        - 135 = giro maximo a la derecha
    """

    def __init__(self):
        self.servo_min = 80
        self.servo_max = 135
        self.servo_recto = 113
        self.output_universe = np.linspace(self.servo_min, self.servo_max,
                                           self.servo_max - self.servo_min + 1)

        # Conjuntos de entrada [0, 1] (7 conjuntos)
        self.mf_in_muy_izq_params = [0.0, 0.0, 0.05, 0.20]
        self.mf_in_med_izq_params = [0.05, 0.20, 0.35]
        self.mf_in_poco_izq_params = [0.20, 0.35, 0.50]
        self.mf_in_centro_params = [0.35, 0.50, 0.65]
        self.mf_in_poco_der_params = [0.50, 0.65, 0.80]
        self.mf_in_med_der_params = [0.65, 0.80, 0.95]
        self.mf_in_muy_der_params = [0.80, 0.95, 1.0, 1.0]

        # Conjuntos de salida [80, 135] (asimetrico: 33 izq, 22 der)
        self.mf_out_params = {
            'muy_izq':  [80, 80, 91],
            'med_izq':  [80, 91, 102],
            'poco_izq': [91, 102, 113],
            'centro':   [102, 113, 120],
            'poco_der': [113, 120, 127],
            'med_der':  [120, 127, 135],
            'muy_der':  [127, 135, 135],
        }

        self.mf_out = {}
        for nombre, params in self.mf_out_params.items():
            self.mf_out[nombre] = trimf(self.output_universe, params)

    def fuzzificar(self, pos):
        mu_muy_izq  = trapmf_scalar(pos, self.mf_in_muy_izq_params)
        mu_med_izq  = trimf_scalar(pos, self.mf_in_med_izq_params)
        mu_poco_izq = trimf_scalar(pos, self.mf_in_poco_izq_params)
        mu_cen      = trimf_scalar(pos, self.mf_in_centro_params)
        mu_poco_der = trimf_scalar(pos, self.mf_in_poco_der_params)
        mu_med_der  = trimf_scalar(pos, self.mf_in_med_der_params)
        mu_muy_der  = trapmf_scalar(pos, self.mf_in_muy_der_params)
        return (mu_muy_izq, mu_med_izq, mu_poco_izq, mu_cen,
                mu_poco_der, mu_med_der, mu_muy_der)

    def evaluar_reglas(self, mu_muy_izq, mu_med_izq, mu_poco_izq,
                       mu_cen, mu_poco_der, mu_med_der, mu_muy_der):
        # 7 reglas directas
        r1  = mu_muy_izq
        r2  = mu_med_izq
        r3  = mu_poco_izq
        r4  = mu_cen
        r5  = mu_poco_der
        r6  = mu_med_der
        r7  = mu_muy_der
        # 4 reglas de refuerzo en extremos
        r8  = min(mu_muy_izq, 1.0 - mu_med_izq)
        r9  = min(mu_muy_der, 1.0 - mu_med_der)
        r10 = min(mu_med_izq, 1.0 - mu_poco_izq)
        r11 = min(mu_med_der, 1.0 - mu_poco_der)
        # 3 reglas de transicion suave
        r12 = min(mu_poco_izq, mu_cen)
        r13 = min(mu_poco_der, mu_cen)
        r14 = min(mu_cen, 1.0 - mu_poco_izq, 1.0 - mu_poco_der)
        # 1 regla de seguridad
        r15 = min(1.0 - mu_muy_izq, 1.0 - mu_med_izq, 1.0 - mu_poco_izq,
                  1.0 - mu_cen, 1.0 - mu_poco_der, 1.0 - mu_med_der,
                  1.0 - mu_muy_der)

        activaciones = {
            'muy_izq':  max(r1, r8),
            'med_izq':  max(r2, r10),
            'poco_izq': r3,
            'centro':   max(r4, r12, r13, r14, r15),
            'poco_der': r5,
            'med_der':  max(r6, r11),
            'muy_der':  max(r7, r9),
        }
        return activaciones

    def defuzzificar(self, activaciones):
        aggregated = np.zeros_like(self.output_universe, dtype=float)
        for nombre, nivel in activaciones.items():
            clipped = np.minimum(self.mf_out[nombre], nivel)
            aggregated = np.maximum(aggregated, clipped)

        area_total = np.sum(aggregated)
        if area_total == 0:
            return float(self.servo_recto)

        centroide = np.sum(self.output_universe * aggregated) / area_total
        return float(centroide)

    def calcular(self, posicion_norm):
        posicion_norm = max(0.0, min(1.0, posicion_norm))
        mus = self.fuzzificar(posicion_norm)
        activaciones = self.evaluar_reglas(*mus)
        angulo = self.defuzzificar(activaciones)
        return angulo


# ============================================================
# CONFIGURACION
# ============================================================

# Broker local (Mosquitto corre en esta misma Raspberry Pi)
BROKER = "127.0.0.1"
PORT = 1883
TOPIC_SERVO = "esp32/servo/control"

# Servo
SERVO_MIN = 80
SERVO_MAX = 135
SERVO_RECTO = 113

# --- Deteccion de linea blanca ---
# Umbral de binarizacion: pixeles con valor > UMBRAL_BLANCO se consideran linea
UMBRAL_BLANCO = 200
# Area minima de contorno (pixeles) para filtrar ruido
MIN_AREA = 300
# Proporcion del frame que se usa como ROI (zona inferior de la imagen)
# 0.4 = el 40% inferior del frame. Ajustar segun la altura de montaje de la camara.
ROI_PROPORCION = 0.4


def enviar_angulo_servo(client, angulo):
    angulo_int = int(round(angulo))
    angulo_int = max(SERVO_MIN, min(SERVO_MAX, angulo_int))
    client.publish(TOPIC_SERVO, str(angulo_int))
    return angulo_int


# ============================================================
# BUCLE PRINCIPAL DE VISION + CONTROL
# ============================================================

def main():
    fuzzy = ControlDifuso()

    # Inicializacion de la camara CSI con Picamera2
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
    print("Seguidor de linea Ackermann iniciado. ESC para salir.")

    while True:
        frame = picam2.capture_array()
        height, width, _ = frame.shape

        # --- Deteccion de linea blanca sobre fondo negro ---
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        # ROI: solo la franja inferior del frame (donde la linea esta mas cerca)
        roi_y = int(height * (1.0 - ROI_PROPORCION))
        roi_gray = gray[roi_y:height, :]

        # Binarizacion: blanco = linea, negro = fondo
        _, thresh = cv2.threshold(roi_gray, UMBRAL_BLANCO, 255, cv2.THRESH_BINARY)

        # Reducir ruido
        kernel = np.ones((3, 3), np.uint8)
        thresh = cv2.morphologyEx(thresh, cv2.MORPH_OPEN, kernel)
        thresh = cv2.morphologyEx(thresh, cv2.MORPH_CLOSE, kernel)

        # Buscar contornos en el ROI
        contours, _ = cv2.findContours(thresh, cv2.RETR_EXTERNAL,
                                       cv2.CHAIN_APPROX_SIMPLE)

        # --- Visualizacion: lineas de referencia sobre el frame completo ---
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

        # Linea horizontal marcando el inicio del ROI
        cv2.line(frame, (0, roi_y), (width, roi_y), (0, 200, 200), 1)
        cv2.putText(frame, "ROI", (5, roi_y - 5),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 200, 200), 1)

        angulo = float(SERVO_RECTO)
        linea_detectada = False
        posicion_norm = 0.5

        if contours:
            # Tomar el contorno mas grande (la linea principal)
            mayor = max(contours, key=cv2.contourArea)
            area = cv2.contourArea(mayor)

            if area > MIN_AREA:
                # Centroide del contorno usando momentos
                M = cv2.moments(mayor)
                if M["m00"] > 0:
                    cx = int(M["m10"] / M["m00"])
                    cy = int(M["m01"] / M["m00"])

                    # Normalizar posicion horizontal al rango [0, 1]
                    posicion_norm = cx / width

                    # Control difuso
                    angulo = fuzzy.calcular(posicion_norm)
                    linea_detectada = True

                    # Dibujar sobre el frame (ajustar cy al sistema completo)
                    cy_frame = cy + roi_y
                    cv2.drawContours(frame[roi_y:height, :], [mayor], -1,
                                     (255, 0, 0), 2)
                    cv2.circle(frame, (cx, cy_frame), 6, (0, 0, 255), -1)
                    cv2.putText(frame, f"Pos: {posicion_norm:.2f}",
                                (cx + 10, cy_frame - 10),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                                (255, 255, 0), 1)

        # Enviar al servo si el cambio es significativo (>2 grados)
        if linea_detectada and abs(angulo - ultimo_angulo) > 2:
            angulo_enviado = enviar_angulo_servo(client, angulo)
            ultimo_angulo = angulo
            print(f"Servo -> {angulo_enviado} deg  |  Pos: {posicion_norm:.2f}")

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

        # Barra indicadora inferior (113 centrado en pantalla)
        # Mitad izquierda del frame = [80, 113], mitad derecha = [113, 135]
        mitad = width // 2
        rango_izq = SERVO_RECTO - SERVO_MIN  # 33
        rango_der = SERVO_MAX - SERVO_RECTO   # 22

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

        cv2.imshow("Seguidor de Linea Ackermann", frame)
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
