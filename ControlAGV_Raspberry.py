# -*- coding: utf-8 -*-
"""
Created on Sat Apr 11 21:40:13 2026

@author: oreaj
"""

# -*- coding: utf-8 -*-
"""
Control Difuso para Carro Ackermann 1:10 con VisiÃ³n
Sistema Mamdani con defuzzificaciÃ³n por Centro de Gravedad
(Adaptado para Raspberry Pi con Picamera2)
"""

import cv2
import numpy as np
import paho.mqtt.client as mqtt
from picamera2 import Picamera2  # <-- NUEVA LIBRERÃA IMPORTADA


# ============================================================
# FUNCIONES DE MEMBRESÃA
# ============================================================

def trimf(x, params):
    """FunciÃ³n de membresÃ­a triangular. params = [a, b, c]"""
    a, b, c = params
    result = np.zeros_like(x, dtype=float)
    if b != a:
        mask = (x >= a) & (x <= b)
        result[mask] = (x[mask] - a) / (b - a)
    if c != b:
        mask = (x > b) & (x <= c)
        result[mask] = (c - x[mask]) / (c - b)
    # Punto exacto en b
    result[x == b] = 1.0
    return result

def trapmf(x, params):
    """FunciÃ³n de membresÃ­a trapezoidal. params = [a, b, c, d]"""
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
    """EvalÃºa trimf para un valor escalar."""
    return float(trimf(np.array([val]), params)[0])

def trapmf_scalar(val, params):
    """EvalÃºa trapmf para un valor escalar."""
    return float(trapmf(np.array([val]), params)[0])


# ============================================================
# CLASE CONTROL DIFUSO
# ============================================================

class ControlDifuso:
    """
    Controlador difuso tipo Mamdani para direcciÃ³n Ackermann.
    Entrada: posiciÃ³n normalizada del objeto en el frame [0, 1]
    Salida: Ã¡ngulo del servo [80, 135] grados
    """

    def __init__(self):
        self.servo_min = 80
        self.servo_max = 135
        self.servo_recto = 113
        self.output_universe = np.linspace(self.servo_min, self.servo_max,
                                           self.servo_max - self.servo_min + 1)

        # Conjuntos de entrada
        self.mf_in_muy_izq_params = [0.0, 0.0, 0.05, 0.20]
        self.mf_in_med_izq_params = [0.05, 0.20, 0.35]
        self.mf_in_poco_izq_params = [0.20, 0.35, 0.50]
        self.mf_in_centro_params = [0.35, 0.50, 0.65]
        self.mf_in_poco_der_params = [0.50, 0.65, 0.80]
        self.mf_in_med_der_params = [0.65, 0.80, 0.95]
        self.mf_in_muy_der_params = [0.80, 0.95, 1.0, 1.0]

        # Conjuntos de salida
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
        r1  = mu_muy_izq
        r2  = mu_med_izq
        r3  = mu_poco_izq
        r4  = mu_cen
        r5  = mu_poco_der
        r6  = mu_med_der
        r7  = mu_muy_der

        r8  = min(mu_muy_izq, 1.0 - mu_med_izq)
        r9  = min(mu_muy_der, 1.0 - mu_med_der)
        r10 = min(mu_med_izq, 1.0 - mu_poco_izq)
        r11 = min(mu_med_der, 1.0 - mu_poco_der)

        r12 = min(mu_poco_izq, mu_cen)
        r13 = min(mu_poco_der, mu_cen)
        r14 = min(mu_cen, 1.0 - mu_poco_izq, 1.0 - mu_poco_der)

        r15 = min(1.0 - mu_muy_izq, 1.0 - mu_med_izq, 1.0 - mu_poco_izq, 
                  1.0 - mu_cen, 1.0 - mu_poco_der, 1.0 - mu_med_der, 1.0 - mu_muy_der)

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
# CONFIGURACIÃ“N
# ============================================================

BROKER = "10.108.51.191"
PORT = 1883
TOPIC_SERVO = "esp32/servo/control"

LOWER_BLUE = np.array([100, 150, 0])
UPPER_BLUE = np.array([140, 255, 255])

MIN_AREA = 500

SERVO_MIN = 80
SERVO_MAX = 135
SERVO_RECTO = 113

def enviar_angulo_servo(client, angulo):
    angulo_int = int(round(angulo))
    angulo_int = max(SERVO_MIN, min(SERVO_MAX, angulo_int))
    client.publish(TOPIC_SERVO, str(angulo_int))
    return angulo_int


# ============================================================
# BUCLE PRINCIPAL DE VISIÃ“N + CONTROL
# ============================================================

def main():
    fuzzy = ControlDifuso()

    # --- INICIALIZACIÃ“N DE LA CÃMARA (Modificado para Raspberry) ---
    picam2 = Picamera2()
    # Forzamos resoluciÃ³n 640x480 para mantener el procesamiento rÃ¡pido y estable
    config = picam2.create_preview_configuration(main={"size": (640, 480)})
    picam2.configure(config)
    picam2.start()
    # ---------------------------------------------------------------

    # ConfiguraciÃ³n MQTT
    client = mqtt.Client()
    try:
        client.connect(BROKER, PORT, 60)
        client.loop_start()
    except Exception as e:
        print(f"Advertencia: No se pudo conectar al broker MQTT ({e})")

    ultimo_angulo = SERVO_RECTO
    print("Control Difuso Ackermann iniciado. ESC para salir.")

    while True:
        # --- CAPTURA DE IMAGEN (Modificado para Raspberry) ---
        frame = picam2.capture_array()
        # -----------------------------------------------------

        height, width, _ = frame.shape

        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, LOWER_BLUE, UPPER_BLUE)
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        tercio_izq = width // 3
        tercio_der = 2 * width // 3

        overlay = frame.copy()
        cv2.rectangle(overlay, (tercio_izq, 0), (tercio_der, height), (0, 255, 0), -1)
        cv2.addWeighted(overlay, 0.10, frame, 0.90, 0, frame)

        cv2.line(frame, (tercio_izq, 0), (tercio_izq, height), (0, 255, 0), 1)
        cv2.line(frame, (tercio_der, 0), (tercio_der, height), (0, 255, 0), 1)
        cv2.line(frame, (width // 2, 0), (width // 2, height), (200, 200, 200), 1)

        angulo = float(SERVO_RECTO)
        objeto_detectado = False

        for contour in contours:
            area = cv2.contourArea(contour)
            if area > MIN_AREA:
                x, y, w, h = cv2.boundingRect(contour)
                cx = x + w // 2
                cy = y + h // 2

                posicion_norm = cx / width
                angulo = fuzzy.calcular(posicion_norm)
                objeto_detectado = True

                cv2.rectangle(frame, (x, y), (x + w, y + h), (255, 0, 0), 2)
                cv2.circle(frame, (cx, cy), 5, (0, 0, 255), -1)
                cv2.putText(frame, f"Pos: {posicion_norm:.2f}",
                            (cx + 10, cy - 10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 1)
                break

        if objeto_detectado and abs(angulo - ultimo_angulo) > 2:
            angulo_enviado = enviar_angulo_servo(client, angulo)
            ultimo_angulo = angulo
            print(f"Servo â†’ {angulo_enviado}Â°  |  Pos: {posicion_norm:.2f}")

        # HUD
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

        rango = SERVO_MAX - SERVO_MIN
        barra_x = int(((angulo - SERVO_MIN) / rango) * width)
        cv2.line(frame, (barra_x, height - 30), (barra_x, height), (0, 0, 255), 3)
        cv2.line(frame, (0, height - 15), (width, height - 15), (100, 100, 100), 1)

        for deg, lbl in [(SERVO_MIN, "80"), (SERVO_RECTO, "113"), (SERVO_MAX, "135")]:
            px = int(((deg - SERVO_MIN) / rango) * width)
            cv2.line(frame, (px, height - 25), (px, height - 5), (200, 200, 200), 1)
            cv2.putText(frame, lbl, (px - 10, height - 28),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.35, (200, 200, 200), 1)

        cv2.imshow("Control Difuso Ackermann", frame)
        cv2.imshow("Mascara Azul", mask)

        if cv2.waitKey(1) & 0xFF == 27:
            break

    # --- APAGADO SEGURO DE LA CÃMARA ---
    picam2.stop()
    # -----------------------------------
    cv2.destroyAllWindows()
    client.loop_stop()
    client.disconnect()
    print("Sistema finalizado.")

if __name__ == "__main__":
    main()
