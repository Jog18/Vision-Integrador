# -*- coding: utf-8 -*-
"""
Control Difuso para Carro Ackermann 1:10 con Visión
Sistema Mamdani con defuzzificación por Centro de Gravedad

@author: oreaj
"""

import cv2
import numpy as np
import paho.mqtt.client as mqtt


# ============================================================
# FUNCIONES DE MEMBRESÍA
# ============================================================

def trimf(x, params):
    """Función de membresía triangular. params = [a, b, c]"""
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
    """Función de membresía trapezoidal. params = [a, b, c, d]"""
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
    """Evalúa trimf para un valor escalar."""
    return float(trimf(np.array([val]), params)[0])


def trapmf_scalar(val, params):
    """Evalúa trapmf para un valor escalar."""
    return float(trapmf(np.array([val]), params)[0])


# ============================================================
# CLASE CONTROL DIFUSO
# ============================================================

class ControlDifuso:
    """
    Controlador difuso tipo Mamdani para dirección Ackermann.

    Entrada: posición normalizada del objeto en el frame [0, 1]
        - 0.0 = borde izquierdo del frame
        - 0.5 = centro del frame
        - 1.0 = borde derecho del frame

    Salida: ángulo del servo [0, 180] grados
        - 0°   = giro máximo a la izquierda
        - 90°  = línea recta
        - 180° = giro máximo a la derecha

    Conjuntos difusos de entrada (7):
        Muy_Izquierda, Med_Izquierda, Poco_Izquierda,
        Centro,
        Poco_Derecha, Med_Derecha, Muy_Derecha

    Conjuntos difusos de salida (7):
        Muy_Izquierda, Med_Izquierda, Poco_Izquierda,
        Centro,
        Poco_Derecha, Med_Derecha, Muy_Derecha
    """

    def __init__(self):
        # Universo de salida discretizado (resolución de 1 grado)
        self.output_universe = np.linspace(0, 180, 181)

        # ---- Conjuntos difusos de ENTRADA [0, 1] (7 conjuntos) ----
        # Muy_Izquierda: trapezoidal, máximo de 0 a 0.05, cae hasta 0.20
        self.mf_in_muy_izq_params = [0.0, 0.0, 0.05, 0.20]
        # Med_Izquierda: triangular, sube desde 0.05, pico en 0.20, baja hasta 0.35
        self.mf_in_med_izq_params = [0.05, 0.20, 0.35]
        # Poco_Izquierda: triangular, sube desde 0.20, pico en 0.35, baja hasta 0.50
        self.mf_in_poco_izq_params = [0.20, 0.35, 0.50]
        # Centro: triangular, sube desde 0.35, pico en 0.50, baja hasta 0.65
        self.mf_in_centro_params = [0.35, 0.50, 0.65]
        # Poco_Derecha: triangular, sube desde 0.50, pico en 0.65, baja hasta 0.80
        self.mf_in_poco_der_params = [0.50, 0.65, 0.80]
        # Med_Derecha: triangular, sube desde 0.65, pico en 0.80, baja hasta 0.95
        self.mf_in_med_der_params = [0.65, 0.80, 0.95]
        # Muy_Derecha: trapezoidal, sube desde 0.80, máximo de 0.95 a 1.0
        self.mf_in_muy_der_params = [0.80, 0.95, 1.0, 1.0]

        # ---- Conjuntos difusos de SALIDA [0, 180] ----
        self.mf_out_params = {
            'muy_izq':  [0, 0, 30],
            'med_izq':  [15, 40, 65],
            'poco_izq': [45, 67, 90],
            'centro':   [75, 90, 105],
            'poco_der': [90, 113, 135],
            'med_der':  [115, 140, 165],
            'muy_der':  [150, 180, 180],
        }

        # Pre-calcular MFs de salida sobre el universo
        self.mf_out = {}
        for nombre, params in self.mf_out_params.items():
            self.mf_out[nombre] = trimf(self.output_universe, params)

    # ----------------------------------------------------------------
    # PASO 1: FUZZIFICACIÓN
    # ----------------------------------------------------------------
    def fuzzificar(self, pos):
        """
        Calcula los grados de pertenencia de la posición normalizada
        a cada uno de los 7 conjuntos difusos de entrada.
        """
        mu_muy_izq  = trapmf_scalar(pos, self.mf_in_muy_izq_params)
        mu_med_izq  = trimf_scalar(pos, self.mf_in_med_izq_params)
        mu_poco_izq = trimf_scalar(pos, self.mf_in_poco_izq_params)
        mu_cen      = trimf_scalar(pos, self.mf_in_centro_params)
        mu_poco_der = trimf_scalar(pos, self.mf_in_poco_der_params)
        mu_med_der  = trimf_scalar(pos, self.mf_in_med_der_params)
        mu_muy_der  = trapmf_scalar(pos, self.mf_in_muy_der_params)
        return (mu_muy_izq, mu_med_izq, mu_poco_izq, mu_cen,
                mu_poco_der, mu_med_der, mu_muy_der)

    # ----------------------------------------------------------------
    # PASO 2: EVALUACIÓN DE REGLAS (Base de Conocimiento + Inferencia)
    # ----------------------------------------------------------------
    def evaluar_reglas(self, mu_muy_izq, mu_med_izq, mu_poco_izq,
                       mu_cen, mu_poco_der, mu_med_der, mu_muy_der):
        """
        Evalúa las 15 reglas difusas y retorna el nivel de activación
        para cada conjunto de salida.

        Reglas directas (7):
        R1:  IF Muy_Izquierda                → Muy_Izquierda
        R2:  IF Med_Izquierda                → Med_Izquierda
        R3:  IF Poco_Izquierda               → Poco_Izquierda
        R4:  IF Centro                       → Centro
        R5:  IF Poco_Derecha                 → Poco_Derecha
        R6:  IF Med_Derecha                  → Med_Derecha
        R7:  IF Muy_Derecha                  → Muy_Derecha

        Reglas de refuerzo en extremos (4):
        R8:  IF Muy_Izq AND NOT Med_Izq      → Muy_Izquierda
        R9:  IF Muy_Der AND NOT Med_Der       → Muy_Derecha
        R10: IF Med_Izq AND NOT Poco_Izq      → Med_Izquierda
        R11: IF Med_Der AND NOT Poco_Der       → Med_Derecha

        Reglas de transición suave (3):
        R12: IF Poco_Izq AND Centro           → Centro
        R13: IF Poco_Der AND Centro           → Centro
        R14: IF Centro AND NOT Poco_Izq AND NOT Poco_Der → Centro (puro)

        Regla de seguridad (1):
        R15: IF NOT ninguno significativo      → Centro
        """
        # Operador AND = min,  NOT x = 1 - x

        # --- 7 reglas directas ---
        r1  = mu_muy_izq                                  # → Muy_Izquierda
        r2  = mu_med_izq                                  # → Med_Izquierda
        r3  = mu_poco_izq                                 # → Poco_Izquierda
        r4  = mu_cen                                      # → Centro
        r5  = mu_poco_der                                 # → Poco_Derecha
        r6  = mu_med_der                                  # → Med_Derecha
        r7  = mu_muy_der                                  # → Muy_Derecha

        # --- 4 reglas de refuerzo en extremos ---
        r8  = min(mu_muy_izq, 1.0 - mu_med_izq)          # → Muy_Izquierda
        r9  = min(mu_muy_der, 1.0 - mu_med_der)          # → Muy_Derecha
        r10 = min(mu_med_izq, 1.0 - mu_poco_izq)         # → Med_Izquierda
        r11 = min(mu_med_der, 1.0 - mu_poco_der)         # → Med_Derecha

        # --- 3 reglas de transición suave hacia centro ---
        r12 = min(mu_poco_izq, mu_cen)                   # → Centro
        r13 = min(mu_poco_der, mu_cen)                   # → Centro
        r14 = min(mu_cen, 1.0 - mu_poco_izq,
                  1.0 - mu_poco_der)                      # → Centro (puro)

        # --- 1 regla de seguridad ---
        r15 = min(1.0 - mu_muy_izq, 1.0 - mu_med_izq,
                  1.0 - mu_poco_izq, 1.0 - mu_cen,
                  1.0 - mu_poco_der, 1.0 - mu_med_der,
                  1.0 - mu_muy_der)                       # → Centro

        # Agregar activaciones por conjunto de salida (OR = max)
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

    # ----------------------------------------------------------------
    # PASO 3: DEFUZZIFICACIÓN – Centro de Gravedad (Centroide)
    # ----------------------------------------------------------------
    def defuzzificar(self, activaciones):
        """
        Método Mamdani:
        1. Recorta cada MF de salida al nivel de activación de su regla.
        2. Agrega todas las MFs recortadas usando el operador máximo.
        3. Calcula el centroide del área resultante.
        """
        aggregated = np.zeros_like(self.output_universe, dtype=float)

        for nombre, nivel in activaciones.items():
            clipped = np.minimum(self.mf_out[nombre], nivel)
            aggregated = np.maximum(aggregated, clipped)

        # Centroide
        area_total = np.sum(aggregated)
        if area_total == 0:
            return 90.0  # Sin información → recto

        centroide = np.sum(self.output_universe * aggregated) / area_total
        return float(centroide)

    # ----------------------------------------------------------------
    # PIPELINE COMPLETO
    # ----------------------------------------------------------------
    def calcular(self, posicion_norm):
        """
        Ejecuta el pipeline completo del control difuso.
        Input:  posicion_norm ∈ [0, 1]
        Output: ángulo del servo ∈ [0, 180]
        """
        posicion_norm = max(0.0, min(1.0, posicion_norm))
        mus = self.fuzzificar(posicion_norm)
        activaciones = self.evaluar_reglas(*mus)
        angulo = self.defuzzificar(activaciones)
        return angulo


# ============================================================
# CONFIGURACIÓN
# ============================================================

BROKER = "10.108.51.191"
PORT = 1883
TOPIC_SERVO = "esp32/servo/control"

LOWER_BLUE = np.array([100, 150, 0])
UPPER_BLUE = np.array([140, 255, 255])

MIN_AREA = 500


def enviar_angulo_servo(client, angulo):
    """Publica el ángulo del servo al ESP32 vía MQTT."""
    angulo_int = int(round(angulo))
    angulo_int = max(0, min(180, angulo_int))
    client.publish(TOPIC_SERVO, str(angulo_int))
    return angulo_int


# ============================================================
# BUCLE PRINCIPAL DE VISIÓN + CONTROL
# ============================================================

def main():
    fuzzy = ControlDifuso()

    cap = cv2.VideoCapture(0)
    if not cap.isOpened():
        print("Error: No se pudo abrir la cámara")
        return

    client = mqtt.Client()
    client.connect(BROKER, PORT, 60)
    client.loop_start()

    ultimo_angulo = 113

    print("Control Difuso Ackermann iniciado. ESC para salir.")

    while True:
        ret, frame = cap.read()
        if not ret:
            break

        height, width, _ = frame.shape

        # --- Procesamiento de imagen ---
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, LOWER_BLUE, UPPER_BLUE)
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL,
                                       cv2.CHAIN_APPROX_SIMPLE)

        # --- Visualización: tercios de referencia ---
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

        angulo = 90.0
        objeto_detectado = False

        for contour in contours:
            area = cv2.contourArea(contour)
            if area > MIN_AREA:
                x, y, w, h = cv2.boundingRect(contour)
                cx = x + w // 2
                cy = y + h // 2

                # Normalizar posición al rango [0, 1]
                posicion_norm = cx / width

                # Control difuso
                angulo = fuzzy.calcular(posicion_norm)
                objeto_detectado = True

                # Dibujar detección
                cv2.rectangle(frame, (x, y), (x + w, y + h), (255, 0, 0), 2)
                cv2.circle(frame, (cx, cy), 5, (0, 0, 255), -1)
                cv2.putText(frame, f"Pos: {posicion_norm:.2f}",
                            (cx + 10, cy - 10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 1)
                break

        # Enviar al servo si el cambio es significativo (>2°)
        if objeto_detectado and abs(angulo - ultimo_angulo) > 2:
            angulo_enviado = enviar_angulo_servo(client, angulo)
            ultimo_angulo = angulo
            print(f"Servo → {angulo_enviado}°  |  Pos: {posicion_norm:.2f}")

        # --- HUD ---
        angulo_int = int(round(angulo))
        if angulo_int < 80:
            color = (0, 165, 255)
            etiqueta = "IZQUIERDA"
        elif angulo_int > 100:
            color = (255, 165, 0)
            etiqueta = "DERECHA"
        else:
            color = (0, 220, 0)
            etiqueta = "CENTRO"

        cv2.putText(frame, f"Servo: {angulo_int} deg", (10, 35),
                    cv2.FONT_HERSHEY_SIMPLEX, 1.0, color, 2)
        cv2.putText(frame, etiqueta, (10, 70),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, color, 2)

        # Barra indicadora inferior
        barra_x = int((angulo / 180.0) * width)
        cv2.line(frame, (barra_x, height - 30), (barra_x, height),
                 (0, 0, 255), 3)
        cv2.line(frame, (0, height - 15), (width, height - 15),
                 (100, 100, 100), 1)

        # Marcas de 0°, 90°, 180° en la barra
        for deg, lbl in [(0, "0"), (90, "90"), (180, "180")]:
            px = int((deg / 180.0) * width)
            cv2.line(frame, (px, height - 25), (px, height - 5),
                     (200, 200, 200), 1)
            cv2.putText(frame, lbl, (px - 10, height - 28),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.35, (200, 200, 200), 1)

        cv2.imshow("Control Difuso Ackermann", frame)
        cv2.imshow("Mascara Azul", mask)

        if cv2.waitKey(1) & 0xFF == 27:
            break

    cap.release()
    cv2.destroyAllWindows()
    client.loop_stop()
    client.disconnect()
    print("Sistema finalizado.")


if __name__ == "__main__":
    main()
