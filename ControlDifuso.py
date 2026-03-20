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

    Conjuntos difusos de entrada (3):
        Izquierda, Centro, Derecha

    Conjuntos difusos de salida (7):
        Muy_Izquierda, Med_Izquierda, Poco_Izquierda,
        Centro,
        Poco_Derecha, Med_Derecha, Muy_Derecha
    """

    def __init__(self):
        # Universo de salida discretizado (resolución de 1 grado)
        self.output_universe = np.linspace(0, 180, 181)

        # ---- Conjuntos difusos de ENTRADA [0, 1] ----
        # Izquierda: trapezoidal, máximo de 0 a 0.15, cae hasta 0.45
        self.mf_in_izquierda_params = [0.0, 0.0, 0.15, 0.45]
        # Centro: triangular, sube desde 0.25, pico en 0.5, baja hasta 0.75
        self.mf_in_centro_params = [0.25, 0.5, 0.75]
        # Derecha: trapezoidal, sube desde 0.55, máximo de 0.85 a 1.0
        self.mf_in_derecha_params = [0.55, 0.85, 1.0, 1.0]

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
        a cada conjunto difuso de entrada.
        """
        mu_izq = trapmf_scalar(pos, self.mf_in_izquierda_params)
        mu_cen = trimf_scalar(pos, self.mf_in_centro_params)
        mu_der = trapmf_scalar(pos, self.mf_in_derecha_params)
        return mu_izq, mu_cen, mu_der

    # ----------------------------------------------------------------
    # PASO 2: EVALUACIÓN DE REGLAS (Base de Conocimiento + Inferencia)
    # ----------------------------------------------------------------
    def evaluar_reglas(self, mu_izq, mu_cen, mu_der):
        """
        Evalúa las 9 reglas difusas y retorna el nivel de activación
        para cada conjunto de salida.

        Reglas:
        R1: IF Izquierda AND NOT Centro   → Muy_Izquierda
        R2: IF Izquierda                  → Med_Izquierda
        R3: IF Izquierda AND Centro       → Poco_Izquierda
        R4: IF Centro AND NOT Izq AND NOT Der → Centro (centro puro)
        R5: IF Centro                     → Centro
        R6: IF Derecha AND Centro         → Poco_Derecha
        R7: IF Derecha                    → Med_Derecha
        R8: IF Derecha AND NOT Centro     → Muy_Derecha
        R9: IF NOT Izq AND NOT Der AND NOT Centro → Centro (seguridad)
        """
        # Operador AND = min,  NOT x = 1 - x

        r1 = min(mu_izq, 1.0 - mu_cen)          # Muy_Izquierda
        r2 = mu_izq                               # Med_Izquierda
        r3 = min(mu_izq, mu_cen)                  # Poco_Izquierda
        r4 = min(mu_cen, 1.0 - mu_izq, 1.0 - mu_der)  # Centro (puro)
        r5 = mu_cen                               # Centro
        r6 = min(mu_der, mu_cen)                  # Poco_Derecha
        r7 = mu_der                               # Med_Derecha
        r8 = min(mu_der, 1.0 - mu_cen)           # Muy_Derecha
        r9 = min(1.0 - mu_izq, 1.0 - mu_der, 1.0 - mu_cen)  # Centro (seguridad)

        # Agregar activaciones por conjunto de salida (OR = max)
        activaciones = {
            'muy_izq':  r1,
            'med_izq':  r2,
            'poco_izq': r3,
            'centro':   max(r4, r5, r9),
            'poco_der': r6,
            'med_der':  r7,
            'muy_der':  r8,
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
        mu_izq, mu_cen, mu_der = self.fuzzificar(posicion_norm)
        activaciones = self.evaluar_reglas(mu_izq, mu_cen, mu_der)
        angulo = self.defuzzificar(activaciones)
        return angulo


# ============================================================
# CONFIGURACIÓN
# ============================================================

BROKER = "10.91.115.191"
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

    ultimo_angulo = 90

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
