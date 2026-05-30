# -*- coding: utf-8 -*-
"""
Control Difuso para Carro Ackermann 1:10 — Seguidor de Linea
con Maquina de Estados para 3 estaciones de color (Rojo, Azul, Amarillo).
VERSION s3: Maniobra de estacionamiento (entrada + salida).

PISTA: 240x240cm, linea blanca sobre fondo negro, estaciones 50x50cm.
       Curvas de radio 80cm y 150cm. Margen exterior 35cm.

LOGICA:
  - Sigue linea blanca con control difuso (Mamdani, skfuzzy).
  - Detecta lineas de color perpendiculares a la pista (Rojo, Azul, Amarillo).
  - Cuenta lineas con anti-rebote (debouncing por transicion).
  - En la 3ra linea de un color: ejecuta MANIOBRA DE ESTACIONAMIENTO.
  - Rojo (descarga) / Azul (carga): espera 5 segundos y sale con maniobra.
  - Amarillo: reposo indefinido hasta comando externo (SCADA o boton).

ESTADOS:
  SEGUIR_LINEA       -> Seguimiento normal + deteccion/conteo de lineas
  ESTACIONANDO       -> Maniobra de entrada a la estacion (multi-paso)
  ESPERA_ESTACION    -> Rojo/Azul: motor detenido 5 segundos
  REPOSO_AMARILLO    -> Amarillo: motor detenido hasta comando externo
  SALIENDO_ESTACION  -> Maniobra de salida de la estacion (multi-paso)

@author: oreaj
"""

import cv2
import numpy as np
import skfuzzy as fuzz
from skfuzzy import control as ctrl
import paho.mqtt.client as mqtt
from picamera2 import Picamera2
import time


# ============================================================
# SISTEMA DIFUSO CON SCIKIT-FUZZY (SIMETRICO)
# ============================================================

def crear_sistema_difuso():
    """
    Crea el sistema de control difuso Mamdani.
    Entrada: posicion normalizada [0, 1]
    Salida: angulo del servo [65, 160] (simetrico, centro=113, +-48)
    """
    posicion = ctrl.Antecedent(np.linspace(0, 1, 1001), 'posicion')
    servo = ctrl.Consequent(np.linspace(65, 160, 96), 'servo', defuzzify_method='centroid')

    posicion['muy_izq']  = fuzz.trapmf(posicion.universe, [0.0, 0.0, 0.05, 0.20])
    posicion['med_izq']  = fuzz.trimf(posicion.universe, [0.05, 0.20, 0.35])
    posicion['poco_izq'] = fuzz.trimf(posicion.universe, [0.20, 0.35, 0.50])
    posicion['centro']   = fuzz.trimf(posicion.universe, [0.35, 0.50, 0.65])
    posicion['poco_der'] = fuzz.trimf(posicion.universe, [0.50, 0.65, 0.80])
    posicion['med_der']  = fuzz.trimf(posicion.universe, [0.65, 0.80, 0.95])
    posicion['muy_der']  = fuzz.trapmf(posicion.universe, [0.80, 0.95, 1.0, 1.0])

    servo['muy_izq']  = fuzz.trimf(servo.universe, [65,  65,  81])
    servo['med_izq']  = fuzz.trimf(servo.universe, [65,  81,  97])
    servo['poco_izq'] = fuzz.trimf(servo.universe, [81,  97,  113])
    servo['centro']   = fuzz.trimf(servo.universe, [97,  113, 129])
    servo['poco_der'] = fuzz.trimf(servo.universe, [113, 129, 145])
    servo['med_der']  = fuzz.trimf(servo.universe, [129, 145, 160])
    servo['muy_der']  = fuzz.trimf(servo.universe, [145, 160, 160])

    r1 = ctrl.Rule(posicion['muy_izq'], servo['muy_izq'])
    r2 = ctrl.Rule(posicion['med_izq'], servo['med_izq'])
    r3 = ctrl.Rule(posicion['poco_izq'], servo['poco_izq'])
    r4 = ctrl.Rule(posicion['centro'], servo['centro'])
    r5 = ctrl.Rule(posicion['poco_der'], servo['poco_der'])
    r6 = ctrl.Rule(posicion['med_der'], servo['med_der'])
    r7 = ctrl.Rule(posicion['muy_der'], servo['muy_der'])

    sistema = ctrl.ControlSystem([r1, r2, r3, r4, r5, r6, r7])
    return ctrl.ControlSystemSimulation(sistema)


# ============================================================
# CONFIGURACION GENERAL
# ============================================================

BROKER = "127.0.0.1"
PORT = 1883
TOPIC_SERVO = "esp32/servo/control"
TOPIC_ARRANQUE = "esp32/arranque"
TOPIC_ESTADO = "arranque/paro"
TOPIC_ESTACION = "agv/estacion"

SERVO_MIN = 65
SERVO_MAX = 160
SERVO_RECTO = 113

# --- Deteccion de linea blanca ---
UMBRAL_BLANCO = 200
MIN_AREA_LINEA = 300
ROI_PROPORCION = 1

# --- Deteccion de lineas de color ---
ROI_COLOR_INICIO = 0.55
ROI_COLOR_FIN = 0.85

RANGOS_HSV = {
    'Rojo_bajo': {'bajo': np.array([0,   120, 70]), 'alto': np.array([10,  255, 255])},
    'Rojo_alto': {'bajo': np.array([170, 120, 70]), 'alto': np.array([180, 255, 255])},
    'Azul':      {'bajo': np.array([100, 150,  0]), 'alto': np.array([140, 255, 255])},
    'Amarillo':  {'bajo': np.array([22,  130, 130]), 'alto': np.array([33,  255, 255])},
}

MIN_AREA_COLOR = 1500

LINEAS_PARA_ESTACIONAR = 3

# --- Tiempos ---
TIEMPO_ESPERA_ESTACION = 5.0
COOLDOWN_SALIDA = 3.0

# --- Nombres de zona por color ---
ZONAS = {
    'Azul': 'CARGA',
    'Rojo': 'DESCARGA',
    'Amarillo': 'REPOSO',
}


# ============================================================
# CONFIGURACION DE MANIOBRA DE ESTACIONAMIENTO (s3)
# ============================================================

# Lado de la pista donde se encuentra cada estacion.
# 'derecha' = la estacion esta a la derecha del sentido de marcha.
# 'izquierda' = la estacion esta a la izquierda.
# >>> AJUSTAR SEGUN LA PISTA REAL <<<
LADO_ESTACION = {
    'Rojo':     'derecha',
    'Azul':     'derecha',
    'Amarillo': 'derecha',
}

# Tiempos de cada paso de la maniobra (en segundos).
# >>> CALIBRAR EN LA PISTA REAL — dependen de la velocidad PWM y el radio de giro <<<
TIEMPO_AVANCE_PASALINEA   = 0.5   # avanzar recto para rebasar la linea de color
TIEMPO_GIRO_ENTRADA       = 1.5   # girar hacia la estacion (forward + servo al maximo)
TIEMPO_ENDEREZAR_ENTRADA  = 0.5   # enderezar dentro de la estacion (forward + servo recto)
TIEMPO_GIRO_SALIDA        = 1.5   # reversa girando de vuelta hacia la pista
TIEMPO_REVERSA_RECTA      = 0.5   # reversa recto para alinear con la pista
TIEMPO_AVANCE_BUSQUEDA    = 0.5   # avanzar recto buscando la linea blanca


def generar_maniobra_entrada(color):
    """
    Genera la secuencia de pasos para entrar a la estacion.
    Cada paso: (angulo_servo, comando_motor, duracion_seg, descripcion)
    """
    lado = LADO_ESTACION.get(color, 'derecha')
    angulo_giro = SERVO_MAX if lado == 'derecha' else SERVO_MIN

    return [
        (SERVO_RECTO, 'GO',   TIEMPO_AVANCE_PASALINEA,  'Avanzar recto para rebasar linea'),
        (angulo_giro, 'GO',   TIEMPO_GIRO_ENTRADA,      f'Girar {lado} entrando a estacion'),
        (SERVO_RECTO, 'GO',   TIEMPO_ENDEREZAR_ENTRADA, 'Enderezar dentro de estacion'),
        (SERVO_RECTO, 'STOP', 0.0,                      'Detenido en estacion'),
    ]


def generar_maniobra_salida(color):
    """
    Genera la secuencia de pasos para salir de la estacion.
    Cada paso: (angulo_servo, comando_motor, duracion_seg, descripcion)
    """
    lado = LADO_ESTACION.get(color, 'derecha')
    # Para salir, se gira al lado OPUESTO en reversa
    angulo_giro = SERVO_MIN if lado == 'derecha' else SERVO_MAX

    return [
        (angulo_giro, 'REVERSE', TIEMPO_GIRO_SALIDA,     f'Reversa girando hacia la pista'),
        (SERVO_RECTO, 'REVERSE', TIEMPO_REVERSA_RECTA,   'Reversa recto para alinear'),
        (SERVO_RECTO, 'GO',      TIEMPO_AVANCE_BUSQUEDA, 'Avanzar buscando linea blanca'),
    ]


# ============================================================
# ESTADOS DE LA MAQUINA DE ESTADOS
# ============================================================

SEGUIR_LINEA = "SEGUIR_LINEA"
ESTACIONANDO = "ESTACIONANDO"
ESPERA_ESTACION = "ESPERA_ESTACION"
REPOSO_AMARILLO = "REPOSO_AMARILLO"
SALIENDO_ESTACION = "SALIENDO_ESTACION"


# ============================================================
# VARIABLE GLOBAL MQTT
# ============================================================

estado_esp32 = "PARO"


def on_message(client, userdata, msg):
    global estado_esp32
    if msg.topic == TOPIC_ESTADO:
        nuevo = msg.payload.decode()
        if nuevo != estado_esp32:
            print(f"[MQTT] ESP32 estado: {estado_esp32} -> {nuevo}")
        estado_esp32 = nuevo


# ============================================================
# FUNCIONES DE CONTROL (via MQTT al ESP32)
# ============================================================

def enviar_angulo_servo(client, angulo):
    angulo_int = int(round(angulo))
    angulo_int = max(SERVO_MIN, min(SERVO_MAX, angulo_int))
    client.publish(TOPIC_SERVO, str(angulo_int))
    return angulo_int


def calcular_angulo(simulacion, posicion_norm):
    posicion_norm = max(0.0, min(1.0, posicion_norm))
    simulacion.input['posicion'] = posicion_norm
    simulacion.compute()
    return simulacion.output['servo']


def cmd_stop(client):
    """Envia comando STOP al ESP32."""
    client.publish(TOPIC_ARRANQUE, "STOP")
    print("[MOTOR] STOP")


def cmd_go(client):
    """Envia comando GO al ESP32 (motor hacia adelante)."""
    client.publish(TOPIC_ARRANQUE, "GO")
    print("[MOTOR] GO (adelante)")


def cmd_reverse(client):
    """Envia comando REVERSE al ESP32 (motor en reversa)."""
    client.publish(TOPIC_ARRANQUE, "REVERSE")
    print("[MOTOR] REVERSE")


def publicar_estacion(client, zona):
    """Publica la zona actual del AGV."""
    client.publish(TOPIC_ESTACION, zona)
    print(f"[ESTACION] Publicado zona: {zona}")


# ============================================================
# MATRIZ DE TRANSFORMACION — Correccion de perspectiva de camara
# ============================================================

SRC_POINTS = np.float32([[0, 0], [640, 0], [0, 144]])
DST_POINTS = np.float32([[10, 0], [630, 0], [0, 144]])
MATRIZ_PERSPECTIVA = cv2.getAffineTransform(SRC_POINTS, DST_POINTS)


# ============================================================
# FUNCIONES DE PROCESAMIENTO DE IMAGEN
# ============================================================

def detectar_linea_blanca(frame, roi_y, width, height):
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    roi_gray = gray[roi_y:height, :]

    _, thresh = cv2.threshold(roi_gray, UMBRAL_BLANCO, 255, cv2.THRESH_BINARY)
    kernel = np.ones((3, 3), np.uint8)
    thresh = cv2.morphologyEx(thresh, cv2.MORPH_OPEN, kernel)
    thresh = cv2.morphologyEx(thresh, cv2.MORPH_CLOSE, kernel)

    contours, _ = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    if contours:
        mayor = max(contours, key=cv2.contourArea)
        area = cv2.contourArea(mayor)
        if area > MIN_AREA_LINEA:
            M = cv2.moments(mayor)
            if M["m00"] > 0:
                cx = int(M["m10"] / M["m00"])
                posicion_norm = cx / width
                return True, posicion_norm, mayor, thresh

    return False, 0.5, None, thresh


def detectar_lineas_color(frame, height):
    y_ini = int(height * ROI_COLOR_INICIO)
    y_fin = int(height * ROI_COLOR_FIN)
    roi = frame[y_ini:y_fin, :]

    roi_w = roi.shape[1]
    roi_h = roi.shape[0]
    roi = cv2.warpAffine(roi, MATRIZ_PERSPECTIVA, (roi_w, roi_h))

    roi_filtrado = cv2.GaussianBlur(roi, (7, 7), 0)
    hsv = cv2.cvtColor(roi_filtrado, cv2.COLOR_BGR2HSV)

    kernel = np.ones((7, 7), np.uint8)
    visibles = {}

    # --- Rojo: combinar DOS mascaras ---
    mascara_rojo1 = cv2.inRange(hsv, RANGOS_HSV['Rojo_bajo']['bajo'],
                                     RANGOS_HSV['Rojo_bajo']['alto'])
    mascara_rojo2 = cv2.inRange(hsv, RANGOS_HSV['Rojo_alto']['bajo'],
                                     RANGOS_HSV['Rojo_alto']['alto'])
    mascara_rojo = cv2.bitwise_or(mascara_rojo1, mascara_rojo2)
    mascara_rojo = cv2.morphologyEx(mascara_rojo, cv2.MORPH_OPEN, kernel)
    mascara_rojo = cv2.morphologyEx(mascara_rojo, cv2.MORPH_CLOSE, kernel)

    contours, _ = cv2.findContours(mascara_rojo, cv2.RETR_EXTERNAL,
                                   cv2.CHAIN_APPROX_SIMPLE)
    detectado_rojo = False
    if contours:
        mayor = max(contours, key=cv2.contourArea)
        if cv2.contourArea(mayor) > MIN_AREA_COLOR:
            detectado_rojo = True
            x, y, w, h = cv2.boundingRect(mayor)
            cv2.rectangle(frame, (x, y + y_ini), (x + w, y + h + y_ini),
                          (0, 0, 255), 2)
            cv2.putText(frame, "Rojo", (x, y + y_ini - 5),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)
    visibles['Rojo'] = detectado_rojo

    # --- Azul y Amarillo ---
    for color in ['Azul', 'Amarillo']:
        rango = RANGOS_HSV[color]
        mascara = cv2.inRange(hsv, rango['bajo'], rango['alto'])
        mascara = cv2.morphologyEx(mascara, cv2.MORPH_OPEN, kernel)
        mascara = cv2.morphologyEx(mascara, cv2.MORPH_CLOSE, kernel)
        if color == 'Amarillo':
            mascara = cv2.erode(mascara, np.ones((3, 3), np.uint8), iterations=1)

        contours, _ = cv2.findContours(mascara, cv2.RETR_EXTERNAL,
                                       cv2.CHAIN_APPROX_SIMPLE)
        detectado = False
        if contours:
            mayor = max(contours, key=cv2.contourArea)
            if cv2.contourArea(mayor) > MIN_AREA_COLOR:
                detectado = True
                x, y, w, h = cv2.boundingRect(mayor)
                cv2.rectangle(frame, (x, y + y_ini), (x + w, y + h + y_ini),
                              (0, 255, 255), 2)
                cv2.putText(frame, color, (x, y + y_ini - 5),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 2)
        visibles[color] = detectado

    return visibles


# ============================================================
# FUNCIONES DE VISUALIZACION (HUD)
# ============================================================

def dibujar_hud_seguimiento(frame, angulo, linea_detectada, posicion_norm,
                            contorno_mayor, roi_y, width, height,
                            contadores, linea_visible_flags):
    tercio_izq = width // 3
    tercio_der = 2 * width // 3

    overlay = frame.copy()
    cv2.rectangle(overlay, (tercio_izq, 0), (tercio_der, height), (0, 255, 0), -1)
    cv2.addWeighted(overlay, 0.10, frame, 0.90, 0, frame)

    cv2.line(frame, (tercio_izq, 0), (tercio_izq, height), (0, 255, 0), 1)
    cv2.line(frame, (tercio_der, 0), (tercio_der, height), (0, 255, 0), 1)
    cv2.line(frame, (width // 2, 0), (width // 2, height), (200, 200, 200), 1)

    cv2.line(frame, (0, roi_y), (width, roi_y), (0, 200, 200), 1)
    cv2.putText(frame, "ROI", (5, roi_y - 5),
                cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 200, 200), 1)

    y_ini_color = int(height * ROI_COLOR_INICIO)
    y_fin_color = int(height * ROI_COLOR_FIN)
    cv2.line(frame, (0, y_ini_color), (width, y_ini_color), (255, 0, 255), 1)
    cv2.line(frame, (0, y_fin_color), (width, y_fin_color), (255, 0, 255), 1)
    cv2.putText(frame, "COLOR ROI", (5, y_ini_color - 5),
                cv2.FONT_HERSHEY_SIMPLEX, 0.35, (255, 0, 255), 1)

    if linea_detectada and contorno_mayor is not None:
        M = cv2.moments(contorno_mayor)
        if M["m00"] > 0:
            cx = int(M["m10"] / M["m00"])
            cy = int(M["m01"] / M["m00"])
            cy_frame = cy + roi_y
            cv2.drawContours(frame[roi_y:height, :], [contorno_mayor], -1, (255, 0, 0), 2)
            cv2.circle(frame, (cx, cy_frame), 6, (0, 0, 255), -1)
            cv2.putText(frame, f"Pos: {posicion_norm:.2f}",
                        (cx + 10, cy_frame - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 1)

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

    y_texto = 140
    colores_bgr = {'Rojo': (0, 0, 255), 'Azul': (255, 100, 0), 'Amarillo': (0, 255, 255)}
    for nombre_color, cnt in contadores.items():
        indicador = " <<" if linea_visible_flags.get(nombre_color, False) else ""
        cv2.putText(frame, f"{nombre_color}: {cnt}/{LINEAS_PARA_ESTACIONAR}{indicador}",
                    (10, y_texto), cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                    colores_bgr[nombre_color], 2)
        y_texto += 25

    mitad = width // 2
    rango_izq = SERVO_RECTO - SERVO_MIN
    rango_der = SERVO_MAX - SERVO_RECTO

    if angulo <= SERVO_RECTO:
        barra_x = int(((angulo - SERVO_MIN) / rango_izq) * mitad)
    else:
        barra_x = mitad + int(((angulo - SERVO_RECTO) / rango_der) * mitad)

    cv2.line(frame, (barra_x, height - 30), (barra_x, height), (0, 0, 255), 3)
    cv2.line(frame, (0, height - 15), (width, height - 15), (100, 100, 100), 1)

    for deg, lbl in [(SERVO_MIN, "65"), (SERVO_RECTO, "113"), (SERVO_MAX, "160")]:
        if deg <= SERVO_RECTO:
            px = int(((deg - SERVO_MIN) / rango_izq) * mitad)
        else:
            px = mitad + int(((deg - SERVO_RECTO) / rango_der) * mitad)
        cv2.line(frame, (px, height - 25), (px, height - 5), (200, 200, 200), 1)
        cv2.putText(frame, lbl, (px - 10, height - 28),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.35, (200, 200, 200), 1)


def dibujar_hud_maniobra(frame, titulo, paso_actual, total_pasos, descripcion,
                          progreso, color_estacion):
    """HUD para los estados ESTACIONANDO y SALIENDO_ESTACION."""
    colores_bgr = {'Rojo': (0, 0, 255), 'Azul': (255, 100, 0), 'Amarillo': (0, 255, 255)}
    color = colores_bgr.get(color_estacion, (255, 255, 255))

    cv2.putText(frame, titulo, (10, 35),
                cv2.FONT_HERSHEY_SIMPLEX, 0.9, color, 2)
    cv2.putText(frame, f"Estacion: {color_estacion} ({ZONAS.get(color_estacion, '')})",
                (10, 65), cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 1)
    cv2.putText(frame, f"Paso {paso_actual + 1}/{total_pasos}: {descripcion}",
                (10, 100), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (200, 200, 200), 1)

    # Barra de progreso del paso actual
    barra_x = 10
    barra_y = 115
    barra_w = 300
    barra_h = 15
    cv2.rectangle(frame, (barra_x, barra_y),
                  (barra_x + barra_w, barra_y + barra_h), (80, 80, 80), -1)
    progreso_w = int(barra_w * min(1.0, progreso))
    if progreso_w > 0:
        cv2.rectangle(frame, (barra_x, barra_y),
                      (barra_x + progreso_w, barra_y + barra_h), color, -1)
    cv2.rectangle(frame, (barra_x, barra_y),
                  (barra_x + barra_w, barra_y + barra_h), (150, 150, 150), 1)


def dibujar_hud_estado(frame, estado, info_extra=""):
    colores_estado = {
        SEGUIR_LINEA:      (0, 220, 0),
        ESTACIONANDO:      (0, 180, 255),
        ESPERA_ESTACION:   (0, 255, 255),
        REPOSO_AMARILLO:   (0, 200, 255),
        SALIENDO_ESTACION: (180, 220, 0),
    }
    color = colores_estado.get(estado, (255, 255, 255))
    cv2.putText(frame, f"Pi: {estado}", (10, 470),
                cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)
    cv2.putText(frame, f"ESP32: {estado_esp32}", (10, 440),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (180, 180, 180), 1)
    if info_extra:
        cv2.putText(frame, info_extra, (10, 415),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1)


# ============================================================
# FUNCION AUXILIAR: Ejecutar un paso de maniobra
# ============================================================

def ejecutar_paso_maniobra(client, paso):
    """Envia los comandos MQTT correspondientes a un paso de maniobra."""
    angulo, comando, _, descripcion = paso

    enviar_angulo_servo(client, angulo)

    if comando == 'STOP':
        cmd_stop(client)
    elif comando == 'REVERSE':
        cmd_reverse(client)
    else:
        cmd_go(client)

    print(f"[MANIOBRA] {descripcion} | Servo={angulo} | Motor={comando}")


# ============================================================
# BUCLE PRINCIPAL — MAQUINA DE ESTADOS
# ============================================================

def main():
    global estado_esp32

    # --- Inicializar sistema difuso ---
    simulacion = crear_sistema_difuso()
    print("Sistema difuso skfuzzy creado (SIMETRICO 65-113-160).")

    # --- Inicializar camara ---
    picam2 = Picamera2()
    config = picam2.create_preview_configuration(main={"size": (640, 480)})
    picam2.configure(config)
    picam2.start()

    # --- Inicializar MQTT ---
    client = mqtt.Client()
    client.on_message = on_message
    try:
        client.connect(BROKER, PORT, 60)
        client.subscribe(TOPIC_ESTADO)
        client.loop_start()
    except Exception as e:
        print(f"Advertencia: No se pudo conectar al broker MQTT ({e})")

    # --- Variables de estado ---
    estado = SEGUIR_LINEA
    ultimo_angulo = SERVO_RECTO
    ultima_posicion = 0.5

    contadores = {'Rojo': 0, 'Azul': 0, 'Amarillo': 0}
    linea_visible = {'Rojo': False, 'Azul': False, 'Amarillo': False}

    color_estacion_actual = None
    tiempo_inicio_espera = 0.0
    tiempo_salida = 0.0
    en_cooldown = False

    # --- Variables de maniobra (s3) ---
    maniobra_pasos = []        # lista de pasos de la maniobra actual
    maniobra_idx = 0           # indice del paso actual
    maniobra_inicio_paso = 0.0 # timestamp de inicio del paso actual
    maniobra_paso_iniciado = False

    # Publicar estado inicial
    publicar_estacion(client, "EN_RUTA")

    print("=" * 60)
    print("MAQUINA DE ESTADOS — Seguidor de Linea + Estacionamiento")
    print(f"  Estacionar en la linea #{LINEAS_PARA_ESTACIONAR} de cada color")
    print(f"  Rojo (descarga) / Azul (carga): espera {TIEMPO_ESPERA_ESTACION}s")
    print(f"  Amarillo: reposo indefinido")
    print(f"  Maniobra de entrada y salida habilitada")
    print("  ESC para salir")
    print("=" * 60)

    while True:
        frame = picam2.capture_array()
        height, width = frame.shape[:2]

        if frame.shape[2] == 4:
            frame = cv2.cvtColor(frame, cv2.COLOR_RGBA2BGR)
        else:
            frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)

        roi_y = int(height * (1.0 - ROI_PROPORCION))

        # ==============================================================
        # ESTADO: SEGUIR_LINEA
        # ==============================================================
        if estado == SEGUIR_LINEA:

            # --- Deteccion de lineas de color ---
            if en_cooldown:
                if time.time() - tiempo_salida > COOLDOWN_SALIDA:
                    en_cooldown = False
                    print("[COOLDOWN] Finalizado. Deteccion de color reactivada.")
                visibles = {'Rojo': False, 'Azul': False, 'Amarillo': False}
            else:
                visibles = detectar_lineas_color(frame, height)

            # --- Anti-rebote y conteo ---
            color_que_para = None

            for color in contadores:
                if visibles[color] and not linea_visible[color]:
                    contadores[color] += 1
                    linea_visible[color] = True
                    print(f"[CONTEO] Linea {color} detectada: "
                          f"{contadores[color]}/{LINEAS_PARA_ESTACIONAR}")

                    if contadores[color] >= LINEAS_PARA_ESTACIONAR:
                        color_que_para = color

                elif not visibles[color] and linea_visible[color]:
                    linea_visible[color] = False

            # --- Verificar si hay que estacionar ---
            if color_que_para is not None:
                color_estacion_actual = color_que_para
                contadores[color_que_para] = 0
                linea_visible[color_que_para] = False

                # Iniciar maniobra de entrada
                maniobra_pasos = generar_maniobra_entrada(color_estacion_actual)
                maniobra_idx = 0
                maniobra_paso_iniciado = False

                estado_esp32 = "PARO"  # forzar estado local
                publicar_estacion(client, "ESTACIONANDO")

                estado = ESTACIONANDO
                print(f"[FSM] {SEGUIR_LINEA} -> {ESTACIONANDO} "
                      f"({color_estacion_actual} = {ZONAS[color_estacion_actual]})")

                dibujar_hud_estado(frame, estado,
                                   f"Estacionando: {color_estacion_actual}")
                cv2.imshow("Seguidor de Linea", frame)
                cv2.waitKey(1)
                continue

            # --- Seguimiento de linea blanca ---
            detectada, posicion_norm, contorno, thresh = \
                detectar_linea_blanca(frame, roi_y, width, height)

            angulo = float(SERVO_RECTO)

            if detectada:
                ultima_posicion = posicion_norm
                angulo = calcular_angulo(simulacion, posicion_norm)
            else:
                if ultima_posicion < 0.5:
                    angulo = float(SERVO_MIN)
                else:
                    angulo = float(SERVO_MAX)

            if abs(angulo - ultimo_angulo) > 2:
                angulo_enviado = enviar_angulo_servo(client, angulo)
                ultimo_angulo = angulo
                if detectada:
                    print(f"Servo -> {angulo_enviado} deg | Pos: {posicion_norm:.2f}")
                else:
                    print(f"Servo -> {angulo_enviado} deg | BUSCANDO LINEA")

            # --- Visualizacion ---
            dibujar_hud_seguimiento(frame, angulo, detectada, posicion_norm,
                                    contorno, roi_y, width, height,
                                    contadores, linea_visible)
            dibujar_hud_estado(frame, estado)
            cv2.imshow("Umbral Linea (ROI)", thresh)

        # ==============================================================
        # ESTADO: ESTACIONANDO (maniobra de entrada multi-paso)
        # ==============================================================
        elif estado == ESTACIONANDO:

            paso = maniobra_pasos[maniobra_idx]
            _, _, duracion, descripcion = paso

            # Iniciar el paso actual si no se ha iniciado
            if not maniobra_paso_iniciado:
                ejecutar_paso_maniobra(client, paso)
                maniobra_inicio_paso = time.time()
                maniobra_paso_iniciado = True

            # Calcular progreso del paso
            if duracion > 0:
                transcurrido = time.time() - maniobra_inicio_paso
                progreso = transcurrido / duracion
            else:
                transcurrido = 0
                progreso = 1.0

            # HUD de maniobra
            dibujar_hud_maniobra(frame, "ESTACIONANDO",
                                 maniobra_idx, len(maniobra_pasos),
                                 descripcion, progreso, color_estacion_actual)
            dibujar_hud_estado(frame, estado,
                               f"Paso {maniobra_idx + 1}/{len(maniobra_pasos)}")

            # Verificar si el paso termino
            if duracion == 0 or transcurrido >= duracion:
                maniobra_idx += 1
                maniobra_paso_iniciado = False

                # Verificar si la maniobra completa termino
                if maniobra_idx >= len(maniobra_pasos):
                    cmd_stop(client)
                    zona = ZONAS[color_estacion_actual]
                    publicar_estacion(client, zona)

                    if color_estacion_actual == 'Amarillo':
                        estado = REPOSO_AMARILLO
                        print(f"[FSM] {ESTACIONANDO} -> {REPOSO_AMARILLO}")
                    else:
                        estado = ESPERA_ESTACION
                        tiempo_inicio_espera = time.time()
                        print(f"[FSM] {ESTACIONANDO} -> {ESPERA_ESTACION} "
                              f"({color_estacion_actual} = {zona}, "
                              f"{TIEMPO_ESPERA_ESTACION}s)")

        # ==============================================================
        # ESTADO: ESPERA_ESTACION (Rojo / Azul)
        # Motor detenido durante TIEMPO_ESPERA_ESTACION segundos.
        # Al terminar, inicia maniobra de SALIDA.
        # ==============================================================
        elif estado == ESPERA_ESTACION:
            transcurrido = time.time() - tiempo_inicio_espera
            restante = max(0, TIEMPO_ESPERA_ESTACION - transcurrido)

            zona = ZONAS.get(color_estacion_actual, "")
            cv2.putText(frame, f"ESTACION — {color_estacion_actual} ({zona})", (10, 35),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 255), 2)
            cv2.putText(frame, f"Esperando: {restante:.1f}s", (10, 70),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 200, 200), 2)
            dibujar_hud_estado(frame, estado, f"Salida en {restante:.1f}s")

            if transcurrido >= TIEMPO_ESPERA_ESTACION:
                # Iniciar maniobra de salida
                maniobra_pasos = generar_maniobra_salida(color_estacion_actual)
                maniobra_idx = 0
                maniobra_paso_iniciado = False

                publicar_estacion(client, "SALIENDO")

                estado = SALIENDO_ESTACION
                print(f"[FSM] {ESPERA_ESTACION} -> {SALIENDO_ESTACION}")

        # ==============================================================
        # ESTADO: REPOSO_AMARILLO
        # Motor detenido de forma indefinida.
        # Sale cuando el ESP32 reporta MOVIMIENTO (por SCADA GO o boton ON).
        # Al salir, inicia maniobra de SALIDA.
        # ==============================================================
        elif estado == REPOSO_AMARILLO:
            cv2.putText(frame, "REPOSO — Estacion Amarilla", (10, 35),
                        cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 200, 255), 2)
            cv2.putText(frame, "Esperando comando externo...", (10, 70),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 200, 200), 2)
            cv2.putText(frame, "(SCADA -> ARRANQUE o boton ON)", (10, 100),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (180, 180, 180), 1)
            dibujar_hud_estado(frame, estado)

            if estado_esp32 == "MOVIMIENTO":
                # Iniciar maniobra de salida
                maniobra_pasos = generar_maniobra_salida(color_estacion_actual)
                maniobra_idx = 0
                maniobra_paso_iniciado = False

                publicar_estacion(client, "SALIENDO")

                estado = SALIENDO_ESTACION
                print(f"[FSM] {REPOSO_AMARILLO} -> {SALIENDO_ESTACION}")

        # ==============================================================
        # ESTADO: SALIENDO_ESTACION (maniobra de salida multi-paso)
        # Al terminar, transiciona a SEGUIR_LINEA con cooldown.
        # ==============================================================
        elif estado == SALIENDO_ESTACION:

            paso = maniobra_pasos[maniobra_idx]
            _, _, duracion, descripcion = paso

            if not maniobra_paso_iniciado:
                ejecutar_paso_maniobra(client, paso)
                maniobra_inicio_paso = time.time()
                maniobra_paso_iniciado = True

            if duracion > 0:
                transcurrido = time.time() - maniobra_inicio_paso
                progreso = transcurrido / duracion
            else:
                transcurrido = 0
                progreso = 1.0

            dibujar_hud_maniobra(frame, "SALIENDO DE ESTACION",
                                 maniobra_idx, len(maniobra_pasos),
                                 descripcion, progreso, color_estacion_actual)
            dibujar_hud_estado(frame, estado,
                               f"Paso {maniobra_idx + 1}/{len(maniobra_pasos)}")

            if duracion == 0 or transcurrido >= duracion:
                maniobra_idx += 1
                maniobra_paso_iniciado = False

                if maniobra_idx >= len(maniobra_pasos):
                    # Maniobra de salida completa -> volver a seguir linea
                    publicar_estacion(client, "EN_RUTA")

                    # Establecer ultima_posicion para buscar la linea
                    # del lado correcto al volver a SEGUIR_LINEA
                    lado = LADO_ESTACION.get(color_estacion_actual, 'derecha')
                    if lado == 'derecha':
                        ultima_posicion = 0.0  # buscar hacia la izquierda
                    else:
                        ultima_posicion = 1.0  # buscar hacia la derecha

                    estado = SEGUIR_LINEA
                    en_cooldown = True
                    tiempo_salida = time.time()
                    color_estacion_actual = None
                    ultimo_angulo = SERVO_RECTO
                    for c in linea_visible:
                        linea_visible[c] = False
                    print(f"[FSM] {SALIENDO_ESTACION} -> {SEGUIR_LINEA} "
                          f"(cooldown activo)")

        # --- Mostrar ventana principal ---
        cv2.imshow("Seguidor de Linea", frame)

        if cv2.waitKey(1) & 0xFF == 27:
            break

    # --- Limpieza ---
    cmd_stop(client)
    picam2.stop()
    cv2.destroyAllWindows()
    client.loop_stop()
    client.disconnect()
    print("Sistema finalizado.")


if __name__ == "__main__":
    main()
