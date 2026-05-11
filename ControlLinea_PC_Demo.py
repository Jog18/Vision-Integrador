# -*- coding: utf-8 -*-
"""
Control Difuso para Carro Ackermann 1:10 — Demo en PC
Usa scikit-fuzzy (skfuzzy) para el sistema Mamdani
Detecta linea blanca sobre fondo negro usando webcam USB
Incluye panel de visualizacion del control difuso en tiempo real

@author: oreaj
"""

import cv2
import numpy as np
import skfuzzy as fuzz
from skfuzzy import control as ctrl


# ============================================================
# SISTEMA DIFUSO CON SCIKIT-FUZZY
# ============================================================

# Universos (globales para reutilizar en visualizacion)
UNI_POS = np.linspace(0, 1, 1001)
UNI_SERVO = np.linspace(80, 135, 56)

# Parametros de funciones de membresia de entrada
MF_IN = {
    'muy_izq':  ('trap', [0.0, 0.0, 0.05, 0.20]),
    'med_izq':  ('tri',  [0.05, 0.20, 0.35]),
    'poco_izq': ('tri',  [0.20, 0.35, 0.50]),
    'centro':   ('tri',  [0.35, 0.50, 0.65]),
    'poco_der': ('tri',  [0.50, 0.65, 0.80]),
    'med_der':  ('tri',  [0.65, 0.80, 0.95]),
    'muy_der':  ('trap', [0.80, 0.95, 1.0, 1.0]),
}

# Parametros de funciones de membresia de salida
MF_OUT = {
    'muy_izq':  [80, 80, 91],
    'med_izq':  [80, 91, 102],
    'poco_izq': [91, 102, 113],
    'centro':   [102, 113, 120],
    'poco_der': [113, 120, 127],
    'med_der':  [120, 127, 135],
    'muy_der':  [127, 135, 135],
}

NOMBRES_MF = ['muy_izq', 'med_izq', 'poco_izq', 'centro',
              'poco_der', 'med_der', 'muy_der']

COLORES_MF = [
    (255, 0, 0),      # muy_izq  - azul
    (255, 128, 0),     # med_izq  - azul claro
    (255, 255, 0),     # poco_izq - cyan
    (0, 255, 0),       # centro   - verde
    (0, 255, 255),     # poco_der - amarillo
    (0, 128, 255),     # med_der  - naranja
    (0, 0, 255),       # muy_der  - rojo
]


def crear_sistema_difuso():
    posicion = ctrl.Antecedent(UNI_POS, 'posicion')
    servo = ctrl.Consequent(UNI_SERVO, 'servo', defuzzify_method='centroid')

    posicion['muy_izq']  = fuzz.trapmf(posicion.universe, [0.0, 0.0, 0.05, 0.20])
    posicion['med_izq']  = fuzz.trimf(posicion.universe, [0.05, 0.20, 0.35])
    posicion['poco_izq'] = fuzz.trimf(posicion.universe, [0.20, 0.35, 0.50])
    posicion['centro']   = fuzz.trimf(posicion.universe, [0.35, 0.50, 0.65])
    posicion['poco_der'] = fuzz.trimf(posicion.universe, [0.50, 0.65, 0.80])
    posicion['med_der']  = fuzz.trimf(posicion.universe, [0.65, 0.80, 0.95])
    posicion['muy_der']  = fuzz.trapmf(posicion.universe, [0.80, 0.95, 1.0, 1.0])

    servo['muy_izq']  = fuzz.trimf(servo.universe, [80, 80, 91])
    servo['med_izq']  = fuzz.trimf(servo.universe, [80, 91, 102])
    servo['poco_izq'] = fuzz.trimf(servo.universe, [91, 102, 113])
    servo['centro']   = fuzz.trimf(servo.universe, [102, 113, 120])
    servo['poco_der'] = fuzz.trimf(servo.universe, [113, 120, 127])
    servo['med_der']  = fuzz.trimf(servo.universe, [120, 127, 135])
    servo['muy_der']  = fuzz.trimf(servo.universe, [127, 135, 135])

    r1 = ctrl.Rule(posicion['muy_izq'], servo['muy_izq'])
    r2 = ctrl.Rule(posicion['med_izq'], servo['med_izq'])
    r3 = ctrl.Rule(posicion['poco_izq'], servo['poco_izq'])
    r4 = ctrl.Rule(posicion['centro'], servo['centro'])
    r5 = ctrl.Rule(posicion['poco_der'], servo['poco_der'])
    r6 = ctrl.Rule(posicion['med_der'], servo['med_der'])
    r7 = ctrl.Rule(posicion['muy_der'], servo['muy_der'])

    sistema = ctrl.ControlSystem([r1, r2, r3, r4, r5, r6, r7])
    simulacion = ctrl.ControlSystemSimulation(sistema)

    return simulacion


def fuzzificar_entrada(pos):
    """Calcula el grado de pertenencia de pos en cada conjunto de entrada."""
    grados = {}
    for nombre, (tipo, params) in MF_IN.items():
        if tipo == 'trap':
            mu = float(fuzz.trapmf(np.array([pos]), params)[0])
        else:
            mu = float(fuzz.trimf(np.array([pos]), params)[0])
        grados[nombre] = mu
    return grados


# ============================================================
# CONFIGURACION
# ============================================================

SERVO_MIN = 80
SERVO_MAX = 135
SERVO_RECTO = 113

UMBRAL_BLANCO = 200
MIN_AREA = 300
ROI_PROPORCION = 0.4

# Dimensiones del panel de visualizacion difusa
PANEL_W = 640
PANEL_H = 520


# ============================================================
# DIBUJO DEL PANEL DIFUSO
# ============================================================

def dibujar_grafica_mf(panel, x0, y0, w, h, universe, mf_dict, nombres,
                       colores, titulo, valor_actual, grados, es_salida=False,
                       resultado_centroide=None, aggregated=None):
    """Dibuja funciones de membresia con activaciones en una region del panel."""
    # Fondo
    cv2.rectangle(panel, (x0, y0), (x0 + w, y0 + h), (30, 30, 30), -1)
    cv2.rectangle(panel, (x0, y0), (x0 + w, y0 + h), (80, 80, 80), 1)

    # Titulo
    cv2.putText(panel, titulo, (x0 + 10, y0 + 20),
                cv2.FONT_HERSHEY_SIMPLEX, 0.55, (220, 220, 220), 1)

    # Area de dibujo
    margen = 40
    gx0 = x0 + margen
    gy0 = y0 + 35
    gw = w - margen - 15
    gh = h - 55

    # Eje horizontal
    cv2.line(panel, (gx0, gy0 + gh), (gx0 + gw, gy0 + gh), (100, 100, 100), 1)
    # Eje vertical
    cv2.line(panel, (gx0, gy0), (gx0, gy0 + gh), (100, 100, 100), 1)

    # Etiquetas del eje Y
    for val_y in [0.0, 0.5, 1.0]:
        py = int(gy0 + gh - val_y * gh)
        cv2.line(panel, (gx0 - 3, py), (gx0, py), (100, 100, 100), 1)
        cv2.putText(panel, f"{val_y:.1f}", (x0 + 2, py + 4),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.3, (150, 150, 150), 1)

    u_min = universe[0]
    u_max = universe[-1]

    # Etiquetas del eje X
    for val_x in np.linspace(u_min, u_max, 6):
        px = int(gx0 + (val_x - u_min) / (u_max - u_min) * gw)
        cv2.line(panel, (px, gy0 + gh), (px, gy0 + gh + 3), (100, 100, 100), 1)
        label = f"{val_x:.2f}" if u_max <= 1 else f"{val_x:.0f}"
        cv2.putText(panel, label, (px - 10, gy0 + gh + 14),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.3, (150, 150, 150), 1)

    # Dibujar cada funcion de membresia
    for idx, nombre in enumerate(nombres):
        if es_salida:
            mf_vals = fuzz.trimf(universe, mf_dict[nombre])
        else:
            tipo, params = mf_dict[nombre]
            if tipo == 'trap':
                mf_vals = fuzz.trapmf(universe, params)
            else:
                mf_vals = fuzz.trimf(universe, params)

        color = colores[idx]
        mu = grados.get(nombre, 0.0)

        # Color atenuado si no esta activo
        if mu < 0.01:
            draw_color = tuple(int(c * 0.3) for c in color)
            thickness = 1
        else:
            draw_color = color
            thickness = 2

        # Trazar la curva
        n_pts = min(200, len(universe))
        step = max(1, len(universe) // n_pts)
        pts = []
        for i in range(0, len(universe), step):
            px = int(gx0 + (universe[i] - u_min) / (u_max - u_min) * gw)
            py = int(gy0 + gh - mf_vals[i] * gh)
            pts.append((px, py))
        for j in range(len(pts) - 1):
            cv2.line(panel, pts[j], pts[j + 1], draw_color, thickness)

        # Rellenar area activada (recortada al nivel mu)
        if mu > 0.01:
            fill_pts = []
            for i in range(0, len(universe), step):
                val = min(mf_vals[i], mu)
                px = int(gx0 + (universe[i] - u_min) / (u_max - u_min) * gw)
                py = int(gy0 + gh - val * gh)
                fill_pts.append([px, py])
            # Cerrar por abajo
            if fill_pts:
                fill_pts.append([fill_pts[-1][0], gy0 + gh])
                fill_pts.append([fill_pts[0][0], gy0 + gh])
                overlay = panel.copy()
                cv2.fillPoly(overlay, [np.array(fill_pts)],
                             tuple(int(c * 0.4) for c in color))
                cv2.addWeighted(overlay, 0.5, panel, 0.5, 0, panel)

    # Linea vertical en el valor actual
    if valor_actual is not None:
        vx = int(gx0 + (valor_actual - u_min) / (u_max - u_min) * gw)
        cv2.line(panel, (vx, gy0), (vx, gy0 + gh), (0, 255, 255), 2)
        label = f"{valor_actual:.2f}" if u_max <= 1 else f"{valor_actual:.1f}"
        cv2.putText(panel, label, (vx + 4, gy0 + 12),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 255, 255), 1)

    # Centroide (solo salida)
    if resultado_centroide is not None:
        cx = int(gx0 + (resultado_centroide - u_min) / (u_max - u_min) * gw)
        cv2.line(panel, (cx, gy0), (cx, gy0 + gh), (0, 0, 255), 2)
        # Triangulo marcador
        cv2.drawMarker(panel, (cx, gy0 + gh + 3), (0, 0, 255),
                       cv2.MARKER_TRIANGLE_UP, 10, 2)
        cv2.putText(panel, f"CoG: {resultado_centroide:.1f}",
                    (cx + 5, gy0 + 12),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 0, 255), 1)


def dibujar_barras_reglas(panel, x0, y0, w, h, grados_in, grados_out):
    """Dibuja barras de activacion de las 7 reglas."""
    cv2.rectangle(panel, (x0, y0), (x0 + w, y0 + h), (30, 30, 30), -1)
    cv2.rectangle(panel, (x0, y0), (x0 + w, y0 + h), (80, 80, 80), 1)

    cv2.putText(panel, "ACTIVACION DE REGLAS", (x0 + 10, y0 + 20),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (220, 220, 220), 1)

    barra_h = 16
    espacio = 4
    margen_x = 110
    max_barra_w = w - margen_x - 50

    etiquetas_regla = [
        "R1: muy_izq",
        "R2: med_izq",
        "R3: poco_izq",
        "R4: centro",
        "R5: poco_der",
        "R6: med_der",
        "R7: muy_der",
    ]

    for i, nombre in enumerate(NOMBRES_MF):
        by = y0 + 35 + i * (barra_h + espacio)
        mu = grados_in.get(nombre, 0.0)

        # Etiqueta
        cv2.putText(panel, etiquetas_regla[i], (x0 + 5, by + 12),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.35, (180, 180, 180), 1)

        # Fondo de barra
        cv2.rectangle(panel, (x0 + margen_x, by),
                      (x0 + margen_x + max_barra_w, by + barra_h),
                      (50, 50, 50), -1)

        # Barra de activacion
        bw = int(mu * max_barra_w)
        if bw > 0:
            color = COLORES_MF[i]
            cv2.rectangle(panel, (x0 + margen_x, by),
                          (x0 + margen_x + bw, by + barra_h), color, -1)

        # Valor numerico
        cv2.putText(panel, f"{mu:.2f}",
                    (x0 + margen_x + max_barra_w + 5, by + 12),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.35, (200, 200, 200), 1)


def crear_panel_difuso(posicion_norm, angulo):
    """Crea el panel completo de visualizacion del control difuso."""
    panel = np.zeros((PANEL_H, PANEL_W, 3), dtype=np.uint8)

    # Titulo principal
    cv2.putText(panel, "CONTROL DIFUSO MAMDANI - TIEMPO REAL",
                (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0, 200, 255), 2)
    cv2.line(panel, (10, 32), (PANEL_W - 10, 32), (0, 200, 255), 1)

    # Calcular fuzzificacion
    grados_in = fuzzificar_entrada(posicion_norm)

    # Los grados de salida son iguales (reglas directas 1 a 1)
    grados_out = {}
    for nombre in NOMBRES_MF:
        grados_out[nombre] = grados_in[nombre]

    # 1) Funciones de membresia de ENTRADA (arriba izquierda)
    dibujar_grafica_mf(panel, 5, 40, PANEL_W // 2 - 10, 200,
                       UNI_POS, MF_IN, NOMBRES_MF, COLORES_MF,
                       "ENTRADA: Posicion [0-1] (Fuzzificacion)",
                       posicion_norm, grados_in)

    # 2) Funciones de membresia de SALIDA (arriba derecha)
    dibujar_grafica_mf(panel, PANEL_W // 2 + 5, 40, PANEL_W // 2 - 10, 200,
                       UNI_SERVO, MF_OUT, NOMBRES_MF, COLORES_MF,
                       "SALIDA: Servo [80-135] (Defuzzificacion)",
                       None, grados_out, es_salida=True,
                       resultado_centroide=angulo)

    # 3) Barras de activacion de reglas (abajo)
    dibujar_barras_reglas(panel, 5, 250, PANEL_W - 10, 185,
                          grados_in, grados_out)

    # 4) Resumen numerico inferior
    y_res = 450
    cv2.rectangle(panel, (5, y_res), (PANEL_W - 5, PANEL_H - 5), (30, 30, 30), -1)
    cv2.rectangle(panel, (5, y_res), (PANEL_W - 5, PANEL_H - 5), (80, 80, 80), 1)

    cv2.putText(panel, f"Entrada (posicion): {posicion_norm:.3f}",
                (15, y_res + 22), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1)

    cv2.putText(panel, f"Salida (servo): {angulo:.1f} grados",
                (15, y_res + 47), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1)

    # Reglas activas
    activas = [n for n in NOMBRES_MF if grados_in[n] > 0.01]
    txt_activas = ", ".join(activas) if activas else "ninguna"
    cv2.putText(panel, f"Conjuntos activos: {txt_activas}",
                (300, y_res + 22), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (180, 255, 180), 1)

    metodo = "Centroide (Centro de Gravedad)"
    cv2.putText(panel, f"Defuzzificacion: {metodo}",
                (300, y_res + 47), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (180, 180, 255), 1)

    return panel


# ============================================================
# BUCLE PRINCIPAL DE VISION + CONTROL
# ============================================================

def main():
    simulacion = crear_sistema_difuso()
    print("Sistema difuso skfuzzy creado.")

    # Webcam USB (cambiar indice si tienes varias camaras)
    cap = cv2.VideoCapture(0)
    if not cap.isOpened():
        print("Error: No se pudo abrir la camara.")
        return
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

    print("Demo control difuso en PC. ESC para salir.")

    while True:
        ret, frame = cap.read()
        if not ret:
            break

        height, width = frame.shape[:2]

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

        # --- Visualizacion sobre el frame ---
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

                    simulacion.input['posicion'] = max(0.0, min(1.0, posicion_norm))
                    simulacion.compute()
                    angulo = simulacion.output['servo']
                    linea_detectada = True

                    cy_frame = cy + roi_y
                    cv2.drawContours(frame[roi_y:height, :], [mayor], -1,
                                     (255, 0, 0), 2)
                    cv2.circle(frame, (cx, cy_frame), 6, (0, 0, 255), -1)
                    cv2.putText(frame, f"Pos: {posicion_norm:.2f}",
                                (cx + 10, cy_frame - 10),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                                (255, 255, 0), 1)

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

        # --- Panel de control difuso ---
        panel = crear_panel_difuso(posicion_norm, angulo)

        # Mostrar ventanas
        cv2.imshow("Seguidor de Linea - Camara", frame)
        cv2.imshow("Umbral Linea (ROI)", thresh)
        cv2.imshow("Control Difuso - Visualizacion", panel)

        if cv2.waitKey(1) & 0xFF == 27:
            break

    cap.release()
    cv2.destroyAllWindows()
    print("Sistema finalizado.")


if __name__ == "__main__":
    main()
