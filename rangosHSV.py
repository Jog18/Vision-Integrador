# -*- coding: utf-8 -*-
"""
Calibrador de rangos HSV para Raspberry Pi con Picamera2
Permite ajustar en tiempo real los rangos HSV de 3 colores
(Rojo, Azul, Amarillo) usando trackbars para calibrar la
deteccion de colores de la Pi Camera.

Controles:
  - Trackbars para ajustar H, S, V minimo y maximo de cada color
  - Tecla 1/2/3: cambiar entre color Rojo/Azul/Amarillo
  - Tecla P: imprimir los rangos actuales en consola
  - Tecla ESC: salir

@author: oreaj
"""

import cv2
import numpy as np
from picamera2 import Picamera2

# ============================================================
# RANGOS HSV INICIALES (punto de partida para calibrar)
# ============================================================
rangos_colores = {
    'Rojo':     {'bajo': [0, 100, 100],   'alto': [10, 255, 255]},
    'Azul':     {'bajo': [100, 150, 0],   'alto': [140, 255, 255]},
    'Amarillo': {'bajo': [20, 100, 100],  'alto': [35, 255, 255]},
}

nombres_colores = list(rangos_colores.keys())
color_actual_idx = 0  # indice del color seleccionado

VENTANA_TRACKBARS = "Calibracion HSV"
VENTANA_CAMARA = "Camara"
VENTANA_MASCARA = "Mascara"


def nada(x):
    pass


def crear_trackbars(color_nombre):
    """Crea los trackbars para el color seleccionado."""
    r = rangos_colores[color_nombre]
    cv2.destroyWindow(VENTANA_TRACKBARS)
    cv2.namedWindow(VENTANA_TRACKBARS, cv2.WINDOW_NORMAL)
    cv2.resizeWindow(VENTANA_TRACKBARS, 400, 300)

    cv2.createTrackbar("H min", VENTANA_TRACKBARS, r['bajo'][0], 179, nada)
    cv2.createTrackbar("H max", VENTANA_TRACKBARS, r['alto'][0], 179, nada)
    cv2.createTrackbar("S min", VENTANA_TRACKBARS, r['bajo'][1], 255, nada)
    cv2.createTrackbar("S max", VENTANA_TRACKBARS, r['alto'][1], 255, nada)
    cv2.createTrackbar("V min", VENTANA_TRACKBARS, r['bajo'][2], 255, nada)
    cv2.createTrackbar("V max", VENTANA_TRACKBARS, r['alto'][2], 255, nada)


def leer_trackbars():
    """Lee los valores actuales de los trackbars."""
    h_min = cv2.getTrackbarPos("H min", VENTANA_TRACKBARS)
    h_max = cv2.getTrackbarPos("H max", VENTANA_TRACKBARS)
    s_min = cv2.getTrackbarPos("S min", VENTANA_TRACKBARS)
    s_max = cv2.getTrackbarPos("S max", VENTANA_TRACKBARS)
    v_min = cv2.getTrackbarPos("V min", VENTANA_TRACKBARS)
    v_max = cv2.getTrackbarPos("V max", VENTANA_TRACKBARS)
    return [h_min, s_min, v_min], [h_max, s_max, v_max]


def imprimir_rangos():
    """Imprime todos los rangos HSV en formato listo para copiar al codigo."""
    print("\n" + "=" * 50)
    print("RANGOS HSV CALIBRADOS")
    print("=" * 50)
    for nombre, rango in rangos_colores.items():
        b = rango['bajo']
        a = rango['alto']
        print(f"  {nombre:10s} -> bajo: ({b[0]:3d}, {b[1]:3d}, {b[2]:3d})  "
              f"alto: ({a[0]:3d}, {a[1]:3d}, {a[2]:3d})")
    print("=" * 50)
    # Formato para copiar directamente al codigo de control
    am = rangos_colores['Amarillo']
    print(f"\n# Para ControlLinea (estacionamiento):")
    print(f"AMARILLO_HSV_BAJO = np.array({am['bajo']})")
    print(f"AMARILLO_HSV_ALTO = np.array({am['alto']})")
    print()


def main():
    global color_actual_idx

    # Iniciar Pi Camera
    picam2 = Picamera2()
    config = picam2.create_preview_configuration(main={"size": (640, 480)})
    picam2.configure(config)
    picam2.start()

    cv2.namedWindow(VENTANA_CAMARA, cv2.WINDOW_NORMAL)
    cv2.namedWindow(VENTANA_MASCARA, cv2.WINDOW_NORMAL)

    crear_trackbars(nombres_colores[color_actual_idx])

    print("Calibrador HSV iniciado.")
    print("  Tecla 1: Rojo | Tecla 2: Azul | Tecla 3: Amarillo")
    print("  Tecla P: Imprimir rangos | ESC: Salir")

    while True:
        frame = picam2.capture_array()

        # Picamera2 puede entregar XRGB o RGB, convertir a BGR para OpenCV
        if frame.shape[2] == 4:
            frame = cv2.cvtColor(frame, cv2.COLOR_RGBA2BGR)
        else:
            frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)

        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        # Leer trackbars y actualizar el rango del color seleccionado
        color_nombre = nombres_colores[color_actual_idx]
        bajo, alto = leer_trackbars()
        rangos_colores[color_nombre]['bajo'] = bajo
        rangos_colores[color_nombre]['alto'] = alto

        # Mascara del color activo (para calibrar)
        mask_activa = cv2.inRange(hsv, np.array(bajo), np.array(alto))
        kernel = np.ones((5, 5), np.uint8)
        mask_activa = cv2.morphologyEx(mask_activa, cv2.MORPH_OPEN, kernel)
        mask_activa = cv2.morphologyEx(mask_activa, cv2.MORPH_CLOSE, kernel)

        # Deteccion de contornos del color activo
        contornos, _ = cv2.findContours(mask_activa, cv2.RETR_EXTERNAL,
                                        cv2.CHAIN_APPROX_SIMPLE)
        for contorno in contornos:
            area = cv2.contourArea(contorno)
            if area > 500:
                x, y, w, h = cv2.boundingRect(contorno)
                cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
                cv2.putText(frame, f"{color_nombre} A:{int(area)}",
                            (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX,
                            0.5, (0, 255, 0), 2)

        # Tambien dibujar detecciones de los otros colores (sin trackbars)
        for i, nombre in enumerate(nombres_colores):
            if i == color_actual_idx:
                continue
            r = rangos_colores[nombre]
            mask_otro = cv2.inRange(hsv, np.array(r['bajo']), np.array(r['alto']))
            mask_otro = cv2.morphologyEx(mask_otro, cv2.MORPH_OPEN, kernel)
            cnts, _ = cv2.findContours(mask_otro, cv2.RETR_EXTERNAL,
                                       cv2.CHAIN_APPROX_SIMPLE)
            for c in cnts:
                if cv2.contourArea(c) > 500:
                    x, y, w, h = cv2.boundingRect(c)
                    cv2.rectangle(frame, (x, y), (x + w, y + h), (128, 128, 128), 1)
                    cv2.putText(frame, nombre, (x, y - 10),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.4, (128, 128, 128), 1)

        # HUD
        cv2.putText(frame, f"Color: {color_nombre} [1/2/3]", (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
        cv2.putText(frame, f"H:[{bajo[0]}-{alto[0]}] S:[{bajo[1]}-{alto[1]}] V:[{bajo[2]}-{alto[2]}]",
                    (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (200, 200, 200), 1)

        cv2.imshow(VENTANA_CAMARA, frame)
        cv2.imshow(VENTANA_MASCARA, mask_activa)

        key = cv2.waitKey(1) & 0xFF
        if key == 27:  # ESC
            break
        elif key == ord('1'):
            color_actual_idx = 0
            crear_trackbars(nombres_colores[0])
            print(f"-> Calibrando: {nombres_colores[0]}")
        elif key == ord('2'):
            color_actual_idx = 1
            crear_trackbars(nombres_colores[1])
            print(f"-> Calibrando: {nombres_colores[1]}")
        elif key == ord('3'):
            color_actual_idx = 2
            crear_trackbars(nombres_colores[2])
            print(f"-> Calibrando: {nombres_colores[2]}")
        elif key == ord('p') or key == ord('P'):
            imprimir_rangos()

    # Al salir, imprimir los rangos finales
    imprimir_rangos()

    picam2.stop()
    cv2.destroyAllWindows()
    print("Calibrador finalizado.")


if __name__ == "__main__":
    main()
