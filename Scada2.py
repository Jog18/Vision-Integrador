# -*- coding: utf-8 -*-
"""
Interfaz grafica para monitoreo y control del AGV
Con soporte para estaciones de color (Carga, Descarga, Reposo)
@author: oreaj
"""

import tkinter as tk
from tkinter import messagebox
import paho.mqtt.client as mqtt
from collections import deque
import csv
import os
from datetime import datetime

from reportlab.platypus import SimpleDocTemplate, Paragraph, Spacer, Table, Image
from reportlab.lib.styles import getSampleStyleSheet
from reportlab.lib.pagesizes import letter
import matplotlib.pyplot as plt

# --- Configuracion MQTT ---
BROKER = "192.168.1.68"
PORT = 1883

# --- Datos en tiempo real ---
MAX_PUNTOS = 60
datos_temp = deque(maxlen=MAX_PUNTOS)
porcentaje_bat = 0.0
temperatura_actual = 0.0
estado_agv = "PARO"  # PARO, MOVIMIENTO, EMERGENCIA
zona_agv = "EN_RUTA"  # EN_RUTA, CARGA, DESCARGA, REPOSO

# --- Flags de alertas ---
alerta_temp_mostrada = False
alerta_bat_mostrada = False

# --- Registro de eventos CSV ---
ARCHIVO_EVENTOS = os.path.join(os.path.dirname(os.path.abspath(__file__)), "eventos.csv")
ENCABEZADOS_CSV = ["Fecha", "Hora", "arranque/paro", "Origen", "LM35/uno", "bateria/%"]

origen_ultimo_comando = None


def inicializar_csv():
    if not os.path.exists(ARCHIVO_EVENTOS):
        with open(ARCHIVO_EVENTOS, "w", newline="", encoding="utf-8") as f:
            writer = csv.writer(f)
            writer.writerow(ENCABEZADOS_CSV)


def registrar_evento(evento, origen, temp, bat):
    ahora = datetime.now()
    fila = [
        ahora.strftime("%Y-%m-%d"),
        ahora.strftime("%H:%M:%S"),
        evento,
        origen,
        f"{temp:.1f}",
        f"{bat:.1f}"
    ]
    with open(ARCHIVO_EVENTOS, "a", newline="", encoding="utf-8") as f:
        writer = csv.writer(f)
        writer.writerow(fila)


inicializar_csv()


def generar_grafica_temperatura():
    nombre = "grafica_temp.png"

    if len(datos_temp) == 0:
        return None

    plt.figure()
    plt.plot(list(datos_temp))
    plt.title("Temperatura LM35")
    plt.xlabel("Tiempo")
    plt.ylabel("Temperatura (C)")
    plt.grid()

    plt.savefig(nombre)
    plt.close()

    return nombre

# ===================== MQTT =====================
def on_connect(client, userdata, flags, rc):
    client.subscribe("arranque/paro")
    client.subscribe("LM35/uno")
    client.subscribe("bateria/porcentaje")
    client.subscribe("agv/estacion")


def on_message(client, userdata, msg):
    global porcentaje_bat, temperatura_actual, estado_agv, zona_agv
    global alerta_temp_mostrada, alerta_bat_mostrada
    global origen_ultimo_comando

    topic = msg.topic
    payload = msg.payload.decode()

    if topic == "arranque/paro":
        if payload != estado_agv:
            if origen_ultimo_comando is not None:
                origen = "VIRTUAL"
                origen_ultimo_comando = None
            else:
                origen = "FISICA"

            estado_agv = payload
            registrar_evento(payload, origen, temperatura_actual, porcentaje_bat)

    elif topic == "agv/estacion":
        zona_agv = payload

    elif topic == "LM35/uno":
        try:
            temperatura_actual = float(payload)
            datos_temp.append(temperatura_actual)

            if temperatura_actual > 40 and not alerta_temp_mostrada:
                alerta_temp_mostrada = True
                registrar_evento("ALERTA_TEMP", "SENSOR",
                                 temperatura_actual, porcentaje_bat)
                root.after(0, lambda: mostrar_alerta_temp(temperatura_actual))
            elif temperatura_actual <= 40:
                alerta_temp_mostrada = False
        except ValueError:
            pass

    elif topic == "bateria/porcentaje":
        try:
            porcentaje_bat = float(payload)

            if porcentaje_bat < 20 and not alerta_bat_mostrada:
                alerta_bat_mostrada = True
                registrar_evento("ALERTA_BAT", "SENSOR",
                                 temperatura_actual, porcentaje_bat)
            elif porcentaje_bat >= 20:
                alerta_bat_mostrada = False
        except ValueError:
            pass


def mostrar_alerta_temp(temp):
    messagebox.showwarning("Alerta Temperatura",
                           f"Temperatura critica: {temp:.1f} C\nSupera el limite de 40 C")


# Conectar MQTT
mqtt_client = mqtt.Client()
mqtt_client.on_connect = on_connect
mqtt_client.on_message = on_message
mqtt_client.connect(BROKER, PORT, 60)
mqtt_client.loop_start()


# ===================== COMANDOS =====================
def cmd_arranque():
    global origen_ultimo_comando
    origen_ultimo_comando = "VIRTUAL"
    mqtt_client.publish("esp32/arranque", "GO")


def cmd_paro():
    global origen_ultimo_comando
    origen_ultimo_comando = "VIRTUAL"
    mqtt_client.publish("esp32/arranque", "STOP")


def cmd_emergencia():
    global origen_ultimo_comando
    origen_ultimo_comando = "VIRTUAL"
    mqtt_client.publish("esp32/arranque", "EMERGENCIA")


def cmd_reset_emergencia():
    global origen_ultimo_comando
    origen_ultimo_comando = "VIRTUAL"
    mqtt_client.publish("esp32/arranque", "RESET_EMERGENCIA")

def generar_reporte():
    fecha = datetime.now().strftime("%Y-%m-%d")
    nombre_pdf = f"Reporte_SCADA_{fecha}.pdf"

    estilos = getSampleStyleSheet()
    elementos = []

    elementos.append(Paragraph("REPORTE DIARIO SCADA", estilos['Title']))
    elementos.append(Spacer(1,20))
    elementos.append(Paragraph(f"Fecha: {fecha}", estilos['Normal']))
    elementos.append(Spacer(1,20))

    eventos = []
    alarmas = 0

    with open(ARCHIVO_EVENTOS, newline='', encoding="utf-8") as f:
        reader = csv.reader(f)
        next(reader)
        for row in reader:
            eventos.append(row)
            if "ALERTA" in row[2]:
                alarmas += 1

    elementos.append(Paragraph(f"Numero de alarmas registradas: {alarmas}", estilos['Normal']))
    elementos.append(Spacer(1,20))

    grafica = generar_grafica_temperatura()
    if grafica:
        elementos.append(Paragraph("Grafica de temperatura:", estilos['Heading3']))
        elementos.append(Image(grafica, width=400, height=200))
        elementos.append(Spacer(1,20))

    tabla_data = [ENCABEZADOS_CSV] + eventos
    tabla = Table(tabla_data)

    elementos.append(Paragraph("Eventos registrados:", estilos['Heading3']))
    elementos.append(tabla)

    pdf = SimpleDocTemplate(nombre_pdf, pagesize=letter)
    pdf.build(elementos)

    messagebox.showinfo("Reporte generado", f"Reporte guardado como:\n{nombre_pdf}")

# ===================== INTERFAZ =====================
root = tk.Tk()
root.title("AGV - Panel de Control")
root.configure(bg="#1e1e1e")
root.resizable(False, False)

COLOR_FONDO = "#1e1e1e"
COLOR_PANEL = "#2d2d2d"
COLOR_TEXTO = "#e0e0e0"
COLOR_BORDE = "#404040"

# --- Layout principal ---
frame_izq = tk.Frame(root, bg=COLOR_FONDO)
frame_izq.pack(side=tk.LEFT, fill=tk.BOTH, padx=10, pady=10)

frame_der = tk.Frame(root, bg=COLOR_FONDO)
frame_der.pack(side=tk.RIGHT, fill=tk.BOTH, padx=10, pady=10)

# ==================== COLUMNA IZQUIERDA ====================

# --- Focos de estado ---
frame_focos = tk.LabelFrame(frame_izq, text=" Estado del AGV ",
                            bg=COLOR_PANEL, fg=COLOR_TEXTO,
                            font=("Consolas", 11, "bold"),
                            bd=2, relief=tk.GROOVE)
frame_focos.pack(fill=tk.X, pady=(0, 10))

focos_inner = tk.Frame(frame_focos, bg=COLOR_PANEL)
focos_inner.pack(pady=10)

FOCO_SIZE = 50

# Foco PARO (rojo)
frame_f1 = tk.Frame(focos_inner, bg=COLOR_PANEL)
frame_f1.pack(side=tk.LEFT, padx=15)
canvas_paro = tk.Canvas(frame_f1, width=FOCO_SIZE, height=FOCO_SIZE,
                        bg=COLOR_PANEL, highlightthickness=0)
canvas_paro.pack()
foco_paro = canvas_paro.create_oval(5, 5, FOCO_SIZE-5, FOCO_SIZE-5,
                                    fill="#3d1111", outline="#555")
tk.Label(frame_f1, text="PARO", bg=COLOR_PANEL, fg=COLOR_TEXTO,
         font=("Consolas", 9)).pack()

# Foco MOVIMIENTO (verde)
frame_f2 = tk.Frame(focos_inner, bg=COLOR_PANEL)
frame_f2.pack(side=tk.LEFT, padx=15)
canvas_mov = tk.Canvas(frame_f2, width=FOCO_SIZE, height=FOCO_SIZE,
                       bg=COLOR_PANEL, highlightthickness=0)
canvas_mov.pack()
foco_mov = canvas_mov.create_oval(5, 5, FOCO_SIZE-5, FOCO_SIZE-5,
                                  fill="#113d11", outline="#555")
tk.Label(frame_f2, text="MOVIMIENTO", bg=COLOR_PANEL, fg=COLOR_TEXTO,
         font=("Consolas", 9)).pack()

# Foco EMERGENCIA (amarillo/naranja)
frame_f3 = tk.Frame(focos_inner, bg=COLOR_PANEL)
frame_f3.pack(side=tk.LEFT, padx=15)
canvas_emg = tk.Canvas(frame_f3, width=FOCO_SIZE, height=FOCO_SIZE,
                       bg=COLOR_PANEL, highlightthickness=0)
canvas_emg.pack()
foco_emg = canvas_emg.create_oval(5, 5, FOCO_SIZE-5, FOCO_SIZE-5,
                                  fill="#3d3411", outline="#555")
tk.Label(frame_f3, text="EMERGENCIA", bg=COLOR_PANEL, fg=COLOR_TEXTO,
         font=("Consolas", 9)).pack()

# Label del estado actual
lbl_estado = tk.Label(frame_focos, text="Estado: PARO", bg=COLOR_PANEL,
                      fg="#ff5555", font=("Consolas", 13, "bold"))
lbl_estado.pack(pady=(0, 10))

# --- Indicadores de Zona / Estacion ---
frame_zonas = tk.LabelFrame(frame_izq, text=" Zona del AGV ",
                            bg=COLOR_PANEL, fg=COLOR_TEXTO,
                            font=("Consolas", 11, "bold"),
                            bd=2, relief=tk.GROOVE)
frame_zonas.pack(fill=tk.X, pady=(0, 10))

zonas_inner = tk.Frame(frame_zonas, bg=COLOR_PANEL)
zonas_inner.pack(pady=10)

FOCO_ZONA_SIZE = 45

# Foco CARGA (azul)
frame_z1 = tk.Frame(zonas_inner, bg=COLOR_PANEL)
frame_z1.pack(side=tk.LEFT, padx=12)
canvas_carga = tk.Canvas(frame_z1, width=FOCO_ZONA_SIZE, height=FOCO_ZONA_SIZE,
                         bg=COLOR_PANEL, highlightthickness=0)
canvas_carga.pack()
foco_carga = canvas_carga.create_oval(4, 4, FOCO_ZONA_SIZE-4, FOCO_ZONA_SIZE-4,
                                      fill="#111133", outline="#555")
tk.Label(frame_z1, text="CARGA", bg=COLOR_PANEL, fg="#6688ff",
         font=("Consolas", 9, "bold")).pack()

# Foco DESCARGA (rojo)
frame_z2 = tk.Frame(zonas_inner, bg=COLOR_PANEL)
frame_z2.pack(side=tk.LEFT, padx=12)
canvas_descarga = tk.Canvas(frame_z2, width=FOCO_ZONA_SIZE, height=FOCO_ZONA_SIZE,
                            bg=COLOR_PANEL, highlightthickness=0)
canvas_descarga.pack()
foco_descarga = canvas_descarga.create_oval(4, 4, FOCO_ZONA_SIZE-4, FOCO_ZONA_SIZE-4,
                                            fill="#331111", outline="#555")
tk.Label(frame_z2, text="DESCARGA", bg=COLOR_PANEL, fg="#ff6666",
         font=("Consolas", 9, "bold")).pack()

# Foco REPOSO (amarillo)
frame_z3 = tk.Frame(zonas_inner, bg=COLOR_PANEL)
frame_z3.pack(side=tk.LEFT, padx=12)
canvas_reposo = tk.Canvas(frame_z3, width=FOCO_ZONA_SIZE, height=FOCO_ZONA_SIZE,
                          bg=COLOR_PANEL, highlightthickness=0)
canvas_reposo.pack()
foco_reposo = canvas_reposo.create_oval(4, 4, FOCO_ZONA_SIZE-4, FOCO_ZONA_SIZE-4,
                                        fill="#332211", outline="#555")
tk.Label(frame_z3, text="REPOSO", bg=COLOR_PANEL, fg="#ffcc44",
         font=("Consolas", 9, "bold")).pack()

# Foco EN RUTA (verde)
frame_z4 = tk.Frame(zonas_inner, bg=COLOR_PANEL)
frame_z4.pack(side=tk.LEFT, padx=12)
canvas_enruta = tk.Canvas(frame_z4, width=FOCO_ZONA_SIZE, height=FOCO_ZONA_SIZE,
                          bg=COLOR_PANEL, highlightthickness=0)
canvas_enruta.pack()
foco_enruta = canvas_enruta.create_oval(4, 4, FOCO_ZONA_SIZE-4, FOCO_ZONA_SIZE-4,
                                        fill="#113311", outline="#555")
tk.Label(frame_z4, text="EN RUTA", bg=COLOR_PANEL, fg="#66ff66",
         font=("Consolas", 9, "bold")).pack()

# Label de zona actual
lbl_zona = tk.Label(frame_zonas, text="Zona: EN RUTA", bg=COLOR_PANEL,
                    fg="#66ff66", font=("Consolas", 12, "bold"))
lbl_zona.pack(pady=(0, 10))

# --- Botones de control ---
frame_botones = tk.LabelFrame(frame_izq, text=" Control ",
                              bg=COLOR_PANEL, fg=COLOR_TEXTO,
                              font=("Consolas", 11, "bold"),
                              bd=2, relief=tk.GROOVE)
frame_botones.pack(fill=tk.X, pady=(0, 10))

btn_frame = tk.Frame(frame_botones, bg=COLOR_PANEL)
btn_frame.pack(pady=10)

btn_arranque = tk.Button(btn_frame, text="ARRANQUE", command=cmd_arranque,
                         bg="#1a5c1a", fg="white", activebackground="#2d8c2d",
                         font=("Consolas", 11, "bold"), width=12, height=2,
                         relief=tk.RAISED, bd=3, cursor="hand2")
btn_arranque.pack(pady=5)

btn_paro = tk.Button(btn_frame, text="PARO", command=cmd_paro,
                     bg="#8c1a1a", fg="white", activebackground="#b32d2d",
                     font=("Consolas", 11, "bold"), width=12, height=2,
                     relief=tk.RAISED, bd=3, cursor="hand2")
btn_paro.pack(pady=5)

btn_emergencia = tk.Button(btn_frame, text="E-STOP", command=cmd_emergencia,
                           bg="#8c6b00", fg="white", activebackground="#b38a00",
                           font=("Consolas", 11, "bold"), width=12, height=2,
                           relief=tk.RAISED, bd=3, cursor="hand2")
btn_emergencia.pack(pady=5)

btn_reset = tk.Button(btn_frame, text="RESET E-STOP", command=cmd_reset_emergencia,
                      bg="#444444", fg="white", activebackground="#666666",
                      font=("Consolas", 9), width=12, height=1,
                      relief=tk.RAISED, bd=2, cursor="hand2")
btn_reset.pack(pady=(5, 0))

btn_reporte = tk.Button(btn_frame, text="GENERAR REPORTE",
                        command=generar_reporte,
                        bg="#004488", fg="white",
                        font=("Consolas", 10, "bold"),
                        width=12, height=2)
btn_reporte.pack(pady=5)

# --- Barra bateria ---
frame_bat = tk.LabelFrame(frame_izq, text=" Bateria ",
                          bg=COLOR_PANEL, fg=COLOR_TEXTO,
                          font=("Consolas", 11, "bold"),
                          bd=2, relief=tk.GROOVE)
frame_bat.pack(fill=tk.X, pady=(0, 10))

lbl_bat_val = tk.Label(frame_bat, text="0.0 %", bg=COLOR_PANEL,
                       fg="#00ccff", font=("Consolas", 16, "bold"))
lbl_bat_val.pack(pady=(10, 0))

canvas_bat = tk.Canvas(frame_bat, width=280, height=35,
                       bg="#111111", highlightthickness=1,
                       highlightbackground=COLOR_BORDE)
canvas_bat.pack(pady=10, padx=10)

canvas_bat.create_rectangle(2, 2, 278, 33, outline="#555", width=1)
barra_bat = canvas_bat.create_rectangle(3, 3, 3, 32, fill="#00ccff", outline="")

marca_20 = int(3 + (20 / 100) * 275)
canvas_bat.create_line(marca_20, 2, marca_20, 33, fill="#555", dash=(2, 2))
canvas_bat.create_text(marca_20, 33, text="20%", fill="#777", font=("Consolas", 7), anchor=tk.N)


# ==================== COLUMNA DERECHA ====================

# --- Grafica temperatura ---
frame_grafica = tk.LabelFrame(frame_der, text=" Temperatura LM35 (tiempo real) ",
                              bg=COLOR_PANEL, fg=COLOR_TEXTO,
                              font=("Consolas", 11, "bold"),
                              bd=2, relief=tk.GROOVE)
frame_grafica.pack(fill=tk.BOTH, expand=True)

lbl_temp_val = tk.Label(frame_grafica, text="0.0 C", bg=COLOR_PANEL,
                        fg="#ff6644", font=("Consolas", 18, "bold"))
lbl_temp_val.pack(pady=(5, 0))

GRAF_W = 420
GRAF_H = 280
GRAF_MARGEN_IZQ = 40
GRAF_MARGEN_DER = 10
GRAF_MARGEN_TOP = 10
GRAF_MARGEN_BOT = 25
GRAF_TEMP_MIN = 0
GRAF_TEMP_MAX = 80

canvas_graf = tk.Canvas(frame_grafica, width=GRAF_W, height=GRAF_H,
                        bg="#0a0a0a", highlightthickness=1,
                        highlightbackground=COLOR_BORDE)
canvas_graf.pack(padx=10, pady=10)


# ===================== ACTUALIZACION =====================
def actualizar_interfaz():
    global estado_agv, zona_agv

    # --- Focos de estado ---
    color_paro_on = "#ff3333"
    color_paro_off = "#3d1111"
    color_mov_on = "#33ff33"
    color_mov_off = "#113d11"
    color_emg_on = "#ffcc00"
    color_emg_off = "#3d3411"

    if estado_agv == "PARO":
        canvas_paro.itemconfig(foco_paro, fill=color_paro_on)
        canvas_mov.itemconfig(foco_mov, fill=color_mov_off)
        canvas_emg.itemconfig(foco_emg, fill=color_emg_off)
        lbl_estado.config(text="Estado: PARO", fg="#ff5555")
    elif estado_agv == "MOVIMIENTO":
        canvas_paro.itemconfig(foco_paro, fill=color_paro_off)
        canvas_mov.itemconfig(foco_mov, fill=color_mov_on)
        canvas_emg.itemconfig(foco_emg, fill=color_emg_off)
        lbl_estado.config(text="Estado: MOVIMIENTO", fg="#55ff55")
    elif estado_agv == "EMERGENCIA":
        canvas_paro.itemconfig(foco_paro, fill=color_paro_off)
        canvas_mov.itemconfig(foco_mov, fill=color_mov_off)
        canvas_emg.itemconfig(foco_emg, fill=color_emg_on)
        lbl_estado.config(text="Estado: EMERGENCIA", fg="#ffcc00")

    # --- Focos de zona ---
    color_carga_on = "#4488ff"
    color_carga_off = "#111133"
    color_descarga_on = "#ff4444"
    color_descarga_off = "#331111"
    color_reposo_on = "#ffcc00"
    color_reposo_off = "#332211"
    color_enruta_on = "#44ff44"
    color_enruta_off = "#113311"

    canvas_carga.itemconfig(foco_carga, fill=color_carga_off)
    canvas_descarga.itemconfig(foco_descarga, fill=color_descarga_off)
    canvas_reposo.itemconfig(foco_reposo, fill=color_reposo_off)
    canvas_enruta.itemconfig(foco_enruta, fill=color_enruta_off)

    if zona_agv == "CARGA":
        canvas_carga.itemconfig(foco_carga, fill=color_carga_on)
        lbl_zona.config(text="Zona: CARGA (Azul)", fg="#4488ff")
    elif zona_agv == "DESCARGA":
        canvas_descarga.itemconfig(foco_descarga, fill=color_descarga_on)
        lbl_zona.config(text="Zona: DESCARGA (Rojo)", fg="#ff4444")
    elif zona_agv == "REPOSO":
        canvas_reposo.itemconfig(foco_reposo, fill=color_reposo_on)
        lbl_zona.config(text="Zona: REPOSO (Amarillo)", fg="#ffcc00")
    else:
        canvas_enruta.itemconfig(foco_enruta, fill=color_enruta_on)
        lbl_zona.config(text="Zona: EN RUTA", fg="#44ff44")

    # --- Barra bateria ---
    lbl_bat_val.config(text=f"{porcentaje_bat:.1f} %")
    ancho_barra = int((porcentaje_bat / 100.0) * 275)
    ancho_barra = max(0, min(275, ancho_barra))

    if porcentaje_bat < 20:
        color_barra = "#ff3333"
    elif porcentaje_bat < 50:
        color_barra = "#ffcc00"
    else:
        color_barra = "#00ccff"
    canvas_bat.coords(barra_bat, 3, 3, 3 + ancho_barra, 32)
    canvas_bat.itemconfig(barra_bat, fill=color_barra)
    lbl_bat_val.config(fg=color_barra)

    # --- Valor temperatura ---
    if temperatura_actual > 40:
        color_temp = "#ff3333"
    else:
        color_temp = "#ff6644"
    lbl_temp_val.config(text=f"{temperatura_actual:.1f} C", fg=color_temp)

    # --- Grafica temperatura ---
    canvas_graf.delete("all")

    area_w = GRAF_W - GRAF_MARGEN_IZQ - GRAF_MARGEN_DER
    area_h = GRAF_H - GRAF_MARGEN_TOP - GRAF_MARGEN_BOT

    for i in range(0, GRAF_TEMP_MAX + 1, 10):
        y = GRAF_MARGEN_TOP + area_h - (i / GRAF_TEMP_MAX * area_h)
        color_linea = "#333333"
        if i == 40:
            color_linea = "#553333"
        canvas_graf.create_line(GRAF_MARGEN_IZQ, y,
                                GRAF_W - GRAF_MARGEN_DER, y,
                                fill=color_linea, dash=(2, 4))
        canvas_graf.create_text(GRAF_MARGEN_IZQ - 5, y, text=str(i),
                                fill="#777", font=("Consolas", 8), anchor=tk.E)

    y_alerta = GRAF_MARGEN_TOP + area_h - (40 / GRAF_TEMP_MAX * area_h)
    canvas_graf.create_line(GRAF_MARGEN_IZQ, y_alerta,
                            GRAF_W - GRAF_MARGEN_DER, y_alerta,
                            fill="#ff3333", dash=(4, 2), width=1)
    canvas_graf.create_text(GRAF_W - GRAF_MARGEN_DER, y_alerta - 8,
                            text="40 C", fill="#ff3333",
                            font=("Consolas", 7), anchor=tk.E)

    canvas_graf.create_rectangle(GRAF_MARGEN_IZQ, GRAF_MARGEN_TOP,
                                 GRAF_W - GRAF_MARGEN_DER,
                                 GRAF_H - GRAF_MARGEN_BOT,
                                 outline="#444")

    if len(datos_temp) > 1:
        puntos = list(datos_temp)
        n = len(puntos)
        step_x = area_w / (MAX_PUNTOS - 1)

        coords = []
        for i, val in enumerate(puntos):
            x = GRAF_MARGEN_IZQ + (MAX_PUNTOS - n + i) * step_x
            val_clamped = max(GRAF_TEMP_MIN, min(GRAF_TEMP_MAX, val))
            y = GRAF_MARGEN_TOP + area_h - (val_clamped / GRAF_TEMP_MAX * area_h)
            coords.extend([x, y])

        canvas_graf.create_line(coords, fill="#ff6644", width=2, smooth=True)

        if len(coords) >= 2:
            canvas_graf.create_oval(coords[-2] - 4, coords[-1] - 4,
                                    coords[-2] + 4, coords[-1] + 4,
                                    fill="#ff6644", outline="#ffaa88")

    root.after(500, actualizar_interfaz)


actualizar_interfaz()


# --- Cerrar limpiamente ---
def on_closing():
    mqtt_client.loop_stop()
    mqtt_client.disconnect()
    root.destroy()


root.protocol("WM_DELETE_WINDOW", on_closing)
root.mainloop()
