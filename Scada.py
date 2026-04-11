# -*- coding: utf-8 -*-
"""
Interfaz grafica para monitoreo y control del AGV
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
# %%
BROKER = "10.108.51.191"
# %%
PORT = 1883

# --- Datos en tiempo real ---
MAX_PUNTOS = 60  # 60 segundos de historial en la grafica
datos_temp = deque(maxlen=MAX_PUNTOS)
voltaje_pot = 0.0
temperatura_actual = 0.0
estado_agv = "PARO"  # PARO, MOVIMIENTO, EMERGENCIA

# --- Flags de alertas (para no repetir messagebox) ---
alerta_temp_mostrada = False
alerta_pot_mostrada = False

# --- Registro de eventos CSV ---
ARCHIVO_EVENTOS = os.path.join(os.path.dirname(os.path.abspath(__file__)), "eventos.csv")
ENCABEZADOS_CSV = ["Fecha", "Hora", "arranque/paro", "Origen", "LM35/uno", "pot/uno"]

# Flag para saber si el ultimo comando fue enviado desde la interfaz
origen_ultimo_comando = None  # "VIRTUAL" o None


def inicializar_csv():
    if not os.path.exists(ARCHIVO_EVENTOS):
        with open(ARCHIVO_EVENTOS, "w", newline="", encoding="utf-8") as f:
            writer = csv.writer(f)
            writer.writerow(ENCABEZADOS_CSV)


def registrar_evento(evento, origen, temp, volt):
    ahora = datetime.now()
    fila = [
        ahora.strftime("%Y-%m-%d"),
        ahora.strftime("%H:%M:%S"),
        evento,
        origen,
        f"{temp:.1f}",
        f"{volt:.3f}"
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
    client.subscribe("pot/uno")


def on_message(client, userdata, msg):
    global voltaje_pot, temperatura_actual, estado_agv
    global alerta_temp_mostrada, alerta_pot_mostrada
    global origen_ultimo_comando

    topic = msg.topic
    payload = msg.payload.decode()

    if topic == "arranque/paro":
        if payload != estado_agv:
            # Determinar origen del evento
            if origen_ultimo_comando is not None:
                origen = "VIRTUAL"
                origen_ultimo_comando = None
            else:
                origen = "FISICA"

            estado_agv = payload
            registrar_evento(payload, origen, temperatura_actual, voltaje_pot)

    elif topic == "LM35/uno":
        try:
            temperatura_actual = float(payload)
            datos_temp.append(temperatura_actual)

            # Alerta temperatura > 40
            if temperatura_actual > 40 and not alerta_temp_mostrada:
                alerta_temp_mostrada = True
                registrar_evento("ALERTA_TEMP", "SENSOR",
                                 temperatura_actual, voltaje_pot)
                root.after(0, lambda: mostrar_alerta_temp(temperatura_actual))
            elif temperatura_actual <= 40:
                alerta_temp_mostrada = False
        except ValueError:
            pass

    elif topic == "pot/uno":
        try:
            voltaje_pot = float(payload)

            # Alerta pot < 0.5V
            if voltaje_pot < 0.5 and not alerta_pot_mostrada:
                alerta_pot_mostrada = True
                registrar_evento("ALERTA_POT", "SENSOR",
                                 temperatura_actual, voltaje_pot)
                root.after(0, lambda: mostrar_alerta_pot(voltaje_pot))
            elif voltaje_pot >= 0.5:
                alerta_pot_mostrada = False
        except ValueError:
            pass


def mostrar_alerta_temp(temp):
    messagebox.showwarning("Alerta Temperatura",
                           f"Temperatura critica: {temp:.1f} C\nSupera el limite de 40 C")


def mostrar_alerta_pot(volt):
    messagebox.showwarning("Alerta Potenciometro",
                           f"Voltaje bajo: {volt:.3f} V\nPor debajo del limite de 0.5 V")


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

    # Leer eventos
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

    # Grafica
    grafica = generar_grafica_temperatura()
    if grafica:
        elementos.append(Paragraph("Grafica de temperatura:", estilos['Heading3']))
        elementos.append(Image(grafica, width=400, height=200))
        elementos.append(Spacer(1,20))

    # Tabla de eventos
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

# Canvas para cada foco
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

# --- Barra potenciometro ---
frame_pot = tk.LabelFrame(frame_izq, text=" Potenciometro ",
                          bg=COLOR_PANEL, fg=COLOR_TEXTO,
                          font=("Consolas", 11, "bold"),
                          bd=2, relief=tk.GROOVE)
frame_pot.pack(fill=tk.X, pady=(0, 10))

lbl_pot_val = tk.Label(frame_pot, text="0.000 V", bg=COLOR_PANEL,
                       fg="#00ccff", font=("Consolas", 16, "bold"))
lbl_pot_val.pack(pady=(10, 0))

canvas_pot = tk.Canvas(frame_pot, width=280, height=35,
                       bg="#111111", highlightthickness=1,
                       highlightbackground=COLOR_BORDE)
canvas_pot.pack(pady=10, padx=10)

# Borde de la barra
canvas_pot.create_rectangle(2, 2, 278, 33, outline="#555", width=1)

# Barra de nivel (se actualiza)
barra_pot = canvas_pot.create_rectangle(3, 3, 3, 32, fill="#00ccff", outline="")

# Marcas de referencia
canvas_pot.create_line(42, 2, 42, 33, fill="#555", dash=(2, 2))  # 0.5V
canvas_pot.create_text(42, 33, text="0.5", fill="#777", font=("Consolas", 7), anchor=tk.N)


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
    global estado_agv

    # --- Focos ---
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

    # --- Barra potenciometro ---
    lbl_pot_val.config(text=f"{voltaje_pot:.3f} V")
    ancho_barra = int((voltaje_pot / 3.3) * 275)
    ancho_barra = max(0, min(275, ancho_barra))

    if voltaje_pot < 0.5:
        color_barra = "#ff3333"
    else:
        color_barra = "#00ccff"
    canvas_pot.coords(barra_pot, 3, 3, 3 + ancho_barra, 32)
    canvas_pot.itemconfig(barra_pot, fill=color_barra)
    lbl_pot_val.config(fg=color_barra)

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

    # Cuadricula horizontal y etiquetas Y
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

    # Linea de alerta 40 C
    y_alerta = GRAF_MARGEN_TOP + area_h - (40 / GRAF_TEMP_MAX * area_h)
    canvas_graf.create_line(GRAF_MARGEN_IZQ, y_alerta,
                            GRAF_W - GRAF_MARGEN_DER, y_alerta,
                            fill="#ff3333", dash=(4, 2), width=1)
    canvas_graf.create_text(GRAF_W - GRAF_MARGEN_DER, y_alerta - 8,
                            text="40 C", fill="#ff3333",
                            font=("Consolas", 7), anchor=tk.E)

    # Borde del area de grafica
    canvas_graf.create_rectangle(GRAF_MARGEN_IZQ, GRAF_MARGEN_TOP,
                                 GRAF_W - GRAF_MARGEN_DER,
                                 GRAF_H - GRAF_MARGEN_BOT,
                                 outline="#444")

    # Dibujar datos
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

        # Punto del ultimo valor
        if len(coords) >= 2:
            canvas_graf.create_oval(coords[-2] - 4, coords[-1] - 4,
                                    coords[-2] + 4, coords[-1] + 4,
                                    fill="#ff6644", outline="#ffaa88")

    # Repetir cada 500ms
    root.after(500, actualizar_interfaz)


# Iniciar bucle de actualizacion
actualizar_interfaz()


# --- Cerrar limpiamente ---
def on_closing():
    mqtt_client.loop_stop()
    mqtt_client.disconnect()
    root.destroy()


root.protocol("WM_DELETE_WINDOW", on_closing)
root.mainloop()