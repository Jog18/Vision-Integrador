# -*- coding: utf-8 -*-
"""
Created on Mon Mar 16 11:52:27 2026

@author: USUARIO
"""

from reportlab.platypus import SimpleDocTemplate, Paragraph, Spacer, Table, Image
from reportlab.lib.styles import getSampleStyleSheet
from reportlab.lib.pagesizes import A4
from reportlab.lib.units import cm
import matplotlib.pyplot as plt
from datetime import datetime

# ======================
# DATOS
# ======================
fecha = datetime.now().strftime("%d/%m/%Y")
alarmas = 3
tiempo_total = "6 h 40 min"

eventos = [
    ["Hora", "Evento"],
    ["07:00", "Encendido del sistema"],
    ["08:15", "Alarma temperatura alta"],
    ["08:17", "Paro automático"],
    ["08:30", "Reinicio"],
    ["13:05", "Paro de emergencia"],
    ["13:20", "Sistema restablecido"],
    ["15:45", "Apagado del sistema"]
]

# ======================
# GRÁFICA
# ======================
tiempo = [0,1,2,3,4,5,6]
temperatura = [55,60,72,88,95,90,70]

plt.plot(tiempo, temperatura)
plt.title("Temperatura del Sistema")
plt.xlabel("Tiempo (h)")
plt.ylabel("Temperatura (°C)")
plt.grid()
grafica = "temp.png"
plt.savefig(grafica)
plt.close()

# ======================
# PDF
# ======================
archivo_pdf = "Reporte_Diario.pdf"
estilos = getSampleStyleSheet()
doc = SimpleDocTemplate(archivo_pdf, pagesize=A4)
contenido = []

contenido.append(Paragraph("REPORTE DIARIO DEL SISTEMA", estilos['Title']))
contenido.append(Spacer(1,12))

contenido.append(Paragraph(f"Fecha de operación: {fecha}", estilos['Normal']))
contenido.append(Paragraph(f"Número de alarmas: {alarmas}", estilos['Normal']))
contenido.append(Paragraph(f"Tiempo total de operación: {tiempo_total}", estilos['Normal']))
contenido.append(Spacer(1,12))

contenido.append(Paragraph("Gráfica de temperatura", estilos['Heading2']))
contenido.append(Spacer(1,12))
contenido.append(Image(grafica, width=14*cm, height=8*cm))
contenido.append(Spacer(1,12))

contenido.append(Paragraph("Eventos registrados", estilos['Heading2']))
contenido.append(Spacer(1,12))

tabla = Table(eventos)
contenido.append(tabla)

doc.build(contenido)

print("Reporte generado correctamente")