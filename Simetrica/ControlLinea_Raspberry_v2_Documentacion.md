# Documentacion — ControlLinea_Raspberry_v2.py

## Seguidor de Linea con Control Difuso Mamdani (Universo Simetrico 80-146)

---

## 1. Descripcion General

`ControlLinea_Raspberry_v2.py` es un controlador de direccion para un AGV Ackermann a escala 1:10 que sigue una linea blanca sobre fondo negro. Utiliza una camara CSI (Picamera2) en una Raspberry Pi para capturar video, detectar la linea mediante procesamiento de imagen, y calcular el angulo de direccion optimo a traves de un sistema de control difuso tipo Mamdani.

### Cambio principal respecto a la version anterior

El universo de salida se amplio de **[80, 135]** (asimetrico: 33 izquierda, 22 derecha) a **[80, 146]** (simetrico: 33 izquierda, 33 derecha), manteniendo el centro en 113 grados. Esto proporciona el mismo rango de giro a ambos lados, resultando en un comportamiento de direccion simetrico y funciones de membresia de salida uniformemente distribuidas.

| Parametro | v1 (anterior) | v2 (actual) |
|---|---|---|
| Servo minimo | 80 | 80 |
| Servo recto | 113 | 113 |
| Servo maximo | 135 | **146** |
| Rango izquierda | 33 | 33 |
| Rango derecha | 22 | **33** |
| Paso entre MFs salida | variable | **11 (uniforme)** |

---

## 2. Dependencias

| Libreria | Uso |
|---|---|
| `opencv-python` (cv2) | Captura de video, procesamiento de imagen, visualizacion HUD |
| `numpy` | Operaciones numericas para funciones de membresia y defuzzificacion |
| `paho-mqtt` | Cliente MQTT para comunicacion con el ESP32 |
| `picamera2` | Interfaz con la camara CSI de la Raspberry Pi |

---

## 3. Deteccion de Linea

### 3.1 Pipeline de procesamiento

```
Frame (640x480) → Escala de grises → Recorte ROI (40% inferior)
    → Binarizacion (umbral 200) → Apertura morfologica → Cierre morfologico
    → Busqueda de contornos → Contorno mas grande → Centroide (momentos)
    → Posicion normalizada [0, 1]
```

### 3.2 Parametros ajustables

| Parametro | Valor | Descripcion |
|---|---|---|
| `UMBRAL_BLANCO` | 200 | Pixeles con valor > 200 se consideran linea blanca |
| `MIN_AREA` | 300 | Area minima de contorno en pixeles para filtrar ruido |
| `ROI_PROPORCION` | 0.4 | Fraccion inferior del frame usada como region de interes |

### 3.3 Normalizacion de posicion

La posicion horizontal del centroide de la linea se normaliza al rango [0, 1]:

```
posicion_norm = cx / ancho_frame
```

- `0.0` = linea en el borde izquierdo del frame
- `0.5` = linea centrada
- `1.0` = linea en el borde derecho del frame

---

## 4. Sistema de Control Difuso

### 4.1 Variable de entrada: posicion normalizada [0, 1]

Se definen **7 conjuntos difusos** de entrada, distribuidos simetricamente:

| Conjunto | Tipo | Parametros | Zona dominante |
|---|---|---|---|
| Muy Izquierda | Trapezoidal | [0.00, 0.00, 0.05, 0.20] | 0.00 – 0.20 |
| Med Izquierda | Triangular | [0.05, 0.20, 0.35] | 0.05 – 0.35 |
| Poco Izquierda | Triangular | [0.20, 0.35, 0.50] | 0.20 – 0.50 |
| Centro | Triangular | [0.35, 0.50, 0.65] | 0.35 – 0.65 |
| Poco Derecha | Triangular | [0.50, 0.65, 0.80] | 0.50 – 0.80 |
| Med Derecha | Triangular | [0.65, 0.80, 0.95] | 0.65 – 0.95 |
| Muy Derecha | Trapezoidal | [0.80, 0.95, 1.00, 1.00] | 0.80 – 1.00 |

Los conjuntos de entrada **no cambiaron** respecto a la version anterior.

### 4.2 Variable de salida: angulo del servo [80, 146] grados

Se definen **7 conjuntos difusos** de salida, ahora **uniformemente distribuidos** con un paso de 11 grados:

```
Centros de las MFs:  80 — 91 — 102 — 113 — 124 — 135 — 146
                                       ↑
                                    recto
```

| Conjunto | Tipo | Parametros [a, b, c] | Centro | Direccion |
|---|---|---|---|---|
| Muy Izquierda | Triangular | [80, 80, 91] | 80 | Giro fuerte izquierda |
| Med Izquierda | Triangular | [80, 91, 102] | 91 | Giro medio izquierda |
| Poco Izquierda | Triangular | [91, 102, 113] | 102 | Giro leve izquierda |
| Centro | Triangular | [102, 113, 124] | 113 | Recto |
| Poco Derecha | Triangular | [113, 124, 135] | 124 | Giro leve derecha |
| Med Derecha | Triangular | [124, 135, 146] | 135 | Giro medio derecha |
| Muy Derecha | Triangular | [135, 146, 146] | 146 | Giro fuerte derecha |

#### Comparacion visual con version anterior

```
v1 (asimetrico):  80---91---102---113--120--127--135
                  |---- 33 grados ----|--- 22 ---|

v2 (simetrico):   80---91---102---113---124---135---146
                  |---- 33 grados ----|---- 33 grados ----|
```

### 4.3 Base de reglas (15 reglas)

Las reglas son identicas a la version anterior. La simetria del universo de salida hace que el comportamiento resultante sea naturalmente simetrico.

#### Reglas directas (R1–R7)

Cada conjunto de entrada se mapea directamente a su conjunto de salida correspondiente:

| Regla | SI entrada ES... | ENTONCES salida ES... |
|---|---|---|
| R1 | Muy Izquierda | Muy Izquierda (80) |
| R2 | Med Izquierda | Med Izquierda (91) |
| R3 | Poco Izquierda | Poco Izquierda (102) |
| R4 | Centro | Centro (113) |
| R5 | Poco Derecha | Poco Derecha (124) |
| R6 | Med Derecha | Med Derecha (135) |
| R7 | Muy Derecha | Muy Derecha (146) |

#### Reglas de refuerzo en extremos (R8–R11)

Refuerzan la respuesta cuando la linea esta en posiciones extremas:

| Regla | Condicion | Salida | Activacion |
|---|---|---|---|
| R8 | Muy Izq AND NOT Med Izq | Muy Izquierda | `min(mu_muy_izq, 1 - mu_med_izq)` |
| R9 | Muy Der AND NOT Med Der | Muy Derecha | `min(mu_muy_der, 1 - mu_med_der)` |
| R10 | Med Izq AND NOT Poco Izq | Med Izquierda | `min(mu_med_izq, 1 - mu_poco_izq)` |
| R11 | Med Der AND NOT Poco Der | Med Derecha | `min(mu_med_der, 1 - mu_poco_der)` |

#### Reglas de transicion suave (R12–R14)

Suavizan el comportamiento en zonas de transicion cerca del centro:

| Regla | Condicion | Salida | Activacion |
|---|---|---|---|
| R12 | Poco Izq AND Centro | Centro | `min(mu_poco_izq, mu_cen)` |
| R13 | Poco Der AND Centro | Centro | `min(mu_poco_der, mu_cen)` |
| R14 | Centro AND NOT Poco Izq AND NOT Poco Der | Centro | `min(mu_cen, 1-mu_poco_izq, 1-mu_poco_der)` |

#### Regla de seguridad (R15)

Si ninguna funcion de membresia tiene activacion significativa (linea no detectada claramente), el servo se mantiene recto:

```
R15: SI NOT(todo) → Centro
Activacion: min(1-mu_muy_izq, 1-mu_med_izq, ..., 1-mu_muy_der)
```

### 4.4 Agregacion de activaciones por conjunto de salida

Cada conjunto de salida toma el maximo de todas las reglas que lo activan:

| Salida | Activacion final |
|---|---|
| Muy Izquierda | `max(R1, R8)` |
| Med Izquierda | `max(R2, R10)` |
| Poco Izquierda | `R3` |
| Centro | `max(R4, R12, R13, R14, R15)` |
| Poco Derecha | `R5` |
| Med Derecha | `max(R6, R11)` |
| Muy Derecha | `max(R7, R9)` |

### 4.5 Inferencia Mamdani

1. **Recorte (clipping):** cada funcion de membresia de salida se recorta al nivel de activacion de su regla
2. **Agregacion:** se toma el maximo punto a punto de todas las MFs recortadas

```python
for cada regla:
    clipped = min(mf_salida, nivel_activacion)
    aggregated = max(aggregated, clipped)
```

### 4.6 Defuzzificacion: Centro de Gravedad (Centroide)

```
              Σ (angulo_i × μ(angulo_i))
centroide = ─────────────────────────────
                  Σ μ(angulo_i)
```

- Se discretiza el universo de salida en 67 puntos (80 a 146, paso de 1 grado)
- Si el area total es cero (sin activacion), retorna 113 (recto) como valor seguro

---

## 5. Comunicacion MQTT

| Parametro | Valor |
|---|---|
| Broker | `127.0.0.1` (local en la Raspberry Pi) |
| Puerto | `1883` |
| Topic | `esp32/servo/control` |
| Payload | Angulo entero como string (ej: `"113"`) |
| Umbral de envio | Solo si el cambio es > 2 grados respecto al ultimo envio |

---

## 6. HUD (Visualizacion en Pantalla)

### 6.1 Elementos del frame principal

| Elemento | Descripcion |
|---|---|
| Zona verde semitransparente | Tercio central del frame (zona ideal para la linea) |
| Lineas verdes verticales | Limites del tercio central |
| Linea gris central | Centro exacto del frame |
| Linea cyan horizontal | Limite superior del ROI, con etiqueta "ROI" |
| Contorno azul | Contorno de la linea detectada dentro del ROI |
| Circulo rojo | Centroide de la linea detectada |
| Texto amarillo | Posicion normalizada junto al centroide |
| Texto grande (arriba izq) | Angulo actual del servo y etiqueta de direccion |
| Texto "SIN LINEA" | Se muestra en rojo cuando no se detecta linea |

### 6.2 Etiquetas de direccion

| Condicion | Etiqueta | Color |
|---|---|---|
| Angulo < 110 (recto - 3) | IZQUIERDA | Naranja |
| Angulo > 116 (recto + 3) | DERECHA | Azul-naranja |
| 110 ≤ Angulo ≤ 116 | CENTRO | Verde |

### 6.3 Barra indicadora inferior

La barra inferior mapea el angulo del servo al ancho completo del frame con **113 centrado exactamente en la mitad horizontal**:

```
|  80  ←── mitad izquierda ──→  113  ←── mitad derecha ──→  146  |
|            33 grados            |           33 grados           |
```

Al ser el universo simetrico, la escala es identica a ambos lados: cada pixel representa la misma cantidad de grados tanto a la izquierda como a la derecha del centro. Esto corrige la visualizacion de la version anterior donde la escala era desigual.

Marcas de referencia en la barra: **80**, **113**, **146**.

### 6.4 Ventana secundaria

Se muestra la imagen binarizada del ROI (`"Umbral Linea (ROI)"`) para verificar visualmente la deteccion.

---

## 7. Flujo de Ejecucion

```
1. Inicializar ControlDifuso (crear MFs de salida)
2. Inicializar Picamera2 (640x480)
3. Conectar a broker MQTT (no bloqueante, continua si falla)
4. BUCLE:
   a. Capturar frame
   b. Convertir a escala de grises
   c. Recortar ROI (40% inferior)
   d. Binarizar (umbral 200)
   e. Limpiar con morfologia (apertura + cierre)
   f. Buscar contornos
   g. Si hay contorno valido (area > 300 px):
      - Calcular centroide
      - Normalizar posicion [0, 1]
      - Ejecutar control difuso → angulo
      - Si cambio > 2°: publicar por MQTT
   h. Dibujar HUD
   i. Mostrar frame + umbral
   j. Si ESC: salir
5. Liberar recursos (camara, MQTT, ventanas)
```

---

## 8. Ejemplo Paso a Paso

**Entrada:** posicion normalizada = 0.80 (linea cerca del borde derecho)

### Fuzzificacion

| Conjunto | Grado de pertenencia |
|---|---|
| Muy Izquierda | 0.00 |
| Med Izquierda | 0.00 |
| Poco Izquierda | 0.00 |
| Centro | 0.00 |
| Poco Derecha | 0.00 |
| Med Derecha | 1.00 |
| Muy Derecha | 0.00 |

### Evaluacion de reglas

| Regla | Activacion |
|---|---|
| R6 (Med Der → Med Der) | 1.00 |
| R11 (Med Der AND NOT Poco Der → Med Der) | min(1.00, 1.00) = 1.00 |
| Resto | 0.00 |

### Activaciones finales

| Salida | Nivel |
|---|---|
| Med Derecha | max(1.00, 1.00) = 1.00 |
| Centro | R15 = min(..., 0.00, ...) = 0.00 |
| Resto | 0.00 |

### Defuzzificacion

Con solo Med Derecha activa al 100%: centroide de trimf [124, 135, 146] = **135 grados**

En la version anterior, posicion 0.80 activaba Med Derecha con parametros [120, 127, 135] → centroide ≈ 127 grados. Ahora con el universo simetrico, la respuesta es 135 grados, proporcionando un giro mas pronunciado hacia la derecha.

---

## 9. Ejecucion

```bash
# Asegurarse de que Mosquitto esta corriendo
sudo systemctl start mosquitto

# Ejecutar el seguidor de linea
python3 ControlLinea_Raspberry_v2.py
```

Presionar **ESC** para finalizar el programa.

---

## 10. Configuracion de Red

| Dispositivo | Direccion | Rol |
|---|---|---|
| Raspberry Pi | 127.0.0.1 | Broker MQTT + Vision + Control |
| ESP32 | (en la misma red WiFi) | Suscriptor: recibe angulo del servo |
