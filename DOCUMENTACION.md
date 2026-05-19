# Vision-Integrador — Documentacion del Proyecto

> **Estado:** Prototipo funcional
> **Version:** 5.0
> **Autor:** Josue (im221466@itsatlixco.edu.mx)
> **Fecha de inicio:** Marzo 2026

---

## Tabla de Contenidos

1. [Que es el proyecto](#1-que-es-el-proyecto)
2. [Para que sirve](#2-para-que-sirve)
3. [Arquitectura general](#3-arquitectura-general)
4. [Componentes del sistema](#4-componentes-del-sistema)
   - [4.1 Seguidor de Linea (skfuzzy)](#41-seguidor-de-linea-skfuzzy--controllinea_skfuzzypy)
   - [4.2 Demo en PC](#42-demo-en-pc--controllinea_pc_demopy)
   - [4.3 Interfaz SCADA](#43-interfaz-scada-scadapy)
   - [4.4 Firmware ESP32 Ackermann](#44-firmware-esp32-ackermann-controlackermanmqttcontrolackermanmqttino)
   - [4.5 Lector de Bateria](#45-lector-de-bateria--lectorbaterialectorbateriaino)
5. [Control Difuso — Detalle completo](#5-control-difuso--detalle-completo)
   - [5.1 Variable de entrada](#51-variable-de-entrada)
   - [5.2 Conjuntos difusos de entrada (7)](#52-conjuntos-difusos-de-entrada-7)
   - [5.3 Variable de salida](#53-variable-de-salida)
   - [5.4 Conjuntos difusos de salida (7)](#54-conjuntos-difusos-de-salida-7)
   - [5.5 Funciones de membresia](#55-funciones-de-membresia)
   - [5.6 Fuzzificacion](#56-fuzzificacion)
   - [5.7 Base de reglas (7 reglas)](#57-base-de-reglas-7-reglas)
   - [5.8 Inferencia Mamdani](#58-inferencia-mamdani)
   - [5.9 Defuzzificacion — Centro de Gravedad](#59-defuzzificacion--centro-de-gravedad)
   - [5.10 Comportamiento al perder la linea](#510-comportamiento-al-perder-la-linea)
6. [Sistema de paro de emergencia (E-Stop)](#6-sistema-de-paro-de-emergencia-e-stop)
7. [Registro de eventos (CSV) y reportes PDF](#7-registro-de-eventos-csv-y-reportes-pdf)
8. [Tecnologias utilizadas](#8-tecnologias-utilizadas)
9. [Configuracion de red y hardware](#9-configuracion-de-red-y-hardware)
   - [9.1 Red y MQTT](#91-red-y-mqtt)
   - [9.2 Pines GPIO del ESP32](#92-pines-gpio-del-esp32)
   - [9.3 Parametros de vision](#93-parametros-de-vision)
10. [Estructura de archivos](#10-estructura-de-archivos)
11. [Flujo de datos](#11-flujo-de-datos)
12. [Requisitos para ejecutar el proyecto](#12-requisitos-para-ejecutar-el-proyecto)

---

## 1. Que es el proyecto

**Vision-Integrador** es un sistema de control para un AGV (Automated Guided Vehicle) a escala 1:10 con direccion **Ackermann** que integra:

- **Vision artificial** en tiempo real (procesada en Raspberry Pi 4B con OpenCV).
- **Control difuso tipo Mamdani** implementado con scikit-fuzzy (skfuzzy), con 7 conjuntos de entrada, 7 de salida y 7 reglas para calcular el angulo del servo de direccion de forma continua y suave.
- **Seguimiento de linea blanca** sobre fondo negro usando Picamera2, con ROI configurable y deteccion por umbral + contornos.
- **Recuperacion al perder la linea**: al perder la linea, el servo gira al maximo hacia el lado donde la vio por ultima vez para intentar reencontrarla.
- **Comunicacion inalambrica** mediante protocolo MQTT (broker Mosquitto en Raspberry Pi).
- **Control de hardware** a traves de un microcontrolador ESP32 con servo de direccion y motor DC (TB6612FNG).
- **Monitoreo de bateria** con divisor de voltaje (30kΩ / 7.5kΩ) para bateria Li-ion 12V, publicando porcentaje (0-100%) via MQTT.
- **Lectura de temperatura** con sensor LM35.
- **Sistema de arranque/paro/emergencia** dual (fisico + virtual).
- **Interfaz SCADA** con focos de estado, grafica de temperatura, barra de nivel de bateria, botones de control y generacion de reportes PDF.
- **Registro de eventos** en archivo CSV con trazabilidad completa.

El sistema detecta una linea blanca con la camara, normaliza su posicion horizontal en el frame [0, 1], y el controlador difuso calcula un angulo de servo continuo [80, 135] grados que se envia al ESP32 via MQTT para dirigir el vehiculo.

---

## 2. Para que sirve

| Aplicacion | Descripcion |
|---|---|
| AGV con direccion Ackermann | Vehiculo que sigue una linea blanca con control difuso de direccion |
| Control difuso aplicado | Implementacion del metodo Mamdani con scikit-fuzzy y defuzzificacion por centroide |
| Automatizacion y SCADA | Monitoreo en tiempo real con interfaz grafica, alertas, nivel de bateria y reportes |
| Educacion | Integracion de vision artificial + logica difusa + IoT + microcontroladores |

---

## 3. Arquitectura general

```
+-----------------------------------------------------------------+
|                      RASPBERRY PI 4B                            |
|                                                                 |
|   +-------------+      +-----------------------------------+   |
|   | Camara CSI  | ---> | ControlLinea_skfuzzy.py            |   |
|   | Picamera2   |      | - Deteccion de linea blanca       |   |
|   +-------------+      | - ROI inferior (40% del frame)     |   |
|                         | - Control difuso Mamdani (skfuzzy)|   |
|                         | - Calcula angulo servo [80, 135]  |   |
|                         | - Busqueda al perder linea        |   |
|                         | - Publica angulo via MQTT         |   |
|                         +----------------+------------------+   |
|                                          |                      |
|   +--------------------------------------+------------------+   |
|   | Scada.py (Tkinter)                                      |   |
|   | - 3 focos: PARO / MOVIMIENTO / EMERGENCIA               |   |
|   | - Botones: ARRANQUE / PARO / E-STOP / RESET / REPORTE  |   |
|   | - Grafica de temperatura en tiempo real (60s)           |   |
|   | - Barra de nivel de bateria (0-100%)                    |   |
|   | - Alertas: temp > 40C, bateria < 20%                    |   |
|   | - Registro CSV + Reportes PDF                           |   |
|   +--------------------------------------+------------------+   |
|                                          |                      |
|   +--------------------------------------+------------------+   |
|   |           Broker MQTT (Mosquitto)                       |   |
|   |           Puerto: 1883                                  |   |
|   +--------------------------------------+------------------+   |
+-----------------------------------------------------------------+
                                           | MQTT (WiFi)
                                           v
                         +------------------------------+
                         |     ESP32 (Ackermann)        |
                         |                              |
                         |  Recibe: angulo 80-135       |
                         |    (esp32/servo/control)     |
                         |  Recibe: GO, STOP,           |
                         |    EMERGENCIA, RESET_E...    |
                         |    (esp32/arranque)          |
                         |  Publica: temperatura,       |
                         |    bateria %, estado         |
                         |                              |
                         |  GPIO 5  -- Servo direccion  |
                         |  GPIO 16 -- LED Izquierdo    |
                         |  GPIO 15 -- LED Derecho      |
                         |  GPIO 2  -- LED Estado (ON)  |
                         |  GPIO 23 -- LED Estado (PARO)|
                         |  GPIO 12 -- Boton Encender   |
                         |  GPIO 4  -- Boton Apagar     |
                         |  GPIO 13 -- E-Stop (NC)      |
                         |  GPIO 34 -- Bateria (ADC)    |
                         |  GPIO 36 -- Sensor LM35      |
                         +------------------------------+
```

---

## 4. Componentes del sistema

### 4.1 Seguidor de Linea (skfuzzy) — `ControlLinea_skfuzzy.py`

**Lenguaje:** Python 3
**Dependencias:** `opencv-python`, `numpy`, `scikit-fuzzy`, `scipy`, `networkx`, `paho-mqtt`, `picamera2`
**Ejecutar en:** Raspberry Pi 4B con camara CSI y Mosquitto local

Archivo principal del sistema. Contiene el controlador difuso (via scikit-fuzzy) y el bucle de vision para seguir una linea blanca.

#### Que hace

1. **Crea el sistema difuso** con scikit-fuzzy: 7 conjuntos de entrada, 7 de salida, 7 reglas directas, defuzzificacion por centroide.
2. **Captura video** desde la camara CSI con Picamera2 a 640x480.
3. **Convierte** cada fotograma a escala de grises.
4. **Aplica ROI** sobre la franja inferior del frame (`ROI_PROPORCION = 0.4`, el 40% de abajo).
5. **Umbraliza** con `cv2.threshold` (`UMBRAL_BLANCO = 200`) y limpia con morfologia (apertura + cierre).
6. **Detecta contornos** y toma el mas grande como la linea principal (descarta area < 300 px).
7. **Calcula el centroide** con momentos y **normaliza la posicion** horizontal al rango [0, 1].
8. **Ejecuta el pipeline difuso**: fuzzificacion -> 7 reglas -> defuzzificacion por centroide.
9. **Al perder la linea**: gira el servo al maximo hacia el lado donde la vio por ultima vez (busqueda activa).
10. **Publica el angulo** resultante al ESP32 via MQTT (solo si el cambio es mayor a 2 grados).
11. **Muestra HUD** con: zona central verde, contorno de la linea, centroide, angulo del servo, etiqueta de direccion y barra indicadora inferior.

#### Constantes ajustables

| Constante | Valor por defecto | Descripcion |
|---|---|---|
| `UMBRAL_BLANCO` | 200 | Umbral de binarizacion (0-255). Subir si hay falsos positivos, bajar si no detecta |
| `MIN_AREA` | 300 | Area minima de contorno en px para filtrar ruido |
| `ROI_PROPORCION` | 0.4 | Fraccion inferior del frame a analizar. Ajustar segun altura de la camara |

> Ver seccion 5 para el detalle completo del control difuso.

---

### 4.2 Demo en PC — `ControlLinea_PC_Demo.py`

**Lenguaje:** Python 3
**Dependencias:** `opencv-python`, `numpy`, `scikit-fuzzy`
**Ejecutar en:** PC con webcam USB

Version de demostración que corre en PC sin hardware. Misma logica de deteccion y control difuso, pero:
- Usa `cv2.VideoCapture(0)` (webcam USB) en vez de Picamera2.
- **No usa MQTT** (no envia comandos al ESP32).
- Incluye un **panel de visualizacion del control difuso** en tiempo real que muestra: funciones de membresia de entrada/salida con activaciones, barras de activacion de las 7 reglas y resumen numerico.

---

### 4.3 Interfaz SCADA — `Scada.py`

**Lenguaje:** Python 3
**Dependencias:** `tkinter`, `paho-mqtt`, `reportlab`, `matplotlib`

Panel de control y monitoreo con tema oscuro.

#### Que tiene

1. **3 focos indicadores de estado:**
   - Rojo: PARO
   - Verde: MOVIMIENTO
   - Amarillo: EMERGENCIA
   - Solo uno encendido a la vez, con label del estado actual.

2. **5 botones de control:**
   - **ARRANQUE** (verde): publica `"GO"` al topic `esp32/arranque`.
   - **PARO** (rojo): publica `"STOP"`.
   - **E-STOP** (amarillo): publica `"EMERGENCIA"`.
   - **RESET E-STOP** (gris): publica `"RESET_EMERGENCIA"`.
   - **GENERAR REPORTE** (azul): genera reporte PDF con grafica de temperatura y tabla de eventos.

3. **Grafica de temperatura en tiempo real:**
   - 60 segundos de historial con linea suavizada.
   - Cuadricula con etiquetas cada 10 C.
   - Linea roja de alerta en 40 C.
   - Punto indicador en el ultimo valor.

4. **Barra de nivel de bateria:**
   - Representacion visual de 0 a 100%.
   - Marca de referencia en 20%.
   - Colores: rojo < 20%, amarillo 20-50%, cyan > 50%.

5. **Sistema de alertas:**
   - Temperatura > 40 C: alerta de temperatura critica.
   - Bateria < 20%: alerta de bateria baja.
   - Solo se muestra una vez hasta que el valor vuelva a rango normal.
   - Las alertas se registran en el CSV.

6. **Generacion de reportes PDF:**
   - Grafica de temperatura exportada como imagen.
   - Tabla con todos los eventos registrados.
   - Conteo de alarmas.

---

### 4.4 Firmware ESP32 Ackermann — `ControlAckermanMQTT/ControlAckermanMQTT.ino`

**Lenguaje:** C++ (Arduino)
**Librerias:** `WiFi.h`, `PubSubClient.h`, `ESP32Servo.h`

Firmware del microcontrolador que controla el servo de direccion Ackermann y el motor DC.

#### Que hace

1. **Conecta al WiFi** y al **broker MQTT**.
2. **Se suscribe** a los topics `esp32/servo/control` y `esp32/arranque`.
3. **Servo de direccion** en GPIO 5 (pulso 500-2400 us):
   - Recibe angulos continuos (80-135) del controlador difuso.
   - Valida rango mecanico del Ackermann y escribe al servo.
   - Posicion inicial y por defecto: 113 grados (recto).
4. **Motor DC** via driver TB6612FNG (PWM fijo = 50, 0-255).
5. **LEDs indicadores de direccion** (zona muerta +-3 grados alrededor de 113):
   - Angulo < 110: LED izquierdo encendido (GPIO 16).
   - Angulo > 116: LED derecho encendido (GPIO 15).
   - 110-116: ambos LEDs encendidos (recto).
6. **Sistema de arranque/paro** con tres modos:
   - **Botones fisicos**: ON (GPIO 12) y OFF (GPIO 4) con debounce de 50ms.
   - **Comandos MQTT**: `"GO"`, `"STOP"`, `"EMERGENCIA"`, `"RESET_EMERGENCIA"`.
   - **E-Stop fisico**: Boton NC en GPIO 13 con interrupcion de hardware y debounce en ISR.
7. **Lectura de bateria** cada segundo (GPIO 34): promedio de 20 muestras ADC, divisor de voltaje (30kΩ / 7.5kΩ, factor 5.0), calcula porcentaje (0-100%) y publica en `bateria/porcentaje`.
8. **Lectura de temperatura** cada segundo: sensor LM35 (GPIO 36), publica en `LM35/uno`.
9. **Publica estado** (`"MOVIMIENTO"`, `"PARO"`, `"EMERGENCIA"`) solo cuando cambia.
10. **Reconexion automatica** WiFi y MQTT.

---

### 4.5 Lector de Bateria — `LectorBateria/LectorBateria.ino`

**Lenguaje:** C++ (Arduino)

Utilidad independiente para probar la lectura de voltaje de la bateria por serial. Lee el ADC en GPIO 34, promedia 20 muestras, aplica el factor del divisor de voltaje (5.0) y muestra voltaje y porcentaje por Serial Monitor cada 5 segundos.

---

## 5. Control Difuso — Detalle completo

El sistema implementa un **controlador difuso tipo Mamdani** usando la libreria scikit-fuzzy (skfuzzy) con las siguientes etapas:

```
Posicion normalizada [0,1] --> Fuzzificacion (7 MFs)
                                      |
                                      v
                              Evaluacion de 7 reglas
                                      |
                                      v
                              Inferencia (recorte + agregacion)
                                      |
                                      v
                              Defuzzificacion (centroide)
                                      |
                                      v
                              Angulo del servo [80, 135]
```

### 5.1 Variable de entrada

**Posicion normalizada del objeto en el frame**

- **Universo de discurso:** [0, 1]
- **0.0** = borde izquierdo del frame
- **0.5** = centro del frame
- **1.0** = borde derecho del frame

Se calcula como:

```
posicion_norm = centro_x_objeto / ancho_frame
```

### 5.2 Conjuntos difusos de entrada (7)

| # | Conjunto | Tipo | Parametros | Descripcion |
|---|----------|------|-----------|-------------|
| 1 | **Muy Izquierda** | Trapezoidal | [0.0, 0.0, 0.05, 0.20] | Maximo de 0 a 0.05, decrece hasta 0.20 |
| 2 | **Med Izquierda** | Triangular | [0.05, 0.20, 0.35] | Pico en 0.20, de 0.05 a 0.35 |
| 3 | **Poco Izquierda** | Triangular | [0.20, 0.35, 0.50] | Pico en 0.35, de 0.20 a 0.50 |
| 4 | **Centro** | Triangular | [0.35, 0.50, 0.65] | Pico en 0.50, de 0.35 a 0.65 |
| 5 | **Poco Derecha** | Triangular | [0.50, 0.65, 0.80] | Pico en 0.65, de 0.50 a 0.80 |
| 6 | **Med Derecha** | Triangular | [0.65, 0.80, 0.95] | Pico en 0.80, de 0.65 a 0.95 |
| 7 | **Muy Derecha** | Trapezoidal | [0.80, 0.95, 1.0, 1.0] | Crece desde 0.80, maximo de 0.95 a 1.0 |

#### Grafica de los 7 conjuntos de entrada

```
u
1.0 +##                                                               ##
    |###           .                .        .               .       ###
0.8 +####         / \              / \      / \             / \     ####
    |#####       /   \            /   \    /   \           /   \   #####
0.6 +######     /     \          /     \  /     \         /     \ ######
    |#######   /       \        /       \/       \       /       #######
0.4 +######## /         \      /        /\        \     /       ########
    |########/           \    /        /  \        \   /       #########
0.2 +########              \ /        /    \        \ /        #########
    |#########              X        /      \        X        ##########
0.0 +----------+-----+-----+--+-----+--+---+--+----+--+-----+---------+
    0.0  0.05  0.20  0.35  0.50  0.65  0.80  0.95  1.0

    Muy   Med   Poco         Poco   Med    Muy
    Izq   Izq   Izq  Centro  Der    Der    Der
```

**Distribucion:** Los 7 conjuntos estan distribuidos simetricamente sobre [0, 1] con espaciado de 0.15 entre picos. Los conjuntos extremos (Muy Izquierda y Muy Derecha) son trapezoidales para cubrir completamente los bordes. Todos los conjuntos adyacentes se traslapan para garantizar transiciones suaves.

### 5.3 Variable de salida

**Angulo del servo de direccion**

- **Universo de discurso:** [80, 135] grados (discretizado en 56 puntos, resolucion de 1 grado)
- **80 grados**  = giro maximo a la izquierda
- **113 grados** = linea recta (sin giro)
- **135 grados** = giro maximo a la derecha

> El rango mecanico real del Ackermann es **asimetrico**: 33 grados disponibles
> a la izquierda de 113 (80-113) y solo 22 grados a la derecha (113-135).
> Los conjuntos difusos de salida estan distribuidos de forma asimetrica
> para cubrir ambos lados con 3 regiones cada uno.

### 5.4 Conjuntos difusos de salida (7)

Todos los conjuntos de salida son **triangulares**:

| # | Conjunto | Parametros [a, b, c] | Rango efectivo | Centro |
|---|----------|---------------------|----------------|--------|
| 1 | **Muy Izquierda**  | [80, 80, 91]    | 80 - 91    | 80  |
| 2 | **Med Izquierda**  | [80, 91, 102]   | 80 - 102   | 91  |
| 3 | **Poco Izquierda** | [91, 102, 113]  | 91 - 113   | 102 |
| 4 | **Centro**         | [102, 113, 120] | 102 - 120  | 113 |
| 5 | **Poco Derecha**   | [113, 120, 127] | 113 - 127  | 120 |
| 6 | **Med Derecha**    | [120, 127, 135] | 120 - 135  | 127 |
| 7 | **Muy Derecha**    | [127, 135, 135] | 127 - 135  | 135 |

#### Distribucion visual de los conjuntos de salida

```
        Muy    Med    Poco         Poco   Med    Muy
        Izq    Izq    Izq  Centro  Der    Der    Der
         ^      ^      ^     ^      ^      ^      ^
        / \    /  \   /  \   / \   /  \  /  \    / \
       /   \  /    \ /    \ /   \ /    \/    \  /   \
      /     \/      X      X     X      X     \/     \
     /      /\     / \    / \   / \    / \    /\      \
    /      /  \   /   \  /   \ /   \  /   \  /  \      \
   +------+---+--+----+-+-----++----+-+----+-+---+------+
   80     91   102    113     120   127     135
```

### 5.5 Funciones de membresia

El sistema implementa dos tipos de funciones de membresia:

**Triangular** (`trimf`): definida por tres parametros [a, b, c]

```
         b (pico = 1.0)
        / \
       /   \
      /     \
     /       \
----a---------c----  (base, mu = 0)
```

```
trimf(x, [a, b, c]):
    si a <= x <= b:  mu = (x - a) / (b - a)
    si b < x <= c:   mu = (c - x) / (c - b)
    en otro caso:    mu = 0
```

**Trapezoidal** (`trapmf`): definida por cuatro parametros [a, b, c, d]

```
     b----------c  (meseta = 1.0)
    / \          \
   /   \          \
  /     \          \
-a-------\----------d--  (base, mu = 0)
```

```
trapmf(x, [a, b, c, d]):
    si a <= x < b:   mu = (x - a) / (b - a)
    si b <= x <= c:  mu = 1.0
    si c < x <= d:   mu = (d - x) / (d - c)
    en otro caso:    mu = 0
```

### 5.6 Fuzzificacion

La fuzzificacion evalua la posicion normalizada contra cada uno de los 7 conjuntos de entrada, obteniendo 7 grados de pertenencia:

```
mu_muy_izq  = trapmf(pos, [0.0, 0.0, 0.05, 0.20])
mu_med_izq  = trimf(pos, [0.05, 0.20, 0.35])
mu_poco_izq = trimf(pos, [0.20, 0.35, 0.50])
mu_cen      = trimf(pos, [0.35, 0.50, 0.65])
mu_poco_der = trimf(pos, [0.50, 0.65, 0.80])
mu_med_der  = trimf(pos, [0.65, 0.80, 0.95])
mu_muy_der  = trapmf(pos, [0.80, 0.95, 1.0, 1.0])
```

### 5.7 Base de reglas (7 reglas)

El sistema usa 7 reglas directas implementadas con `skfuzzy.control.Rule`. Cada conjunto de entrada mapea directamente a su conjunto de salida correspondiente:

| # | Regla | Proposito |
|---|-------|-----------|
| **R1** | IF posicion = Muy_Izquierda THEN servo = Muy_Izquierda | Linea en extremo izquierdo -> giro maximo izquierdo (80°) |
| **R2** | IF posicion = Med_Izquierda THEN servo = Med_Izquierda | Linea a la izquierda -> giro medio izquierdo (91°) |
| **R3** | IF posicion = Poco_Izquierda THEN servo = Poco_Izquierda | Linea ligeramente a la izquierda -> correccion leve (102°) |
| **R4** | IF posicion = Centro THEN servo = Centro | Linea centrada -> mantener recto (113°) |
| **R5** | IF posicion = Poco_Derecha THEN servo = Poco_Derecha | Linea ligeramente a la derecha -> correccion leve (120°) |
| **R6** | IF posicion = Med_Derecha THEN servo = Med_Derecha | Linea a la derecha -> giro medio derecho (127°) |
| **R7** | IF posicion = Muy_Derecha THEN servo = Muy_Derecha | Linea en extremo derecho -> giro maximo derecho (135°) |

#### Codigo de las reglas (skfuzzy)

```python
r1 = ctrl.Rule(posicion['muy_izq'], servo['muy_izq'])
r2 = ctrl.Rule(posicion['med_izq'], servo['med_izq'])
r3 = ctrl.Rule(posicion['poco_izq'], servo['poco_izq'])
r4 = ctrl.Rule(posicion['centro'], servo['centro'])
r5 = ctrl.Rule(posicion['poco_der'], servo['poco_der'])
r6 = ctrl.Rule(posicion['med_der'], servo['med_der'])
r7 = ctrl.Rule(posicion['muy_der'], servo['muy_der'])
```

Las transiciones suaves entre reglas se logran gracias al **traslape de los conjuntos de entrada** (espaciado de 0.15 entre picos). Cuando la posicion cae en una zona de traslape, dos reglas se activan simultaneamente y la defuzzificacion por centroide produce un angulo intermedio.

### 5.8 Inferencia Mamdani

El metodo de inferencia Mamdani sigue estos pasos para cada conjunto de salida:

1. **Recorte (clipping):** La funcion de membresia de salida se recorta al nivel de activacion de la regla correspondiente:
   ```
   MF_recortada(x) = min(MF_original(x), nivel_activacion)
   ```

2. **Agregacion:** Todas las funciones de membresia recortadas se combinan usando el operador maximo (union difusa):
   ```
   MF_agregada(x) = max(MF_recortada_1(x), MF_recortada_2(x), ..., MF_recortada_7(x))
   ```

### 5.9 Defuzzificacion — Centro de Gravedad

Se calcula el centroide del area bajo la curva agregada:

```
            SUM(x_i * mu_agregada(x_i))
angulo = -----------------------------------
              SUM(mu_agregada(x_i))
```

Donde `x_i` recorre el universo discretizado de 80 a 135 (56 puntos con resolucion de 1 grado).

Si el area total es cero (sin deteccion o sin activacion), se retorna **113 grados** (recto) como valor por defecto.

### 5.10 Comportamiento al perder la linea

Cuando la camara no detecta la linea (sin contorno valido), el sistema **no vuelve al centro**. En su lugar, realiza una **busqueda activa** girando el servo al maximo hacia el lado donde vio la linea por ultima vez:

```
ultima_posicion < 0.5  -->  servo = 80°  (full izquierda)
ultima_posicion >= 0.5 -->  servo = 135° (full derecha)
```

Esto permite al AGV intentar reencontrar la linea en curvas cerradas o cuando se sale momentaneamente del trazado.

```
         Linea detectada                    Linea perdida
        (control difuso)               (busqueda activa)

  [Camara ve linea]                [Camara NO ve linea]
         |                                   |
         v                                   v
  posicion_norm = cx/width         Revisar ultima_posicion
         |                              /            \
         v                            <0.5          >=0.5
  skfuzzy calcula angulo              |               |
  [80° - 135°]                  servo = 80°      servo = 135°
         |                      (full izq)       (full der)
         v                              \            /
  Guarda ultima_posicion                 v          v
         |                        [Sigue girando hasta
         v                         reencontrar la linea]
  Envia angulo MQTT
```

---

## 6. Sistema de paro de emergencia (E-Stop)

El sistema cuenta con paro de emergencia dual (fisico y virtual):

### E-Stop fisico (GPIO 13)

- **Boton normalmente cerrado (NC)**: en operacion normal el circuito esta cerrado (GPIO lee LOW).
- **Al presionar o si el cable se rompe**: el circuito se abre (GPIO lee HIGH) -> **fail-safe**.
- Usa **interrupcion de hardware** (`attachInterrupt`, `RISING`) para respuesta en microsegundos.
- La ISR (`isrEmergencia`) apaga LEDs y centra el servo inmediatamente.

### E-Stop virtual (interfaz SCADA)

- El boton **E-STOP** de la interfaz publica `"EMERGENCIA"` al topic `esp32/arranque`.
- El ESP32 activa el flag `emergencia` y el flag `flag` (para distinguir de emergencia fisica).
- Mismo efecto que el boton fisico: apaga todo, centra servo, bloquea el sistema.

### Reset de emergencia

- **E-Stop fisico**: al soltar el boton (NC vuelve a cerrar), el flag se limpia pero NO reanuda automaticamente. Requiere presionar ON o enviar GO.
- **E-Stop virtual**: requiere enviar `"RESET_EMERGENCIA"` Y que el boton fisico no este presionado.

```
                    +-------------------------+
                    |   ESTADO: OPERANDO      |
                    +------------+------------+
                                 |
              +------------------+------------------+
              |                  |                  |
     [Boton NC fisico]   [E-STOP interfaz]   [Cable roto]
     GPIO 13 -> HIGH     MQTT "EMERGENCIA"   GPIO 13 -> HIGH
              |                  |                  |
              +------------------+------------------+
                                 |
                                 v
                    +-------------------------+
                    |   ESTADO: EMERGENCIA    |
                    |   - Servo centrado 113  |
                    |   - LEDs apagados       |
                    |   - Sistema bloqueado   |
                    |   - Registrado en CSV   |
                    +------------+------------+
                                 |
              +------------------+------------------+
              |                                     |
     [Soltar boton NC]                [RESET_EMERGENCIA]
     (limpia flag, no reanuda)        (solo si NC cerrado)
              |                                     |
              +------------------+------------------+
                                 |
                                 v
                    +-------------------------+
                    |   ESTADO: PARO          |
                    |   Esperando ON o GO     |
                    +-------------------------+
```

---

## 7. Registro de eventos (CSV) y reportes PDF

La interfaz SCADA registra automaticamente todos los eventos del sistema en `eventos.csv`.

### Formato del archivo CSV

| Columna | Descripcion |
|---|---|
| `Fecha` | Fecha del evento (YYYY-MM-DD) |
| `Hora` | Hora del evento (HH:MM:SS) |
| `arranque/paro` | Tipo: `MOVIMIENTO`, `PARO`, `EMERGENCIA`, `ALERTA_TEMP`, `ALERTA_BAT` |
| `Origen` | Fuente: `VIRTUAL` (interfaz), `FISICA` (botones ESP32), `SENSOR` (alertas) |
| `LM35/uno` | Temperatura al momento del evento (C) |
| `bateria/%` | Porcentaje de bateria al momento del evento (0-100) |

### Ejemplo

```csv
Fecha,Hora,arranque/paro,Origen,LM35/uno,bateria/%
2026-03-16,10:15:30,MOVIMIENTO,VIRTUAL,24.5,85.0
2026-03-16,10:16:45,PARO,FISICA,25.1,82.3
2026-03-16,10:20:12,ALERTA_TEMP,SENSOR,41.2,78.0
2026-03-16,10:22:05,EMERGENCIA,VIRTUAL,42.0,75.5
2026-03-16,10:23:00,ALERTA_BAT,SENSOR,38.5,15.2
```

### Reportes PDF

El boton **GENERAR REPORTE** en la interfaz SCADA crea un archivo `Reporte_SCADA_YYYY-MM-DD.pdf` que incluye:
- Titulo y fecha.
- Conteo de alarmas registradas.
- Grafica de temperatura (exportada como imagen PNG).
- Tabla completa de eventos del CSV.

---

## 8. Tecnologias utilizadas

| Tecnologia | Rol |
|---|---|
| **Python 3** | Control difuso, vision, interfaz SCADA |
| **OpenCV (`cv2`)** | Captura de video, procesamiento de imagen, deteccion de linea |
| **scikit-fuzzy (`skfuzzy`)** | Sistema de control difuso Mamdani (requiere scipy y networkx) |
| **NumPy** | Arrays para procesamiento de imagen y calculo numerico |
| **Picamera2** | Captura de video desde camara CSI en Raspberry Pi |
| **Tkinter** | Interfaz grafica SCADA |
| **paho-mqtt** | Cliente MQTT para Python |
| **ReportLab** | Generacion de reportes PDF |
| **Matplotlib** | Graficas de temperatura para los reportes |
| **MQTT (protocolo)** | Comunicacion ligera publicador-suscriptor |
| **Mosquitto** | Broker MQTT (en Raspberry Pi) |
| **Raspberry Pi 4B** | Ejecuta vision, control difuso, interfaz y broker MQTT |
| **ESP32** | Microcontrolador con WiFi: servo, motor, sensores, botones |
| **TB6612FNG** | Driver de motor DC |
| **ESP32Servo** | Libreria para control de servo en ESP32 |
| **PubSubClient** | Libreria MQTT para Arduino/ESP32 |

---

## 9. Configuracion de red y hardware

### 9.1 Red y MQTT

> **Nota:** Estas configuraciones estan hardcodeadas en los archivos fuente.

| Parametro | Valor en ControlLinea_skfuzzy.py | Valor en ESP32 | Valor en Scada.py |
|---|---|---|---|
| SSID WiFi | — | `A52 de Roberto` | — |
| Contrasena WiFi | — | `123456789` | — |
| IP del broker MQTT | `127.0.0.1` | `10.249.23.191` | `10.184.97.191` |
| Puerto MQTT | `1883` | `1883` | `1883` |

**Topics MQTT:**

| Topic | Publicador | Suscriptor | Contenido |
|---|---|---|---|
| `esp32/servo/control` | ControlLinea_skfuzzy.py | ESP32 | Angulo del servo (80-135, recto=113) |
| `esp32/arranque` | Scada.py | ESP32 | GO, STOP, EMERGENCIA, RESET_EMERGENCIA |
| `arranque/paro` | ESP32 | Scada.py | MOVIMIENTO, PARO, EMERGENCIA |
| `bateria/porcentaje` | ESP32 | Scada.py | Porcentaje de bateria (0-100%) |
| `LM35/uno` | ESP32 | Scada.py | Temperatura en grados C |

### 9.2 Pines GPIO del ESP32

| Pin GPIO | Nombre en codigo | Funcion |
|---|---|---|
| GPIO 5 | `PIN_SERVO` | Servo de direccion Ackermann (PWM 500-2400 us) |
| GPIO 16 | `ledizq` | LED indicador de giro izquierdo |
| GPIO 15 | `ledder` | LED indicador de giro derecho |
| GPIO 2 | `PIN_LED` | LED de estado encendido |
| GPIO 23 | `PIN_LEDPARO` | LED de estado en paro |
| GPIO 12 | `PIN_BOTON_ON` | Boton de encendido (INPUT_PULLUP) |
| GPIO 4 | `PIN_BOTON_OFF` | Boton de apagado (INPUT_PULLUP) |
| GPIO 13 | `PIN_ESTOP` | Boton E-Stop NC (INPUT_PULLUP, interrupcion RISING) |
| GPIO 34 | `pinBateria` | Lectura de bateria via divisor de voltaje 30kΩ/7.5kΩ (ADC, 12 bits) |
| GPIO 36 | `pinLM35` | Lectura de sensor LM35 (ADC, 12 bits) |
| GPIO 18 | `AIN1` | Motor DC - direccion (TB6612FNG) |
| GPIO 19 | `AIN2` | Motor DC - direccion (TB6612FNG) |
| GPIO 21 | `PWMA` | Motor DC - PWM velocidad (TB6612FNG) |
| GPIO 22 | `STBY` | Motor DC - standby (TB6612FNG) |

### 9.3 Parametros de vision

| Parametro | Valor | Descripcion |
|---|---|---|
| Camara | Picamera2 (CSI) | Camara CSI de la Raspberry Pi, 640x480 |
| `UMBRAL_BLANCO` | `200` | Umbral de binarizacion para detectar linea blanca (0-255) |
| `MIN_AREA` | `300 px` | Area minima de contorno para filtrar ruido |
| `ROI_PROPORCION` | `0.4` | Fraccion inferior del frame a analizar (40% de abajo) |
| Umbral de cambio servo | `2 grados` | Solo envia al ESP32 si el cambio supera este umbral |
| Delay por fotograma | `1 ms` | `cv2.waitKey(1)` — ESC para salir |

---

## 10. Estructura de archivos

```
Vision/
|
|-- ControlLinea_skfuzzy.py        # Control difuso (skfuzzy) + vision + MQTT (archivo principal)
|-- ControlLinea_PC_Demo.py        # Demo en PC con webcam y panel de visualizacion difusa
|-- Scada.py                       # Interfaz SCADA: monitoreo, control, CSV, reportes PDF
|-- eventos.csv                    # Registro de eventos (generado automaticamente)
|
|-- ControlAckermanMQTT/
|   |-- ControlAckermanMQTT.ino    # Firmware ESP32: servo + motor + bateria + sensores + E-Stop
|
|-- LectorBateria/
|   |-- LectorBateria.ino          # Utilidad para probar lectura de bateria por serial
|
|-- DOCUMENTACION.md               # Este archivo
```

---

## 11. Flujo de datos

```
[Camara CSI] --> [Fotograma BGR]
                       |
                       v
              [Escala de grises]
                       |
                       v
              [ROI: 40% inferior]
                       |
                       v
              [Umbral > 200 = blanco]
              [Morfologia: apertura + cierre]
                       |
                       v
             [Deteccion de contornos]
                       |
                 Hay linea > 300px?
                /                  \
              No                    Si
              |                     |
    [BUSQUEDA ACTIVA]         [Calcula centroide]
    ultima_pos < 0.5?               |
      Si -> servo=80°               v
      No -> servo=135°    [Normaliza posicion]
              |           pos = cx / ancho_frame
              |                     |
              |                     v
              |       +-----------------------------+
              |       |    CONTROL DIFUSO MAMDANI   |
              |       |    (scikit-fuzzy)            |
              |       |                             |
              |       |  1. Fuzzificacion (7 MFs)   |
              |       |  2. 7 reglas difusas        |
              |       |  3. Inferencia (recorte)    |
              |       |  4. Defuzzificacion         |
              |       |     (centroide)             |
              |       |                             |
              |       |  Entrada: pos [0, 1]        |
              |       |  Salida: angulo [80, 135]   |
              |       +-----------------------------+
              |                     |
              +----------+----------+
                         |
                         v
                  Cambio > 2 grados?
                 /                  \
               No                    Si
            (ignora)          [Publica angulo MQTT]
                              esp32/servo/control
                                    |
                             [Broker retransmite]
                                    |
                              [ESP32 recibe]
                                    |
                          Emergencia activa?
                         /                  \
                       Si                    No
                 (ignora todo)         ledState == true?
                                      /              \
                                    No                Si
                              (ignora todo)     [Escribe servo]
                                                [Actualiza LEDs]
                                                [Motor adelante]

              [Paralelamente cada segundo]
                         |
             +-----------+-----------+
             |           |           |
       [Publica      [Publica    [Estado solo
       bateria %     temp LM35   al cambiar]
       bateria/      LM35/uno]   arranque/paro
       porcentaje]

              [Scada.py recibe todo]
                         |
             +-----------+-----------+
             |           |           |
       [Actualiza    [Grafica    [Registra
       focos y       temp en     evento en
       barra bat]    tiempo      eventos.csv]
                     real]
```

---

## 12. Requisitos para ejecutar el proyecto

### En la Raspberry Pi 4B

```bash
# Instalar broker MQTT
sudo apt install mosquitto mosquitto-clients
sudo systemctl enable mosquitto
sudo systemctl start mosquitto

# Instalar dependencias de Python
pip install opencv-python paho-mqtt numpy scikit-fuzzy scipy networkx reportlab matplotlib

# Ejecutar el control difuso + vision (seguidor de linea)
python3 ControlLinea_skfuzzy.py

# Ejecutar la interfaz SCADA (en otra terminal)
python3 Scada.py
```

### En el ESP32

1. Instalar el **IDE de Arduino** (version 2.x recomendada).
2. Agregar soporte para **ESP32** en el gestor de placas.
3. Instalar librerias: **PubSubClient** (2.8+) y **ESP32Servo**.
4. Abrir `ControlAckermanMQTT/ControlAckermanMQTT.ino`.
5. Seleccionar la placa correcta (ej. *ESP32 Dev Module*).
6. Compilar y cargar al ESP32.

### En PC (solo demo)

```bash
pip install opencv-python numpy scikit-fuzzy scipy networkx
python3 ControlLinea_PC_Demo.py
```

---

*Documentacion actualizada el 2026-05-11*
