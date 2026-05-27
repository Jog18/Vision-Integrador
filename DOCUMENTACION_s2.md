# Vision-Integrador — Documentacion Version s2 (EstacionamientoAmarillo)

> **Estado:** Version Final
> **Version:** s2 — Estaciones de color con parada directa
> **Autor:** Josue (im221466@itsatlixco.edu.mx)
> **Fecha de inicio:** Marzo 2026
> **Ultima actualizacion:** Mayo 2026 (Version Final)

---

## Tabla de Contenidos

1. [Que es el proyecto](#1-que-es-el-proyecto)
2. [Para que sirve](#2-para-que-sirve)
3. [Arquitectura general](#3-arquitectura-general)
4. [Componentes del sistema](#4-componentes-del-sistema)
   - [4.1 Seguidor de Linea con Estaciones — ControlLinea_skfuzzys2.py](#41-seguidor-de-linea-con-estaciones--controllinea_skfuzzys2py)
   - [4.2 Firmware ESP32 — ControlAckermanMQTTs2.ino](#42-firmware-esp32--controlackermanmqtts2ino)
   - [4.3 Interfaz SCADA — Scada2.py](#43-interfaz-scada--scadapy)
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
6. [Maquina de Estados — Estaciones de color](#6-maquina-de-estados--estaciones-de-color)
   - [6.1 Estados](#61-estados)
   - [6.2 Diagrama de transiciones](#62-diagrama-de-transiciones)
   - [6.3 Deteccion de lineas de color](#63-deteccion-de-lineas-de-color)
   - [6.4 Anti-rebote por transicion (debouncing)](#64-anti-rebote-por-transicion-debouncing)
   - [6.5 Zonas y su significado](#65-zonas-y-su-significado)
   - [6.6 Cooldown post-salida](#66-cooldown-post-salida)
7. [Sistema de paro de emergencia (E-Stop)](#7-sistema-de-paro-de-emergencia-e-stop)
8. [Comunicacion MQTT](#8-comunicacion-mqtt)
   - [8.1 Topics MQTT](#81-topics-mqtt)
   - [8.2 Flujo de mensajes por escenario](#82-flujo-de-mensajes-por-escenario)
9. [Interfaz SCADA — Detalle](#9-interfaz-scada--detalle)
   - [9.1 Panel de estado del AGV](#91-panel-de-estado-del-agv)
   - [9.2 Panel de zona del AGV](#92-panel-de-zona-del-agv)
   - [9.3 Botones de control](#93-botones-de-control)
   - [9.4 Barra de bateria](#94-barra-de-bateria)
   - [9.5 Grafica de temperatura](#95-grafica-de-temperatura)
   - [9.6 Alertas](#96-alertas)
10. [Registro de eventos (CSV) y reportes PDF](#10-registro-de-eventos-csv-y-reportes-pdf)
11. [Tecnologias utilizadas](#11-tecnologias-utilizadas)
12. [Configuracion de red y hardware](#12-configuracion-de-red-y-hardware)
    - [12.1 Red y MQTT](#121-red-y-mqtt)
    - [12.2 Pines GPIO del ESP32](#122-pines-gpio-del-esp32)
    - [12.3 Parametros de vision](#123-parametros-de-vision)
13. [Estructura de archivos](#13-estructura-de-archivos)
14. [Flujo de datos completo](#14-flujo-de-datos-completo)
15. [Pista fisica](#15-pista-fisica)
16. [Constantes ajustables](#16-constantes-ajustables)
17. [Integracion con PLC Siemens S7-1200](#17-integracion-con-plc-siemens-s7-1200)
18. [Requisitos para ejecutar el proyecto](#18-requisitos-para-ejecutar-el-proyecto)
19. [Diferencias con la version s1](#19-diferencias-con-la-version-s1)

---

## 1. Que es el proyecto

**Vision-Integrador (version s2)** es un sistema de control para un AGV (Automated Guided Vehicle) a escala 1:10 con direccion **Ackermann** que integra:

- **Vision artificial** en tiempo real (procesada en Raspberry Pi 4B con OpenCV).
- **Control difuso tipo Mamdani** implementado con scikit-fuzzy (skfuzzy), con 7 conjuntos de entrada, 7 de salida y 7 reglas para calcular el angulo del servo de direccion de forma continua y suave.
- **Seguimiento de linea blanca** sobre fondo negro usando Picamera2, con deteccion por umbral + contornos.
- **Deteccion de lineas de color** (Rojo, Azul, Amarillo) en espacio HSV para identificar estaciones en la pista.
- **Parada automatica en estaciones**: al detectar la 3ra linea de un color, el AGV se detiene directamente. Rojo y Azul esperan 5 segundos; Amarillo espera indefinidamente hasta comando externo.
- **Zonas con significado operativo**: Azul = zona de carga, Rojo = zona de descarga, Amarillo = zona de reposo.
- **Recuperacion al perder la linea**: al perder la linea, el servo gira al maximo hacia el lado donde la vio por ultima vez para intentar reencontrarla.
- **Comunicacion inalambrica** mediante protocolo MQTT (broker Mosquitto en Raspberry Pi).
- **Control de hardware** a traves de un microcontrolador ESP32 con servo de direccion y motor DC (TB6612FNG).
- **Monitoreo de bateria** con divisor de voltaje (30k / 7.5k) para bateria Li-ion 12V, publicando porcentaje (0-100%) via MQTT.
- **Lectura de temperatura** con sensor LM35.
- **Sistema de arranque/paro/emergencia** dual (fisico + virtual).
- **Interfaz SCADA** con focos de estado, focos de zona (carga/descarga/reposo/en ruta), grafica de temperatura, barra de nivel de bateria, botones de control y generacion de reportes PDF.
- **Registro de eventos** en archivo CSV con trazabilidad completa.

El sistema detecta una linea blanca con la camara, normaliza su posicion horizontal en el frame [0, 1], y el controlador difuso calcula un angulo de servo continuo [65, 160] grados que se envia al ESP32 via MQTT para dirigir el vehiculo.

---

## 2. Para que sirve

| Aplicacion | Descripcion |
|---|---|
| AGV con direccion Ackermann | Vehiculo que sigue una linea blanca con control difuso de direccion |
| Estaciones de color | Paradas automaticas en zonas de carga (azul), descarga (rojo) y reposo (amarillo) |
| Control difuso aplicado | Implementacion del metodo Mamdani con scikit-fuzzy y defuzzificacion por centroide |
| Automatizacion y SCADA | Monitoreo en tiempo real con interfaz grafica, indicadores de zona, alertas y reportes |
| Educacion | Integracion de vision artificial + logica difusa + IoT + microcontroladores |

---

## 3. Arquitectura general

```
+-----------------------------------------------------------------+
|                      RASPBERRY PI 4B                            |
|                                                                 |
|   +-------------+      +-----------------------------------+   |
|   | Camara CSI  | ---> | ControlLinea_skfuzzys2.py          |   |
|   | Picamera2   |      | - Deteccion de linea blanca       |   |
|   +-------------+      | - Deteccion de lineas de color    |   |
|                         | - Conteo con anti-rebote          |   |
|                         | - Control difuso Mamdani (skfuzzy)|   |
|                         | - Maquina de estados (3 estados)  |   |
|                         | - Calcula angulo servo [65, 160]  |   |
|                         | - Busqueda al perder linea        |   |
|                         | - Publica angulo via MQTT         |   |
|                         | - Publica zona actual del AGV     |   |
|                         | - Envia STOP/GO en estaciones     |   |
|                         +----------------+------------------+   |
|                                          |                      |
|   +--------------------------------------+------------------+   |
|   | Scada2.py (Tkinter)                                      |   |
|   | - 3 focos: PARO / MOVIMIENTO / EMERGENCIA               |   |
|   | - 4 focos zona: CARGA / DESCARGA / REPOSO / EN RUTA    |   |
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
                         |  Recibe: angulo 65-160       |
                         |    (esp32/servo/control)     |
                         |  Recibe: GO, STOP,           |
                         |    EMERGENCIA, RESET_E...    |
                         |    (esp32/arranque)           |
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

### 4.1 Seguidor de Linea con Estaciones — `ControlLinea_skfuzzys2.py`

**Lenguaje:** Python 3
**Dependencias:** `opencv-python`, `numpy`, `scikit-fuzzy`, `scipy`, `networkx`, `paho-mqtt`, `picamera2`
**Ejecutar en:** Raspberry Pi 4B con camara CSI y Mosquitto local

Archivo principal del sistema. Contiene el controlador difuso (via scikit-fuzzy), el bucle de vision para seguir una linea blanca, la deteccion de lineas de color y la maquina de estados para las paradas en estaciones.

#### Que hace

1. **Crea el sistema difuso** con scikit-fuzzy: 7 conjuntos de entrada, 7 de salida, 7 reglas directas, defuzzificacion por centroide. Rango simetrico [65, 160] con centro en 113.
2. **Captura video** desde la camara CSI con Picamera2 a 640x480.
3. **Convierte** cada fotograma a BGR para OpenCV (Picamera2 entrega RGBA o RGB).
4. **Detecta lineas de color** (Rojo, Azul, Amarillo) en una franja horizontal del frame (ROI de color entre 55% y 85% de la altura) usando espacio HSV, morfologia y contornos.
5. **Cuenta las lineas de color** con anti-rebote por transicion de flanco (solo incrementa cuando una linea pasa de no visible a visible).
6. **Al acumular 4 lineas de un color**: detiene el AGV (envia STOP via MQTT), publica la zona actual en `agv/estacion`, y transiciona al estado correspondiente.
7. **Detecta la linea blanca** en el frame completo: escala de grises, umbral binario (200), morfologia (apertura + cierre), contornos, centroide.
8. **Ejecuta el pipeline difuso**: fuzzificacion -> 7 reglas -> defuzzificacion por centroide.
9. **Al perder la linea blanca**: gira el servo al maximo hacia el lado donde la vio por ultima vez (busqueda activa).
10. **Publica el angulo** resultante al ESP32 via MQTT (solo si el cambio es mayor a 2 grados).
11. **Muestra HUD** con: zona central verde, contorno de la linea, centroide, angulo del servo, etiqueta de direccion, contadores de lineas de color, ROIs dibujadas y barra indicadora inferior.

---

### 4.2 Firmware ESP32 — `ControlAckermanMQTTs2.ino`

**Lenguaje:** C++ (Arduino)
**Librerias:** `WiFi.h`, `PubSubClient.h`, `ESP32Servo.h`

Firmware simplificado del microcontrolador. Controla el servo de direccion Ackermann y el motor DC. No tiene logica de estacionamiento propia — toda la logica de paradas es manejada por la Raspberry Pi.

#### Que hace

1. **Conecta al WiFi** y al **broker MQTT**.
2. **Se suscribe** a los topics `esp32/servo/control` y `esp32/arranque`.
3. **Servo de direccion** en GPIO 5 (pulso 500-2400 us):
   - Recibe angulos continuos (65-160) del controlador difuso.
   - Valida rango y escribe al servo.
   - Posicion inicial y por defecto: 113 grados (recto).
4. **Motor DC** via driver TB6612FNG (PWM fijo = 50, 0-255).
5. **LEDs indicadores de bateria** (3 niveles):
   - LED BAT1 (GPIO 22): siempre encendido cuando bat > 0% (nivel bajo).
   - LED BAT2 (GPIO 16): encendido cuando bat >= 40% (nivel medio).
   - LED BAT3 (GPIO 15): encendido cuando bat >= 80% (nivel alto).
6. **Comandos MQTT**:
   - `"GO"`: activa motor hacia adelante y LED de estado.
   - `"STOP"`: detiene todo, centra servo.
   - `"EMERGENCIA"`: paro de emergencia remoto.
   - `"RESET_EMERGENCIA"`: limpia flag de emergencia (solo si el boton fisico no esta presionado).
7. **Sistema de arranque/paro fisico**:
   - **Boton ON** (GPIO 12): enciende el sistema con debounce de 200ms.
   - **Boton OFF** (GPIO 4): apaga el sistema con debounce de 200ms y doble lectura de confirmacion.
   - **E-Stop fisico** (GPIO 13): boton NC con interrupcion de hardware (RISING) y debounce de 300ms en ISR.
8. **Lectura de bateria** cada segundo (GPIO 34): promedio de 20 muestras ADC, divisor de voltaje (factor 5.37), rango 8V-11.3V, calcula porcentaje (0-100%) y publica en `bateria/porcentaje`.
9. **Lectura de temperatura** cada segundo: sensor LM35 (GPIO 36), publica en `LM35/uno`.
10. **Publica estado** (`"MOVIMIENTO"`, `"PARO"`, `"EMERGENCIA"`) solo cuando cambia.
11. **Reconexion automatica** WiFi y MQTT.

---

### 4.3 Interfaz SCADA — `Scada2.py`

**Lenguaje:** Python 3
**Dependencias:** `tkinter`, `paho-mqtt`, `reportlab`, `matplotlib`

Panel de control y monitoreo con tema oscuro.

#### Que tiene

1. **3 focos indicadores de estado del AGV:**
   - Rojo: PARO
   - Verde: MOVIMIENTO
   - Amarillo: EMERGENCIA
   - Solo uno encendido a la vez, con label del estado actual.

2. **4 focos indicadores de zona del AGV:**
   - Azul: CARGA (estacion azul)
   - Rojo: DESCARGA (estacion roja)
   - Amarillo: REPOSO (estacion amarilla)
   - Verde: EN RUTA (circulando)
   - Solo uno encendido a la vez, con label de la zona actual y color asociado.

3. **5 botones de control:**
   - **ARRANQUE** (verde): publica `"GO"` al topic `esp32/arranque`.
   - **PARO** (rojo): publica `"STOP"`.
   - **E-STOP** (amarillo): publica `"EMERGENCIA"`.
   - **RESET E-STOP** (gris): publica `"RESET_EMERGENCIA"`.
   - **GENERAR REPORTE** (azul): genera reporte PDF con grafica de temperatura y tabla de eventos.

4. **Grafica de temperatura en tiempo real:**
   - 60 segundos de historial con linea suavizada.
   - Cuadricula con etiquetas cada 10 C.
   - Linea roja de alerta en 40 C.
   - Punto indicador en el ultimo valor.

5. **Barra de nivel de bateria:**
   - Representacion visual de 0 a 100%.
   - Marca de referencia en 20%.
   - Colores: rojo < 20%, amarillo 20-50%, cyan > 50%.

6. **Sistema de alertas:**
   - Temperatura > 40 C: alerta visual (messagebox) y registro en CSV.
   - Bateria < 20%: registro en CSV.
   - Solo se dispara una vez hasta que el valor vuelva a rango normal.

7. **Generacion de reportes PDF:**
   - Grafica de temperatura exportada como imagen PNG.
   - Tabla completa de eventos del CSV.
   - Conteo de alarmas registradas.

---

## 5. Control Difuso — Detalle completo

El sistema implementa un **controlador difuso tipo Mamdani** usando la libreria scikit-fuzzy (skfuzzy) con rango **simetrico** de salida.

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
                              Angulo del servo [65, 160]
```

### 5.1 Variable de entrada

**Posicion normalizada del objeto en el frame**

- **Universo de discurso:** [0, 1] (1001 puntos)
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

- **Universo de discurso:** [65, 160] grados (96 puntos, resolucion de 1 grado)
- **65 grados**  = giro maximo a la izquierda
- **113 grados** = linea recta (sin giro)
- **160 grados** = giro maximo a la derecha

> El rango mecanico es **simetrico**: 48 grados disponibles a cada lado de 113.
> Los conjuntos difusos de salida estan distribuidos simetricamente con
> espaciado de 16 grados entre centros.

### 5.4 Conjuntos difusos de salida (7)

Todos los conjuntos de salida son **triangulares**:

| # | Conjunto | Parametros [a, b, c] | Rango efectivo | Centro |
|---|----------|---------------------|----------------|--------|
| 1 | **Muy Izquierda**  | [65, 65, 81]    | 65 - 81    | 65  |
| 2 | **Med Izquierda**  | [65, 81, 97]    | 65 - 97    | 81  |
| 3 | **Poco Izquierda** | [81, 97, 113]   | 81 - 113   | 97  |
| 4 | **Centro**         | [97, 113, 129]  | 97 - 129   | 113 |
| 5 | **Poco Derecha**   | [113, 129, 145] | 113 - 145  | 129 |
| 6 | **Med Derecha**    | [129, 145, 160] | 129 - 160  | 145 |
| 7 | **Muy Derecha**    | [145, 160, 160] | 145 - 160  | 160 |

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
   65     81   97    113     129   145     160
```

### 5.5 Funciones de membresia

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
| **R1** | IF posicion = Muy_Izquierda THEN servo = Muy_Izquierda | Linea en extremo izquierdo -> giro maximo izquierdo (65) |
| **R2** | IF posicion = Med_Izquierda THEN servo = Med_Izquierda | Linea a la izquierda -> giro medio izquierdo (81) |
| **R3** | IF posicion = Poco_Izquierda THEN servo = Poco_Izquierda | Linea ligeramente a la izquierda -> correccion leve (97) |
| **R4** | IF posicion = Centro THEN servo = Centro | Linea centrada -> mantener recto (113) |
| **R5** | IF posicion = Poco_Derecha THEN servo = Poco_Derecha | Linea ligeramente a la derecha -> correccion leve (129) |
| **R6** | IF posicion = Med_Derecha THEN servo = Med_Derecha | Linea a la derecha -> giro medio derecho (145) |
| **R7** | IF posicion = Muy_Derecha THEN servo = Muy_Derecha | Linea en extremo derecho -> giro maximo derecho (160) |

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

El metodo de inferencia Mamdani sigue estos pasos:

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

Donde `x_i` recorre el universo discretizado de 65 a 160 (96 puntos con resolucion de 1 grado).

Si el area total es cero (sin activacion), se retorna **113 grados** (recto) como valor por defecto.

### 5.10 Comportamiento al perder la linea

Cuando la camara no detecta la linea (sin contorno valido), el sistema realiza una **busqueda activa** girando el servo al maximo hacia el lado donde vio la linea por ultima vez:

```
ultima_posicion < 0.5  -->  servo = 65   (full izquierda)
ultima_posicion >= 0.5 -->  servo = 160  (full derecha)
```

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
  [65 - 160]                   servo = 65        servo = 160
         |                      (full izq)       (full der)
         v                              \            /
  Guarda ultima_posicion                 v          v
         |                        [Sigue girando hasta
         v                         reencontrar la linea]
  Envia angulo MQTT
```

---

## 6. Maquina de Estados — Estaciones de color

### 6.1 Estados

| Estado | Descripcion | Duracion |
|--------|-------------|----------|
| **SEGUIR_LINEA** | Seguimiento normal de linea blanca + deteccion y conteo de lineas de color | Indefinida |
| **ESPERA_ESTACION** | AGV detenido en estacion Rojo (descarga) o Azul (carga) | 5 segundos |
| **REPOSO_AMARILLO** | AGV detenido en estacion Amarilla (reposo) | Indefinida (hasta GO externo) |

### 6.2 Diagrama de transiciones

```
                          +-------------------+
                          |   SEGUIR_LINEA    |
                          | (control difuso + |
                          |  conteo de color) |
                          +---------+---------+
                                    |
                        3ra linea de un color detectada
                        Pi envia STOP al ESP32
                        Pi publica zona en agv/estacion
                                    |
                  +-----------------+-----------------+
                  |                                   |
           Rojo o Azul                           Amarillo
                  |                                   |
                  v                                   v
      +---------------------+           +---------------------+
      | ESPERA_ESTACION     |           | REPOSO_AMARILLO     |
      | Rojo = DESCARGA     |           | Esperando comando   |
      | Azul = CARGA        |           | externo (SCADA GO   |
      | Timer: 5 segundos   |           | o boton fisico ON)  |
      +----------+----------+           +----------+----------+
                 |                                  |
           Timer cumplido                 ESP32 reporta
           Pi envia GO                    "MOVIMIENTO"
           Pi publica EN_RUTA             Pi publica EN_RUTA
                 |                                  |
                 +----------------+-----------------+
                                  |
                                  v
                          +-------------------+
                          |   SEGUIR_LINEA    |
                          |  (cooldown 3s:    |
                          |   ignora colores) |
                          +-------------------+
```

### 6.3 Deteccion de lineas de color

La deteccion se realiza en una **franja horizontal** (ROI de color) del frame, entre el 55% y el 85% de la altura:

```python
ROI_COLOR_INICIO = 0.55  # inicio de la franja (desde arriba)
ROI_COLOR_FIN = 0.85     # fin de la franja
```

Para cada color (Rojo, Azul, Amarillo):

1. Se aplica una **transformacion afin** (matriz de perspectiva) al ROI para compensar la distorsion trapezoidal de la camara montada a 15cm de altura.
2. Se aplica un **filtro Gaussiano** (kernel 7x7) para reducir ruido y falsos positivos.
3. Se convierte la ROI a espacio de color **HSV**.
4. Se aplica `cv2.inRange()` con los rangos HSV calibrados.
5. Se aplica morfologia (apertura + cierre) con kernel 7x7. Para amarillo se aplica erosion extra con kernel 3x3.
6. Para **Rojo** se combinan dos mascaras (hue 0-10 y 170-180) con `cv2.bitwise_or` ya que el rojo envuelve el espectro HSV.
7. Se buscan contornos y se toma el mayor.
8. Si el area del contorno mayor supera `MIN_AREA_COLOR` (1500 px), se considera una deteccion valida.

#### Rangos HSV por defecto

| Color | H bajo | S bajo | V bajo | H alto | S alto | V alto |
|-------|--------|--------|--------|--------|--------|--------|
| Rojo (bajo) | 0 | 120 | 70 | 10 | 255 | 255 |
| Rojo (alto) | 170 | 120 | 70 | 180 | 255 | 255 |
| Azul | 100 | 150 | 0 | 140 | 255 | 255 |
| Amarillo | 22 | 130 | 130 | 33 | 255 | 255 |

> **IMPORTANTE:** Estos rangos deben calibrarse con `rangosHSV.py` antes de usar en un entorno real.

### 6.4 Anti-rebote por transicion (debouncing)

Para evitar contar multiples veces la misma linea de color mientras el robot pasa sobre ella:

- Se mantiene un flag `linea_visible[color]` por cada color.
- El contador solo se incrementa en la **transicion False -> True** (flanco de subida): la linea acaba de aparecer en el campo de vision.
- Cuando la linea desaparece (True -> False), se resetea el flag pero **no se incrementa** el contador.

```
Frame 1: color NO visible  (linea_visible = False)
Frame 2: color SI visible  (linea_visible = False -> True) --> CONTEO +1
Frame 3: color SI visible  (linea_visible = True)          --> no cuenta
Frame 4: color SI visible  (linea_visible = True)          --> no cuenta
Frame 5: color NO visible  (linea_visible = True -> False) --> resetea flag
Frame 6: color NO visible  (linea_visible = False)
Frame 7: color SI visible  (linea_visible = False -> True) --> CONTEO +1
```

### 6.5 Zonas y su significado

| Color de linea | Zona | Significado operativo | Comportamiento |
|----------------|------|-----------------------|----------------|
| **Azul** | CARGA | Zona de carga de material | Parada 5 segundos, salida automatica |
| **Rojo** | DESCARGA | Zona de descarga de material | Parada 5 segundos, salida automatica |
| **Amarillo** | REPOSO | Zona de reposo / espera | Parada indefinida hasta comando externo |
| — | EN_RUTA | Circulando en la pista | Seguimiento normal de linea |

La zona actual se publica via MQTT en el topic `agv/estacion` y se muestra en la interfaz SCADA con indicadores LED dedicados.

### 6.6 Cooldown post-salida

Al salir de cualquier estacion (cuando el AGV retoma `SEGUIR_LINEA`), se activa un periodo de **cooldown de 3 segundos** durante el cual se **ignora** toda deteccion de color. Esto evita que el robot vuelva a contar la misma linea que acaba de dejar atras.

```python
COOLDOWN_SALIDA = 3.0  # segundos
```

---

## 7. Sistema de paro de emergencia (E-Stop)

El sistema cuenta con paro de emergencia dual (fisico y virtual):

### E-Stop fisico (GPIO 13)

- **Boton normalmente cerrado (NC)**: en operacion normal el circuito esta cerrado (GPIO lee LOW).
- **Al presionar o si el cable se rompe**: el circuito se abre (GPIO lee HIGH) -> **fail-safe**.
- Usa **interrupcion de hardware** (`attachInterrupt`, `RISING`) para respuesta en microsegundos.
- La ISR (`isrEmergencia`) apaga LEDs, detiene motor y activa flag de emergencia inmediatamente.
- **Debounce en ISR**: ignora interrupciones separadas por menos de 300ms.

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
                    |   - Motor detenido      |
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

## 8. Comunicacion MQTT

### 8.1 Topics MQTT

| Topic | Publicador | Suscriptor | Contenido | Direccion |
|-------|-----------|------------|-----------|-----------|
| `esp32/servo/control` | ControlLinea (Pi) | ESP32 | Angulo del servo (65-160, recto=113) | Pi -> ESP32 |
| `esp32/arranque` | Scada2.py / ControlLinea (Pi) | ESP32 | `GO`, `STOP`, `EMERGENCIA`, `RESET_EMERGENCIA` | Pi -> ESP32 |
| `arranque/paro` | ESP32 | Scada2.py / ControlLinea (Pi) | `MOVIMIENTO`, `PARO`, `EMERGENCIA` | ESP32 -> Pi |
| `agv/estacion` | ControlLinea (Pi) | Scada2.py | `CARGA`, `DESCARGA`, `REPOSO`, `EN_RUTA` | Pi -> SCADA |
| `bateria/porcentaje` | ESP32 | Scada2.py | Porcentaje de bateria (0-100%) | ESP32 -> Pi |
| `LM35/uno` | ESP32 | Scada2.py | Temperatura en grados C | ESP32 -> Pi |

### 8.2 Flujo de mensajes por escenario

#### Parada en estacion Azul (carga) o Roja (descarga)

```
Pi (ControlLinea)                  ESP32                     SCADA
      |                              |                         |
      |-- STOP --> esp32/arranque -->|                         |
      |                              |-- PARO --> arranque/paro -->|
      |-- CARGA --> agv/estacion ----|------------------------>|
      |                              |                    [Foco CARGA ON]
      |     (espera 5 segundos)      |                         |
      |                              |                         |
      |-- GO --> esp32/arranque ---->|                         |
      |                              |-- MOVIMIENTO --> ------>|
      |-- EN_RUTA --> agv/estacion --|------------------------>|
      |                              |                    [Foco EN RUTA ON]
```

#### Parada en estacion Amarilla (reposo)

```
Pi (ControlLinea)                  ESP32                     SCADA
      |                              |                         |
      |-- STOP --> esp32/arranque -->|                         |
      |                              |-- PARO --> arranque/paro -->|
      |-- REPOSO --> agv/estacion ---|------------------------>|
      |                              |                    [Foco REPOSO ON]
      |                              |                         |
      |  (espera indefinida...)      |                         |
      |                              |                         |
      |                              |       [Usuario presiona ARRANQUE]
      |                              |<-- GO -- esp32/arranque |
      |                              |                         |
      |                              |-- MOVIMIENTO ---------> |
      |<---- MOVIMIENTO ------------|                         |
      |                              |                         |
      |-- EN_RUTA --> agv/estacion --|------------------------>|
      |                              |                    [Foco EN RUTA ON]
```

---

## 9. Interfaz SCADA — Detalle

### 9.1 Panel de estado del AGV

Tres focos circulares con colores de semaforo:

| Foco | Color encendido | Color apagado | Estado |
|------|----------------|---------------|--------|
| PARO | `#ff3333` (rojo brillante) | `#3d1111` (rojo oscuro) | Sistema detenido |
| MOVIMIENTO | `#33ff33` (verde brillante) | `#113d11` (verde oscuro) | Motor activo |
| EMERGENCIA | `#ffcc00` (amarillo brillante) | `#3d3411` (amarillo oscuro) | Paro de emergencia |

Solo un foco encendido a la vez. El label debajo muestra el texto "Estado: {ESTADO}" con el color correspondiente.

### 9.2 Panel de zona del AGV

Cuatro focos circulares que indican la ubicacion logica del AGV en la pista:

| Foco | Color encendido | Color apagado | Zona | Significado |
|------|----------------|---------------|------|-------------|
| CARGA | `#4488ff` (azul) | `#111133` | Estacion azul | Zona de carga |
| DESCARGA | `#ff4444` (rojo) | `#331111` | Estacion roja | Zona de descarga |
| REPOSO | `#ffcc00` (amarillo) | `#332211` | Estacion amarilla | Zona de reposo |
| EN RUTA | `#44ff44` (verde) | `#113311` | Circulando | En movimiento |

Solo un foco encendido a la vez. El label debajo muestra "Zona: {ZONA} ({Color})" con el color correspondiente.

### 9.3 Botones de control

| Boton | Color | Comando MQTT | Topic | Funcion |
|-------|-------|--------------|-------|---------|
| ARRANQUE | Verde `#1a5c1a` | `GO` | `esp32/arranque` | Inicia movimiento |
| PARO | Rojo `#8c1a1a` | `STOP` | `esp32/arranque` | Detiene el AGV |
| E-STOP | Amarillo `#8c6b00` | `EMERGENCIA` | `esp32/arranque` | Paro de emergencia |
| RESET E-STOP | Gris `#444444` | `RESET_EMERGENCIA` | `esp32/arranque` | Limpia emergencia |
| GENERAR REPORTE | Azul `#004488` | — | — | Genera PDF con eventos |

### 9.4 Barra de bateria

- Rango visual: 0% a 100%.
- Marca de referencia en 20%.
- Colores dinamicos: rojo < 20%, amarillo 20-50%, cyan > 50%.
- Valor numerico en texto grande sobre la barra.

### 9.5 Grafica de temperatura

- Canvas de 420x280 px con fondo oscuro.
- 60 puntos de historial (1 por segundo).
- Cuadricula horizontal cada 10 C.
- Linea roja punteada de alerta en 40 C.
- Linea de datos suavizada con punto indicador en el ultimo valor.
- Rango: 0 C a 80 C.
- Se actualiza cada 500ms.

### 9.6 Alertas

| Condicion | Tipo | Accion |
|-----------|------|--------|
| Temperatura > 40 C | ALERTA_TEMP | Messagebox de advertencia + registro en CSV |
| Bateria < 20% | ALERTA_BAT | Registro en CSV |

Las alertas se disparan una sola vez hasta que el valor regrese al rango normal.

---

## 10. Registro de eventos (CSV) y reportes PDF

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
2026-05-25,10:15:30,MOVIMIENTO,VIRTUAL,24.5,85.0
2026-05-25,10:16:45,PARO,FISICA,25.1,82.3
2026-05-25,10:20:12,ALERTA_TEMP,SENSOR,41.2,78.0
2026-05-25,10:22:05,EMERGENCIA,VIRTUAL,42.0,75.5
2026-05-25,10:23:00,ALERTA_BAT,SENSOR,38.5,15.2
```

### Reportes PDF

El boton **GENERAR REPORTE** en la interfaz SCADA crea un archivo `Reporte_SCADA_YYYY-MM-DD.pdf` que incluye:
- Titulo y fecha.
- Conteo de alarmas registradas.
- Grafica de temperatura (exportada como imagen PNG).
- Tabla completa de eventos del CSV.

---

## 11. Tecnologias utilizadas

| Tecnologia | Rol |
|---|---|
| **Python 3** | Control difuso, vision, interfaz SCADA |
| **OpenCV (`cv2`)** | Captura de video, procesamiento de imagen, deteccion de linea y colores |
| **scikit-fuzzy (`skfuzzy`)** | Sistema de control difuso Mamdani (requiere scipy y networkx) |
| **NumPy** | Arrays para procesamiento de imagen y calculo numerico |
| **Picamera2** | Captura de video desde camara CSI en Raspberry Pi |
| **Tkinter** | Interfaz grafica SCADA |
| **paho-mqtt** | Cliente MQTT para Python |
| **ReportLab** | Generacion de reportes PDF |
| **Matplotlib** | Graficas de temperatura para los reportes |
| **MQTT (protocolo)** | Comunicacion ligera publicador-suscriptor |
| **Mosquitto** | Broker MQTT (en Raspberry Pi) |
| **Raspberry Pi 4B** | Ejecuta vision, control difuso, maquina de estados, interfaz y broker MQTT |
| **ESP32** | Microcontrolador con WiFi: servo, motor, sensores, botones |
| **TB6612FNG** | Driver de motor DC |
| **ESP32Servo** | Libreria para control de servo en ESP32 |
| **PubSubClient** | Libreria MQTT para Arduino/ESP32 |

---

## 12. Configuracion de red y hardware

### 12.1 Red y MQTT

> **Nota:** Estas configuraciones estan hardcodeadas en los archivos fuente.

| Parametro | ControlLinea_skfuzzys2.py | ESP32 | Scada2.py |
|---|---|---|---|
| SSID WiFi | — | `INFINITUM60B6` | — |
| Contrasena WiFi | — | `NUPatFq39h` | — |
| IP del broker MQTT | `127.0.0.1` (localhost) | `192.168.1.68` | `192.168.1.68` |
| Puerto MQTT | `1883` | `1883` | `1883` |

> ControlLinea usa `127.0.0.1` porque corre en la misma Raspberry Pi que el broker Mosquitto.
> El ESP32 y Scada usan la IP de la Raspberry Pi en la red local.

### 12.2 Pines GPIO del ESP32

| Pin GPIO | Nombre en codigo | Funcion |
|---|---|---|
| GPIO 5 | `PIN_SERVO` | Servo de direccion Ackermann (PWM 500-2400 us, 50 Hz) |
| GPIO 22 | `LED_BAT1` | LED indicador bateria nivel bajo (siempre ON si bat > 0%) |
| GPIO 16 | `LED_BAT2` | LED indicador bateria nivel medio (ON si bat >= 40%) |
| GPIO 15 | `LED_BAT3` | LED indicador bateria nivel alto (ON si bat >= 80%) |
| GPIO 2 | `PIN_LED` | LED de estado encendido |
| GPIO 23 | `PIN_LEDPARO` | LED de estado en paro (parpadea en emergencia 300ms) |
| GPIO 12 | `PIN_BOTON_ON` | Boton de encendido (INPUT_PULLUP, debounce 200ms) |
| GPIO 4 | `PIN_BOTON_OFF` | Boton de apagado (INPUT_PULLUP, debounce 200ms, doble lectura) |
| GPIO 13 | `PIN_ESTOP` | Boton E-Stop NC (INPUT_PULLUP, interrupcion RISING, debounce 300ms) |
| GPIO 34 | `pinBateria` | Lectura de bateria via divisor de voltaje factor 5.37 (ADC, 12 bits) |
| GPIO 36 | `pinLM35` | Lectura de sensor LM35 (ADC, 12 bits) |
| GPIO 19 | `AIN1` | Motor DC - direccion A (TB6612FNG) |
| GPIO 18 | `AIN2` | Motor DC - direccion B (TB6612FNG) |
| GPIO 26 | `PWMA` | Motor DC - PWM velocidad (TB6612FNG, 1kHz, 8 bits) |
| — | `STBY` | Motor DC - standby (TB6612FNG, puenteado a 3.3V) |

### 12.3 Parametros de vision

| Parametro | Valor | Descripcion |
|---|---|---|
| Camara | Picamera2 (CSI) | Camara CSI de la Raspberry Pi, 640x480 |
| `UMBRAL_BLANCO` | `200` | Umbral de binarizacion para detectar linea blanca (0-255) |
| `MIN_AREA_LINEA` | `300 px` | Area minima de contorno de linea blanca para filtrar ruido |
| `ROI_PROPORCION` | `1.0` | Fraccion del frame a analizar para linea blanca (100%) |
| `ROI_COLOR_INICIO` | `0.55` | Inicio de la franja de deteccion de color (55% desde arriba) |
| `ROI_COLOR_FIN` | `0.85` | Fin de la franja de deteccion de color (85% desde arriba) |
| `MIN_AREA_COLOR` | `1500 px` | Area minima de contorno de color para considerar deteccion valida |
| Umbral de cambio servo | `2 grados` | Solo envia al ESP32 si el cambio supera este umbral |
| Delay por fotograma | `1 ms` | `cv2.waitKey(1)` — ESC para salir |

---

## 13. Estructura de archivos

```
Vision/
|
|-- ControlLinea_skfuzzys2.py          # Control difuso + vision + estaciones + MQTT
|-- Scada2.py                          # Interfaz SCADA: monitoreo, control, zonas, CSV, PDF
|-- eventos.csv                        # Registro de eventos (generado automaticamente)
|-- DOCUMENTACION_s2.md                # Este archivo
|-- LOGICA_DIFUSA.md                   # Guia completa de la logica difusa utilizada
|-- GUIA_ESTUDIO_JURADO.md            # Guia de estudio para presentacion ante jurado
|-- Guia_PLC_MQTT.md                   # Guia de integracion PLC S7-1200 via Node-RED
|
|-- ControlAckermanMQTTs2/
|   |-- ControlAckermanMQTTs2.ino      # Firmware ESP32: servo + motor + sensores + E-Stop
```

---

## 14. Flujo de datos completo

```
[Camara CSI] --> [Fotograma RGBA/RGB]
                       |
                       v
              [Conversion a BGR]
                       |
         +-------------+-------------+
         |                           |
         v                           v
[ROI Color: 55%-85%]         [Frame completo]
[Conversion a HSV]           [Escala de grises]
[inRange por color]          [Umbral > 200]
[Morfologia 5x5]             [Morfologia 3x3]
[Contornos > 1500px]         [Contornos > 300px]
         |                           |
         v                           v
[Deteccion de color]         [Deteccion de linea]
  Rojo/Azul/Amarillo           posicion_norm
         |                           |
         v                           |
[Anti-rebote + conteo]              |
  contadores por color              |
         |                           |
   3ra linea detectada?              |
    /           \                    |
   Si            No                  |
   |              \                  |
   v               +--------+-------+
[STOP al ESP32]             |
[Publicar zona]             v
[Cambiar estado]     posicion_norm -> [CONTROL DIFUSO MAMDANI]
   |                              |
   |                     Entrada: pos [0, 1]
   |                     Salida: angulo [65, 160]
   |                              |
   |                     Cambio > 2 grados?
   |                    /                  \
   |                  No                    Si
   |               (ignora)         [Publica angulo MQTT]
   |                                 esp32/servo/control
   |                                       |
   v                                       v
[ESPERA o REPOSO]               [ESP32 recibe angulo]
   |                            [Escribe servo]
   |                            [Actualiza LEDs]
   |
   | Timer 5s (Rojo/Azul) o GO externo (Amarillo)
   |
   v
[GO al ESP32]
[Publicar EN_RUTA]
[Cooldown 3s]
[Volver a SEGUIR_LINEA]

              [Paralelamente cada segundo]
                         |
             +-----------+-----------+
             |           |           |
       [Publica      [Publica    [Estado solo
       bateria %     temp LM35   al cambiar]
       bateria/      LM35/uno]   arranque/paro
       porcentaje]

              [Scada2.py recibe todo]
                         |
         +-------+------+------+-------+
         |       |             |       |
   [Actualiza [Grafica    [Registra [Actualiza
    focos     temp en     evento en  focos de
    estado]   tiempo      CSV]       zona]
              real]
```

---

## 15. Pista fisica

| Parametro | Valor |
|---|---|
| Dimensiones | 240 x 240 cm |
| Linea de seguimiento | Blanca sobre fondo negro |
| Estaciones | 50 x 50 cm cada una |
| Marcadores de estacion | Lineas de color perpendiculares a la pista |
| Curvas | Radio 80 cm y 150 cm |
| Margen exterior | 35 cm |
| Colores de estacion | Rojo (descarga), Azul (carga), Amarillo (reposo) |
| Lineas por estacion | 3 lineas de color perpendiculares antes de la zona |

---

## 16. Constantes ajustables

### En ControlLinea_skfuzzys2.py

| Constante | Valor por defecto | Descripcion |
|---|---|---|
| `BROKER` | `"127.0.0.1"` | IP del broker MQTT |
| `PORT` | `1883` | Puerto del broker MQTT |
| `SERVO_MIN` | `65` | Angulo minimo del servo (giro maximo izquierda) |
| `SERVO_MAX` | `160` | Angulo maximo del servo (giro maximo derecha) |
| `SERVO_RECTO` | `113` | Angulo recto (sin giro) |
| `UMBRAL_BLANCO` | `200` | Umbral de binarizacion. Subir si hay falsos positivos |
| `MIN_AREA_LINEA` | `300` | Area minima de contorno de linea blanca (pixeles) |
| `ROI_PROPORCION` | `1.0` | Proporcion del frame para deteccion de linea blanca |
| `ROI_COLOR_INICIO` | `0.55` | Inicio de la franja de deteccion de color |
| `ROI_COLOR_FIN` | `0.85` | Fin de la franja de deteccion de color |
| `MIN_AREA_COLOR` | `1500` | Area minima para deteccion de color valida (pixeles) |
| `LINEAS_PARA_ESTACIONAR` | `3` | Numero de lineas de un color para activar parada |
| `TIEMPO_ESPERA_ESTACION` | `5.0` | Segundos de espera en estaciones Rojo/Azul |
| `COOLDOWN_SALIDA` | `3.0` | Segundos de cooldown post-salida de estacion |
| `RANGOS_HSV` | (ver tabla) | Rangos HSV para cada color. Calibrar con rangosHSV.py |

### En ControlAckermanMQTTs2.ino

| Constante | Valor por defecto | Descripcion |
|---|---|---|
| `ssid` | `"INFINITUM60B6"` | Nombre de la red WiFi |
| `password` | `"NUPatFq39h"` | Contrasena de la red WiFi |
| `mqtt_server` | `"192.168.1.68"` | IP del broker MQTT |
| `SERVO_MIN` | `65` | Angulo minimo del servo |
| `SERVO_MAX` | `160` | Angulo maximo del servo |
| `SERVO_RECTO` | `113` | Angulo recto |
| `VELOCIDAD_FIJA` | `50` | PWM del motor (0-255) |
| `debounceDelay` | `200` | Delay de anti-rebote para botones (ms) |
| `voltajeMaxBat` | `11.3` | Voltaje maximo de la bateria (V) |
| `voltajeMinBat` | `8.0` | Voltaje minimo de la bateria (V) |
| `factorDivisor` | `5.37` | Factor del divisor de voltaje |

---

## 17. Integracion con PLC Siemens S7-1200

El sistema soporta integracion con un PLC Siemens S7-1200 para control industrial del AGV. Como el firmware v1.0 del PLC no soporta MQTT nativo, se utiliza **Node-RED** como puente entre el protocolo S7 del PLC y el broker MQTT (Mosquitto).

### Arquitectura PLC

```
+----------------+     Protocolo S7     +------------------+      MQTT       +-------------+
|  PLC S7-1200   | <=================> |  Raspberry Pi    | <=============> |   ESP32     |
|  (firmware v1) |     Puerto 102      |  - Mosquitto     |   Puerto 1883   |  Ackermann  |
|                |                      |  - Node-RED      |                 |             |
|  DB1 (DB_AGV)  |                      |  node-red-       |                 |  GO / STOP  |
|                |                      |  contrib-s7      |                 |  MOVIMIENTO |
+----------------+                      +------------------+                 +-------------+
```

### Variables del PLC (DB1)

| Variable | Tipo | Direccion | Descripcion | Direccion |
|---|---|---|---|---|
| `boton_arranque` | Bool | DB1.DBX0.0 | PLC envia GO al AGV | PLC -> AGV |
| `boton_paro` | Bool | DB1.DBX0.1 | PLC envia STOP al AGV | PLC -> AGV |
| `boton_emergencia` | Bool | DB1.DBX0.2 | PLC envia EMERGENCIA | PLC -> AGV |
| `boton_reset` | Bool | DB1.DBX0.3 | PLC envia RESET_EMERGENCIA | PLC -> AGV |
| `estado_movimiento` | Bool | DB1.DBX1.0 | AGV en movimiento | AGV -> PLC |
| `estado_paro` | Bool | DB1.DBX1.1 | AGV en paro | AGV -> PLC |
| `estado_emergencia` | Bool | DB1.DBX1.2 | AGV en emergencia | AGV -> PLC |
| `temperatura` | Real | DB1.DBD2 | Temperatura LM35 | AGV -> PLC |
| `bateria` | Real | DB1.DBD6 | Porcentaje bateria | AGV -> PLC |

### Flujo PLC -> AGV

```
Boton fisico PLC (I0.0) -> DB1.DBX0.0 = TRUE (programa LAD)
    -> Node-RED lee via protocolo S7 (puerto 102)
    -> Node-RED publica "GO" en "esp32/arranque" via MQTT
    -> ESP32 recibe -> motor ON
    -> ESP32 publica "MOVIMIENTO" en "arranque/paro"
    -> Node-RED escribe DB1.DBX1.0 = TRUE en el PLC
    -> PLC enciende luz verde en Q0.0
```

### Requisitos del PLC

- **PUT/GET habilitado** en propiedades del PLC (obligatorio para acceso remoto)
- **Acceso optimizado desactivado** en DB1 (para usar direcciones absolutas)
- PLC y Raspberry Pi en la **misma subred**
- Node-RED con plugin `node-red-contrib-s7` instalado

> La guia completa paso a paso esta en `Guia_PLC_MQTT.md`

---

## 18. Requisitos para ejecutar el proyecto

### En la Raspberry Pi 4B

```bash
# Instalar broker MQTT
sudo apt install mosquitto mosquitto-clients
sudo systemctl enable mosquitto
sudo systemctl start mosquitto

# Instalar dependencias de Python
pip install opencv-python paho-mqtt numpy scikit-fuzzy scipy networkx reportlab matplotlib

# Ejecutar el control difuso + vision + estaciones (seguidor de linea)
python3 ControlLinea_skfuzzys2.py

# Ejecutar la interfaz SCADA (en otra terminal)
python3 Scada2.py
```

### En el ESP32

1. Instalar el **IDE de Arduino** (version 2.x recomendada).
2. Agregar soporte para **ESP32** en el gestor de placas.
3. Instalar librerias: **PubSubClient** (2.8+) y **ESP32Servo**.
4. Abrir `ControlAckermanMQTTs2/ControlAckermanMQTTs2.ino`.
5. Ajustar `ssid`, `password` y `mqtt_server` segun la red.
6. Seleccionar la placa correcta (ej. *ESP32 Dev Module*).
7. Compilar y cargar al ESP32.

---

## 19. Diferencias con la version s1

| Aspecto | Version s1 | Version s2 (EstacionamientoAmarillo) |
|---------|-----------|--------------------------------------|
| **Servo rango** | 80-135 (asimetrico, +-33/22 desde 113) | **65-160 (simetrico, +-48 desde 113)** |
| **ROI linea blanca** | 40% inferior del frame | **100% del frame** |
| **Deteccion de color** | No existia | **Rojo, Azul, Amarillo (HSV + contornos)** |
| **Estaciones** | No existian | **3 estaciones con parada directa** |
| **Maniobra reversa** | No existia | **Eliminada (no se usa en s2)** |
| **Zonas operativas** | No existian | **CARGA (azul), DESCARGA (rojo), REPOSO (amarillo)** |
| **Maquina de estados Pi** | No existia | **3 estados: SEGUIR_LINEA, ESPERA_ESTACION, REPOSO_AMARILLO** |
| **Maquina de estados ESP32** | No existia | **Eliminada (el ESP32 solo ejecuta GO/STOP)** |
| **SCADA focos estado** | 3 (PARO, MOVIMIENTO, EMERGENCIA) | **3 (PARO, MOVIMIENTO, EMERGENCIA)** |
| **SCADA focos zona** | No existian | **4 (CARGA, DESCARGA, REPOSO, EN RUTA)** |
| **Topic agv/estacion** | No existia | **Nuevo: publica zona actual del AGV** |
| **Velocidad motor** | PWM 50 | **PWM 50** |
| **Conjuntos difusos salida** | [80,91,102,113,120,127,135] asimetrico | **[65,81,97,113,129,145,160] simetrico** |
| **Anti-rebote color** | No existia | **Debouncing por transicion de flanco** |
| **Cooldown post-estacion** | No existia | **3 segundos ignorando deteccion de color** |
| **Comandos ESP32 parking** | ESTACIONAR, SALIR_REPOSO, GO_REV | **Eliminados (solo GO/STOP)** |

---

*Documentacion Version Final — Actualizada el 2026-05-26*
