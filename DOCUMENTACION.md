# Vision-Integrador — Documentacion del Proyecto

> **Estado:** Prototipo funcional
> **Version:** 4.0
> **Autor:** Josue (im221466@itsatlixco.edu.mx)
> **Fecha de inicio:** Marzo 2026

---

## Tabla de Contenidos

1. [Que es el proyecto](#1-que-es-el-proyecto)
2. [Para que sirve](#2-para-que-sirve)
3. [Arquitectura general](#3-arquitectura-general)
4. [Componentes del sistema](#4-componentes-del-sistema)
   - [4.1 Control Difuso + Vision](#41-control-difuso--vision-controldifusopy)
   - [4.2 Interfaz SCADA](#42-interfaz-scada-scadapy)
   - [4.3 Firmware ESP32 Ackermann](#43-firmware-esp32-ackermann-esp32esp32_ackermannino)
5. [Control Difuso — Detalle completo](#5-control-difuso--detalle-completo)
   - [5.1 Variable de entrada](#51-variable-de-entrada)
   - [5.2 Conjuntos difusos de entrada (7)](#52-conjuntos-difusos-de-entrada-7)
   - [5.3 Variable de salida](#53-variable-de-salida)
   - [5.4 Conjuntos difusos de salida (7)](#54-conjuntos-difusos-de-salida-7)
   - [5.5 Funciones de membresia](#55-funciones-de-membresia)
   - [5.6 Fuzzificacion](#56-fuzzificacion)
   - [5.7 Base de reglas (15 reglas)](#57-base-de-reglas-15-reglas)
   - [5.8 Inferencia Mamdani](#58-inferencia-mamdani)
   - [5.9 Defuzzificacion — Centro de Gravedad](#59-defuzzificacion--centro-de-gravedad)
   - [5.10 Ejemplo de funcionamiento](#510-ejemplo-de-funcionamiento)
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
- **Control difuso tipo Mamdani** con 7 conjuntos de entrada, 7 de salida y 15 reglas para calcular el angulo del servo de direccion de forma continua y suave.
- **Comunicacion inalambrica** mediante protocolo MQTT (broker Mosquitto en Raspberry Pi).
- **Control de hardware** a traves de un microcontrolador ESP32 con servo de direccion.
- **Lectura de sensores** (potenciometro y sensor de temperatura LM35).
- **Sistema de arranque/paro/emergencia** dual (fisico + virtual).
- **Interfaz SCADA** con focos de estado, grafica de temperatura, barra de potenciometro, botones de control y generacion de reportes PDF.
- **Registro de eventos** en archivo CSV con trazabilidad completa.

El sistema detecta un objeto azul con la camara, normaliza su posicion horizontal en el frame [0, 1], y el controlador difuso calcula un angulo de servo continuo [0, 180] grados que se envia al ESP32 via MQTT para dirigir el vehiculo.

---

## 2. Para que sirve

| Aplicacion | Descripcion |
|---|---|
| AGV con direccion Ackermann | Vehiculo que sigue un objeto azul con control difuso de direccion |
| Control difuso aplicado | Implementacion completa del metodo Mamdani con defuzzificacion por centroide |
| Automatizacion y SCADA | Monitoreo en tiempo real con interfaz grafica, alertas y reportes |
| Educacion | Integracion de vision artificial + logica difusa + IoT + microcontroladores |

---

## 3. Arquitectura general

```
+-----------------------------------------------------------------+
|                      RASPBERRY PI 4B                            |
|                                                                 |
|   +-------------+      +-----------------------------------+   |
|   | Camara USB  | ---> | ControlDifuso.py                  |   |
|   | /dev/video0 |      | - Deteccion de objeto azul (HSV)  |   |
|   +-------------+      | - Normalizacion de posicion [0,1]  |   |
|                         | - Control difuso Mamdani (7 MFs)  |   |
|                         | - Calcula angulo servo [0, 180]   |   |
|                         | - Publica angulo via MQTT         |   |
|                         +----------------+------------------+   |
|                                          |                      |
|   +--------------------------------------+------------------+   |
|   | Scada.py (Tkinter)                                      |   |
|   | - 3 focos: PARO / MOVIMIENTO / EMERGENCIA               |   |
|   | - Botones: ARRANQUE / PARO / E-STOP / RESET / REPORTE  |   |
|   | - Grafica de temperatura en tiempo real (60s)           |   |
|   | - Barra de potenciometro (0-3.3V)                       |   |
|   | - Alertas: temp > 40C, voltaje < 0.5V                   |   |
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
                         |    voltaje, estado            |
                         |                              |
                         |  GPIO 5  -- Servo direccion  |
                         |  GPIO 16 -- LED Izquierdo    |
                         |  GPIO 15 -- LED Derecho      |
                         |  GPIO 2  -- LED Estado (ON)  |
                         |  GPIO 23 -- LED Estado (PARO)|
                         |  GPIO 18 -- Boton Encender   |
                         |  GPIO 4  -- Boton Apagar     |
                         |  GPIO 13 -- E-Stop (NC)      |
                         |  GPIO 34 -- Potenciometro    |
                         |  GPIO 36 -- Sensor LM35      |
                         +------------------------------+
```

---

## 4. Componentes del sistema

### 4.1 Control Difuso + Vision — `ControlDifuso.py`

**Lenguaje:** Python 3
**Dependencias:** `opencv-python`, `paho-mqtt`, `numpy`

Este archivo contiene tanto el controlador difuso como el bucle de vision. Es el componente central del sistema.

#### Que hace

1. **Captura video** desde la camara conectada.
2. **Convierte** cada fotograma de BGR a HSV.
3. **Aplica mascara** de color para aislar pixeles azules (H: 100-140, S: 150-255, V: 0-255).
4. **Detecta contornos** y descarta los de area menor a 500 px.
5. **Calcula el centroide** del objeto azul.
6. **Normaliza la posicion** horizontal al rango [0, 1]: `posicion_norm = centro_x / ancho_frame`.
7. **Ejecuta el pipeline difuso completo**: fuzzificacion (7 conjuntos) -> evaluacion de 15 reglas -> defuzzificacion por centroide.
8. **Publica el angulo** resultante al ESP32 via MQTT (solo si el cambio es mayor a 2 grados).
9. **Muestra HUD** con: zona central verde, deteccion del objeto, angulo del servo, etiqueta de direccion y barra indicadora inferior.

#### Clase `ControlDifuso`

La clase encapsula todo el sistema difuso Mamdani:
- `__init__()`: define los 7 conjuntos de entrada, 7 de salida y pre-calcula las MFs.
- `fuzzificar(pos)`: evalua la posicion contra los 7 conjuntos de entrada.
- `evaluar_reglas(...)`: aplica las 15 reglas y calcula activaciones por conjunto de salida.
- `defuzzificar(activaciones)`: recorta, agrega y calcula centroide.
- `calcular(posicion_norm)`: ejecuta el pipeline completo de entrada a salida.

> Ver seccion 5 para el detalle completo del control difuso.

---

### 4.2 Interfaz SCADA — `Scada.py`

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

4. **Barra de nivel del potenciometro:**
   - Representacion visual de 0 a 3.3V.
   - Marca de referencia en 0.5V.
   - Cambia a rojo cuando el voltaje baja de 0.5V.

5. **Sistema de alertas:**
   - Temperatura > 40 C: alerta de temperatura critica.
   - Potenciometro < 0.5V: alerta de voltaje bajo.
   - Solo se muestra una vez hasta que el valor vuelva a rango normal.
   - Las alertas se registran en el CSV.

6. **Generacion de reportes PDF:**
   - Grafica de temperatura exportada como imagen.
   - Tabla con todos los eventos registrados.
   - Conteo de alarmas.

---

### 4.3 Firmware ESP32 Ackermann — `esp32/esp32_ackermann.ino`

**Lenguaje:** C++ (Arduino)
**Librerias:** `WiFi.h`, `PubSubClient.h`, `ESP32Servo.h`

Firmware del microcontrolador que controla el servo de direccion Ackermann.

#### Que hace

1. **Conecta al WiFi** y al **broker MQTT**.
2. **Se suscribe** a los topics `esp32/servo/control` y `esp32/arranque`.
3. **Servo de direccion** en GPIO 5 (pulso 500-2400 us):
   - Recibe angulos continuos (80-135) del controlador difuso.
   - Valida rango mecanico del Ackermann y escribe al servo.
   - Posicion inicial y por defecto: 113 grados (recto).
4. **LEDs indicadores de direccion** (zona muerta +-3 grados alrededor de 113):
   - Angulo < 110: LED izquierdo encendido (GPIO 16).
   - Angulo > 116: LED derecho encendido (GPIO 15).
   - 110-116: ambos LEDs encendidos (recto).
5. **Sistema de arranque/paro** con tres modos:
   - **Botones fisicos**: ON (GPIO 18) y OFF (GPIO 4) con debounce de 50ms.
   - **Comandos MQTT**: `"GO"`, `"STOP"`, `"EMERGENCIA"`, `"RESET_EMERGENCIA"`.
   - **E-Stop fisico**: Boton NC en GPIO 13 con interrupcion de hardware.
6. **Lectura de sensores** cada segundo: potenciometro (GPIO 34) y LM35 (GPIO 36).
7. **Publica estado** (`"MOVIMIENTO"`, `"PARO"`, `"EMERGENCIA"`) solo cuando cambia.
8. **Reconexion automatica** WiFi y MQTT.

> **Nota:** El archivo `esp32/esp32.ino` contiene el firmware legacy del control diferencial (dos motores DC) y no se usa en la configuracion actual.

---

## 5. Control Difuso — Detalle completo

El sistema implementa un **controlador difuso tipo Mamdani** completo con las siguientes etapas:

```
Posicion normalizada [0,1] --> Fuzzificacion (7 MFs)
                                      |
                                      v
                              Evaluacion de 15 reglas
                                      |
                                      v
                              Inferencia (recorte + agregacion)
                                      |
                                      v
                              Defuzzificacion (centroide)
                                      |
                                      v
                              Angulo del servo [0, 180]
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

### 5.7 Base de reglas (15 reglas)

Las reglas utilizan los operadores:
- **AND** -> operador minimo: `min(uA, uB)`
- **NOT** -> complemento: `1 - uA`
- **OR** (agregacion de reglas con mismo consecuente) -> operador maximo: `max(ri, rj)`

#### Tabla completa de reglas

| # | Tipo | Antecedente | Consecuente | Proposito |
|---|------|------------|-------------|-----------|
| **R1** | Directa | IF Muy_Izquierda | Muy_Izquierda | Objeto en extremo izquierdo -> giro maximo izquierdo |
| **R2** | Directa | IF Med_Izquierda | Med_Izquierda | Objeto a la izquierda -> giro medio izquierdo |
| **R3** | Directa | IF Poco_Izquierda | Poco_Izquierda | Objeto ligeramente a la izquierda -> correccion leve |
| **R4** | Directa | IF Centro | Centro | Objeto centrado -> mantener recto |
| **R5** | Directa | IF Poco_Derecha | Poco_Derecha | Objeto ligeramente a la derecha -> correccion leve |
| **R6** | Directa | IF Med_Derecha | Med_Derecha | Objeto a la derecha -> giro medio derecho |
| **R7** | Directa | IF Muy_Derecha | Muy_Derecha | Objeto en extremo derecho -> giro maximo derecho |
| **R8** | Refuerzo | IF Muy_Izq AND NOT Med_Izq | Muy_Izquierda | Refuerza giro fuerte cuando el objeto esta muy lejos a la izquierda |
| **R9** | Refuerzo | IF Muy_Der AND NOT Med_Der | Muy_Derecha | Refuerza giro fuerte cuando el objeto esta muy lejos a la derecha |
| **R10** | Refuerzo | IF Med_Izq AND NOT Poco_Izq | Med_Izquierda | Refuerza giro medio cuando no hay influencia del centro |
| **R11** | Refuerzo | IF Med_Der AND NOT Poco_Der | Med_Derecha | Refuerza giro medio cuando no hay influencia del centro |
| **R12** | Transicion | IF Poco_Izq AND Centro | Centro | Suaviza la transicion izquierda-centro |
| **R13** | Transicion | IF Poco_Der AND Centro | Centro | Suaviza la transicion derecha-centro |
| **R14** | Transicion | IF Centro AND NOT Poco_Izq AND NOT Poco_Der | Centro | Centro puro, sin ambiguedad lateral |
| **R15** | Seguridad | IF NOT ninguno significativo | Centro | Sin deteccion clara -> mantener recto |

#### Calculo de activacion de cada regla

```
r1  = mu_muy_izq                              # -> Muy_Izquierda
r2  = mu_med_izq                              # -> Med_Izquierda
r3  = mu_poco_izq                             # -> Poco_Izquierda
r4  = mu_cen                                  # -> Centro
r5  = mu_poco_der                             # -> Poco_Derecha
r6  = mu_med_der                              # -> Med_Derecha
r7  = mu_muy_der                              # -> Muy_Derecha

r8  = min(mu_muy_izq, 1 - mu_med_izq)        # -> Muy_Izquierda
r9  = min(mu_muy_der, 1 - mu_med_der)        # -> Muy_Derecha
r10 = min(mu_med_izq, 1 - mu_poco_izq)       # -> Med_Izquierda
r11 = min(mu_med_der, 1 - mu_poco_der)       # -> Med_Derecha

r12 = min(mu_poco_izq, mu_cen)               # -> Centro
r13 = min(mu_poco_der, mu_cen)               # -> Centro
r14 = min(mu_cen, 1 - mu_poco_izq,
          1 - mu_poco_der)                    # -> Centro

r15 = min(1 - mu_muy_izq, 1 - mu_med_izq,
          1 - mu_poco_izq, 1 - mu_cen,
          1 - mu_poco_der, 1 - mu_med_der,
          1 - mu_muy_der)                     # -> Centro
```

#### Agregacion por conjunto de salida

Cuando multiples reglas apuntan al mismo conjunto de salida, se usa OR (maximo):

```
Muy_Izquierda  = max(r1, r8)
Med_Izquierda  = max(r2, r10)
Poco_Izquierda = r3
Centro         = max(r4, r12, r13, r14, r15)
Poco_Derecha   = r5
Med_Derecha    = max(r6, r11)
Muy_Derecha    = max(r7, r9)
```

#### Logica detras de las reglas

- **R1-R7 (directas):** Cada conjunto de entrada mapea directamente a su conjunto de salida correspondiente. Proporcionan la respuesta proporcional base del sistema.
- **R8-R9 (refuerzo extremo):** Usan NOT del vecino hacia el centro para activarse fuertemente solo cuando el objeto esta en el borde extremo del frame, produciendo los giros mas agresivos.
- **R10-R11 (refuerzo medio):** Usan NOT del vecino hacia el centro para reforzar el giro medio cuando no hay ambiguedad con la zona central.
- **R12-R13 (transicion suave):** AND entre un conjunto lateral cercano y Centro genera correcciones suaves en la zona de transicion, evitando cambios bruscos.
- **R14 (centro puro):** Refuerza la posicion recta cuando el objeto esta claramente centrado sin ambiguedad lateral.
- **R15 (seguridad):** Garantiza que si ningun conjunto tiene activacion significativa, el servo se mantiene recto (113 grados).

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

### 5.10 Ejemplo de funcionamiento

#### Escenario: Objeto detectado en posicion normalizada = 0.25

**Paso 1 — Fuzzificacion:**

```
mu_muy_izq  = trapmf(0.25, [0.0, 0.0, 0.05, 0.20])  = 0.00
mu_med_izq  = trimf(0.25, [0.05, 0.20, 0.35])        = 0.67
mu_poco_izq = trimf(0.25, [0.20, 0.35, 0.50])         = 0.33
mu_cen      = trimf(0.25, [0.35, 0.50, 0.65])         = 0.00
mu_poco_der = trimf(0.25, [0.50, 0.65, 0.80])         = 0.00
mu_med_der  = trimf(0.25, [0.65, 0.80, 0.95])         = 0.00
mu_muy_der  = trapmf(0.25, [0.80, 0.95, 1.0, 1.0])   = 0.00
```

**Paso 2 — Activacion de reglas:**

```
r1  = 0.00                              -> Muy_Izquierda
r2  = 0.67                              -> Med_Izquierda
r3  = 0.33                              -> Poco_Izquierda
r4  = 0.00                              -> Centro
r5  = 0.00                              -> Poco_Derecha
r6  = 0.00                              -> Med_Derecha
r7  = 0.00                              -> Muy_Derecha
r8  = min(0.00, 1 - 0.67) = 0.00       -> Muy_Izquierda
r9  = min(0.00, 1 - 0.00) = 0.00       -> Muy_Derecha
r10 = min(0.67, 1 - 0.33) = 0.67       -> Med_Izquierda
r11 = min(0.00, 1 - 0.00) = 0.00       -> Med_Derecha
r12 = min(0.33, 0.00) = 0.00           -> Centro
r13 = min(0.00, 0.00) = 0.00           -> Centro
r14 = min(0.00, 1 - 0.33, 1 - 0.00) = 0.00  -> Centro
r15 = min(1.00, 0.33, 0.67, 1.00, 1.00, 1.00, 1.00) = 0.33  -> Centro
```

**Paso 3 — Agregacion por salida:**

```
Muy_Izquierda  = max(0.00, 0.00) = 0.00
Med_Izquierda  = max(0.67, 0.67) = 0.67
Poco_Izquierda = 0.33
Centro         = max(0.00, 0.00, 0.00, 0.00, 0.33) = 0.33
Poco_Derecha   = 0.00
Med_Derecha    = max(0.00, 0.00) = 0.00
Muy_Derecha    = max(0.00, 0.00) = 0.00
```

**Paso 4 — Defuzzificacion (centroide):**

El area resultante se concentra entre Med_Izquierda (pico en 40), Poco_Izquierda (pico en 67) y Centro (pico en 90), con Med_Izquierda dominando. El centroide resulta aproximadamente en **~58 grados**, indicando un giro moderado a la izquierda.

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
                    |   - Servo centrado 90   |
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
| `arranque/paro` | Tipo: `MOVIMIENTO`, `PARO`, `EMERGENCIA`, `ALERTA_TEMP`, `ALERTA_POT` |
| `Origen` | Fuente: `VIRTUAL` (interfaz), `FISICA` (botones ESP32), `SENSOR` (alertas) |
| `LM35/uno` | Temperatura al momento del evento (C) |
| `pot/uno` | Voltaje del potenciometro al momento del evento (V) |

### Ejemplo

```csv
Fecha,Hora,arranque/paro,Origen,LM35/uno,pot/uno
2026-03-16,10:15:30,MOVIMIENTO,VIRTUAL,24.5,1.650
2026-03-16,10:16:45,PARO,FISICA,25.1,1.700
2026-03-16,10:20:12,ALERTA_TEMP,SENSOR,41.2,1.500
2026-03-16,10:22:05,EMERGENCIA,VIRTUAL,42.0,1.450
2026-03-16,10:23:00,ALERTA_POT,SENSOR,38.5,0.350
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
| **OpenCV (`cv2`)** | Captura de video, procesamiento de imagen, deteccion de color |
| **NumPy** | Calculo de funciones de membresia, arrays para mascara y defuzzificacion |
| **Tkinter** | Interfaz grafica SCADA |
| **paho-mqtt** | Cliente MQTT para Python |
| **ReportLab** | Generacion de reportes PDF |
| **Matplotlib** | Graficas de temperatura para los reportes |
| **MQTT (protocolo)** | Comunicacion ligera publicador-suscriptor |
| **Mosquitto** | Broker MQTT (en Raspberry Pi) |
| **Raspberry Pi 4B** | Ejecuta vision, control difuso, interfaz y broker MQTT |
| **ESP32** | Microcontrolador con WiFi: servo, sensores, botones |
| **ESP32Servo** | Libreria para control de servo en ESP32 |
| **PubSubClient** | Libreria MQTT para Arduino/ESP32 |

---

## 9. Configuracion de red y hardware

### 9.1 Red y MQTT

> **Nota:** Estas configuraciones estan hardcodeadas en los archivos fuente.

| Parametro | Valor en ControlDifuso.py | Valor en ESP32 | Valor en Scada.py |
|---|---|---|---|
| SSID WiFi | — | `JOSUE's Galaxy A52` | — |
| Contrasena WiFi | — | `jog18030` | — |
| IP del broker MQTT | `10.91.115.191` | `10.20.185.191` | `10.0.0.5` |
| Puerto MQTT | `1883` | `1883` | `1883` |

**Topics MQTT:**

| Topic | Publicador | Suscriptor | Contenido |
|---|---|---|---|
| `esp32/servo/control` | ControlDifuso.py | ESP32 | Angulo del servo (80-135, recto=113) |
| `esp32/arranque` | Scada.py | ESP32 | GO, STOP, EMERGENCIA, RESET_EMERGENCIA |
| `arranque/paro` | ESP32 | Scada.py | MOVIMIENTO, PARO, EMERGENCIA |
| `pot/uno` | ESP32 | Scada.py | Voltaje del potenciometro (0-3.3V) |
| `LM35/uno` | ESP32 | Scada.py | Temperatura en grados C |

### 9.2 Pines GPIO del ESP32

| Pin GPIO | Nombre en codigo | Funcion |
|---|---|---|
| GPIO 5 | `PIN_SERVO` | Servo de direccion Ackermann (PWM 500-2400 us) |
| GPIO 16 | `ledizq` | LED indicador de giro izquierdo |
| GPIO 15 | `ledder` | LED indicador de giro derecho |
| GPIO 2 | `PIN_LED` | LED de estado encendido |
| GPIO 23 | `PIN_LEDPARO` | LED de estado en paro |
| GPIO 18 | `PIN_BOTON_ON` | Boton de encendido (INPUT_PULLUP) |
| GPIO 4 | `PIN_BOTON_OFF` | Boton de apagado (INPUT_PULLUP) |
| GPIO 13 | `PIN_ESTOP` | Boton E-Stop NC (INPUT_PULLUP, interrupcion RISING) |
| GPIO 34 | `pinPot1` | Lectura de potenciometro (ADC, 12 bits) |
| GPIO 36 | `pinLM35` | Lectura de sensor LM35 (ADC, 12 bits) |

### 9.3 Parametros de vision

| Parametro | Valor | Descripcion |
|---|---|---|
| Indice de camara | `0` | Primera camara detectada por el sistema |
| Rango HSV minimo | `[100, 150, 0]` | Limite inferior del azul |
| Rango HSV maximo | `[140, 255, 255]` | Limite superior del azul |
| Area minima del objeto | `500 px` | Filtra ruido y objetos pequenos |
| Umbral de cambio servo | `2 grados` | Solo envia al ESP32 si el cambio supera este umbral |
| Delay por fotograma | `1 ms` | `cv2.waitKey(1)` — ESC para salir |

---

## 10. Estructura de archivos

```
Vision/
|
|-- ControlDifuso.py               # Control difuso Mamdani + vision + MQTT
|-- Scada.py                       # Interfaz SCADA: monitoreo, control, CSV, reportes PDF
|-- eventos.csv                    # Registro de eventos (generado automaticamente)
|
|-- esp32/
|   |-- esp32_ackermann.ino        # Firmware actual: servo Ackermann + sensores + E-Stop
|   |-- esp32.ino                  # Firmware legacy: control diferencial (no usado)
|
|-- DOCUMENTACION.md               # Este archivo
|-- ControlDifuso_Documentacion.md # Documentacion detallada del control difuso
```

---

## 11. Flujo de datos

```
[Camara USB] --> [Fotograma BGR]
                       |
                       v
                [Conversion HSV]
                       |
                       v
             [Mascara de color azul]
             [H:100-140, S:150-255, V:0-255]
                       |
                       v
             [Deteccion de contornos]
                       |
                 Hay objeto > 500px?
                /                  \
              No                    Si
              |                     |
        (servo = 90)          [Calcula centroide]
                                    |
                                    v
                          [Normaliza posicion]
                          pos = cx / ancho_frame
                                    |
                                    v
                      +-----------------------------+
                      |    CONTROL DIFUSO MAMDANI   |
                      |                             |
                      |  1. Fuzzificacion (7 MFs)   |
                      |  2. 15 reglas difusas       |
                      |  3. Inferencia (recorte)    |
                      |  4. Defuzzificacion         |
                      |     (centroide)             |
                      |                             |
                      |  Entrada: pos [0, 1]        |
                      |  Salida: angulo [0, 180]    |
                      +-----------------------------+
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

              [Paralelamente cada segundo]
                         |
             +-----------+-----------+
             |           |           |
       [Publica      [Publica    [Estado solo
       voltaje       temp LM35   al cambiar]
       pot/uno]      LM35/uno]   arranque/paro

              [Scada.py recibe todo]
                         |
             +-----------+-----------+
             |           |           |
       [Actualiza    [Grafica    [Registra
       focos y       temp en     evento en
       barra pot]    tiempo      eventos.csv]
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
pip install opencv-python paho-mqtt numpy reportlab matplotlib

# Ejecutar el control difuso + vision
python3 ControlDifuso.py

# Ejecutar la interfaz SCADA (en otra terminal)
python3 Scada.py
```

### En el ESP32

1. Instalar el **IDE de Arduino** (version 2.x recomendada).
2. Agregar soporte para **ESP32** en el gestor de placas.
3. Instalar librerias: **PubSubClient** (2.8+) y **ESP32Servo**.
4. Abrir `esp32/esp32_ackermann.ino`.
5. Seleccionar la placa correcta (ej. *ESP32 Dev Module*).
6. Compilar y cargar al ESP32.

---

*Documentacion actualizada el 2026-03-24*
