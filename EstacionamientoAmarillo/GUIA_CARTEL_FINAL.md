# Guia de Contenido para Cartel Final — Vision-Integrador AGV

> **Instrucciones:** Copia el texto de cada seccion directamente en la caja correspondiente
> de tu plantilla de cartel. Los textos ya estan redactados en tono tecnico y sintetico.
> Las notas marcadas con `[VISUAL]` indican que debes insertar una imagen, diagrama o foto.
> Las notas marcadas con `[NOTA]` son indicaciones para ti, no van en el cartel.
> Los diagramas ASCII de esta guia son para que los redibuje en tu herramienta de diseno
> (PowerPoint, Canva, draw.io, etc.) con cajas, flechas y colores.

---

## 1. TITULO DEL PROYECTO

Elige una de las tres opciones:

**Opcion A — Formal (para cartel academico):**

```
Sistema de Control Difuso y Vision Artificial para la
Navegacion Autonoma de un AGV con Direccion Ackermann
y Supervision SCADA
```

**Opcion B — Tecnica/Descriptiva:**

```
AGV Ackermann con Seguimiento de Linea por Control Difuso
Mamdani, Deteccion de Estaciones por Color e Interfaz
SCADA en Tiempo Real
```

**Opcion C — Corta de alto impacto (recomendada para cartel):**

```
AGV Inteligente: Vision Artificial, Control Difuso
y SCADA para Navegacion Autonoma
```

**Subtitulo (opcional, debajo del titulo):**

```
Integracion de OpenCV, Logica Difusa Mamdani, MQTT e Interfaz SCADA
sobre Raspberry Pi 4B y ESP32
```

---

## 2. INTRODUCCION Y PROBLEMATICA

```
- La navegacion autonoma de vehiculos guiados (AGVs) en entornos
  industriales y educativos requiere sistemas capaces de seguir
  trayectorias con precision, detenerse en estaciones operativas
  y reportar su estado en tiempo real.

- Los controladores clasicos (ON/OFF) generan oscilaciones bruscas
  en la direccion de vehiculos con geometria Ackermann, afectando
  la estabilidad del seguimiento en curvas de radio variable.

- La falta de integracion entre navegacion, supervision remota y
  mecanismos de seguridad dificulta la operacion confiable de AGVs
  a escala en aplicaciones reales.
```

---

## 3. OBJETIVO PRINCIPAL

```
Desarrollar un sistema integrado de navegacion autonoma para un AGV
a escala 1:10 con direccion Ackermann, utilizando vision artificial
(OpenCV) y control difuso tipo Mamdani (scikit-fuzzy) para el
seguimiento de linea, deteccion de estaciones por color en espacio
HSV, y supervision remota mediante interfaz SCADA con comunicacion
MQTT.
```

---

## 4. METODOLOGIA Y DESARROLLO TECNICO

### 4.1 Hardware y Electronica

```
- Chasis Ackermann escala 1:10
- Pista: 240 x 240 cm, linea blanca sobre fondo negro,
  estaciones de color de 50 x 50 cm
- Procesamiento: Raspberry Pi 4B
  (vision, control difuso, broker MQTT, SCADA)
- Actuacion: ESP32 + servo de direccion (PWM 500-2400 us)
  + motor DC via driver TB6612FNG
- Sensores: Camara CSI Picamera2 (640x480),
  LM35 (temperatura), divisor de voltaje 30k/7.5k
  (bateria Li-ion 12V)
- Seguridad: Boton E-Stop normalmente cerrado con
  interrupcion de hardware (fail-safe)
```

```
[VISUAL] Foto del AGV armado con etiquetas senalando:
         camara, ESP32, servo, motor, bateria, E-Stop, LEDs
```

---

### 4.2 Vision Artificial y Control Difuso

```
PIPELINE DE VISION:
  Camara CSI (640x480)
    -> Escala de grises
    -> ROI configurable
    -> Umbral binario (>200 = blanco)
    -> Morfologia (apertura + cierre)
    -> Deteccion de contornos
    -> Centroide del contorno mayor
    -> Posicion normalizada [0, 1]
    -> Controlador difuso Mamdani
    -> Angulo del servo [65, 160]
    -> Publicacion MQTT al ESP32
```

```
CONTROLADOR DIFUSO MAMDANI (scikit-fuzzy):

  ENTRADA: posicion normalizada [0, 1]
  7 conjuntos difusos:
    Muy Izq | Med Izq | Poco Izq | Centro |
    Poco Der | Med Der | Muy Der
  (trapezoidales en extremos, triangulares en centro)

  SALIDA: angulo del servo [65 - 160]
  7 conjuntos difusos triangulares
  Centro = 113 (recto), +-48 grados simetricos

  7 REGLAS IF-THEN directas:
    IF pos = Muy_Izq   THEN servo = Muy_Izq   (65)
    IF pos = Med_Izq   THEN servo = Med_Izq   (81)
    IF pos = Poco_Izq  THEN servo = Poco_Izq  (97)
    IF pos = Centro    THEN servo = Centro     (113)
    IF pos = Poco_Der  THEN servo = Poco_Der   (129)
    IF pos = Med_Der   THEN servo = Med_Der    (145)
    IF pos = Muy_Der   THEN servo = Muy_Der    (160)

  DEFUZZIFICACION: Centroide (centro de gravedad)

  BUSQUEDA ACTIVA: al perder la linea, gira full
  hacia el ultimo lado conocido para reencontrarla.
```

```
[VISUAL] Grafica de funciones de membresia de ENTRADA
         (7 conjuntos sobre [0, 1])

[VISUAL] Grafica de funciones de membresia de SALIDA
         (7 conjuntos sobre [65, 160])
```

> **[NOTA] Para generar las graficas de membresia** ejecuta este snippet en Python:
>
> ```python
> import numpy as np
> import skfuzzy as fuzz
> import matplotlib.pyplot as plt
>
> # ENTRADA
> x = np.linspace(0, 1, 1001)
> plt.figure(figsize=(8, 3))
> plt.plot(x, fuzz.trapmf(x, [0, 0, 0.05, 0.20]), label='Muy Izq')
> plt.plot(x, fuzz.trimf(x, [0.05, 0.20, 0.35]), label='Med Izq')
> plt.plot(x, fuzz.trimf(x, [0.20, 0.35, 0.50]), label='Poco Izq')
> plt.plot(x, fuzz.trimf(x, [0.35, 0.50, 0.65]), label='Centro')
> plt.plot(x, fuzz.trimf(x, [0.50, 0.65, 0.80]), label='Poco Der')
> plt.plot(x, fuzz.trimf(x, [0.65, 0.80, 0.95]), label='Med Der')
> plt.plot(x, fuzz.trapmf(x, [0.80, 0.95, 1.0, 1.0]), label='Muy Der')
> plt.title('Funciones de Membresia - Entrada (Posicion)')
> plt.xlabel('Posicion normalizada')
> plt.ylabel('Grado de pertenencia')
> plt.legend(loc='upper right', fontsize=7)
> plt.tight_layout()
> plt.savefig('membresia_entrada.png', dpi=200)
> plt.show()
>
> # SALIDA
> y = np.linspace(65, 160, 96)
> plt.figure(figsize=(8, 3))
> plt.plot(y, fuzz.trimf(y, [65, 65, 81]),   label='Muy Izq')
> plt.plot(y, fuzz.trimf(y, [65, 81, 97]),   label='Med Izq')
> plt.plot(y, fuzz.trimf(y, [81, 97, 113]),  label='Poco Izq')
> plt.plot(y, fuzz.trimf(y, [97, 113, 129]), label='Centro')
> plt.plot(y, fuzz.trimf(y, [113, 129, 145]),label='Poco Der')
> plt.plot(y, fuzz.trimf(y, [129, 145, 160]),label='Med Der')
> plt.plot(y, fuzz.trimf(y, [145, 160, 160]),label='Muy Der')
> plt.title('Funciones de Membresia - Salida (Angulo Servo)')
> plt.xlabel('Angulo (grados)')
> plt.ylabel('Grado de pertenencia')
> plt.legend(loc='upper right', fontsize=7)
> plt.tight_layout()
> plt.savefig('membresia_salida.png', dpi=200)
> plt.show()
> ```

---

### 4.3 Maquina de Estados — Estaciones de Color

```
DETECCION DE ESTACIONES:
  - 3 colores detectados en espacio HSV: Rojo, Azul, Amarillo
  - ROI de deteccion de color: franja 55% - 85% del frame
  - Anti-rebote por transicion de flanco (solo cuenta
    cuando la linea pasa de no-visible a visible)
  - Parada al acumular 4 lineas del mismo color

ESTADOS:
  [SEGUIR_LINEA] ---(4ta linea Rojo/Azul)---> [ESPERA_ESTACION]
  [SEGUIR_LINEA] ---(4ta linea Amarillo)----> [REPOSO_AMARILLO]
  [ESPERA_ESTACION] ---(5 seg)--------------> [SEGUIR_LINEA]
  [REPOSO_AMARILLO] ---(SCADA GO / btn ON)--> [SEGUIR_LINEA]

ZONAS OPERATIVAS:
  Azul     = CARGA     (espera 5 s, sale automaticamente)
  Rojo     = DESCARGA  (espera 5 s, sale automaticamente)
  Amarillo = REPOSO    (espera indefinida, comando externo)

COOLDOWN: 3 s de inhibicion al salir de estacion
          para evitar re-deteccion inmediata
```

---

### 4.4 Comunicacion IoT y SCADA

```
PROTOCOLO: MQTT (broker Mosquitto, puerto 1883)
ARQUITECTURA: Raspberry Pi 4B <--WiFi/MQTT--> ESP32

TOPICS MQTT:
  esp32/servo/control   Pi -> ESP32   Angulo servo (65-160)
  esp32/arranque        SCADA -> ESP32  GO / STOP / EMERGENCIA
  arranque/paro         ESP32 -> SCADA  MOVIMIENTO / PARO / EMERGENCIA
  bateria/porcentaje    ESP32 -> SCADA  Porcentaje 0-100%
  LM35/uno              ESP32 -> SCADA  Temperatura en C
  agv/estacion          Pi -> SCADA     CARGA / DESCARGA / REPOSO / EN_RUTA

INTERFAZ SCADA (Tkinter, tema oscuro):
  - 3 focos de estado: Paro (rojo) / Movimiento (verde) / Emergencia (amarillo)
  - 4 focos de zona: Carga / Descarga / Reposo / En Ruta
  - 5 botones: Arranque, Paro, E-Stop, Reset E-Stop, Generar Reporte
  - Grafica de temperatura en tiempo real (historial 60 s)
  - Barra de nivel de bateria con alertas por color
  - Alertas automaticas: temp > 40 C, bateria < 20%
  - Registro CSV + generacion de reportes PDF

SEGURIDAD:
  - E-Stop fisico (NC) con interrupcion de hardware
  - E-Stop virtual desde interfaz SCADA
  - Reset requiere condicion segura (boton NC cerrado)
```

---

## DIAGRAMAS DE BLOQUES Y FLUJO

> **[NOTA]** Todos los diagramas de esta seccion estan en ASCII para que los
> redibujes en PowerPoint, Canva o draw.io con cajas de colores y flechas limpias.
> Cada diagrama tiene un titulo y una breve descripcion de colores sugeridos.

---

### DIAGRAMA 1 — Composicion General del Sistema

> Muestra los 3 subsistemas principales (AGV, SCADA, PLC) y como se comunican
> a traves del broker MQTT. Este es el diagrama de nivel mas alto.
> **Colores sugeridos:** AGV=azul, SCADA=verde, PLC=naranja, MQTT=gris oscuro

```
+=====================================================================+
|                     SISTEMA COMPLETO — AGV ACKERMANN                |
+=====================================================================+

+--------------------------+        +--------------------------+
|       RASPBERRY PI 4B    |        |         ESP32            |
|       (Procesamiento)    |        |      (Actuacion)         |
|                          |        |                          |
|  +--------------------+  |        |  +--------------------+  |
|  | ControlLinea       |  |  MQTT  |  | Servo direccion    |  |
|  | skfuzzys2.py       |--|------->|  | (65-160 grados)    |  |
|  | - Vision artificial|  |        |  +--------------------+  |
|  | - Control difuso   |  |        |  +--------------------+  |
|  | - Maq. de estados  |  |        |  | Motor DC           |  |
|  +--------------------+  |        |  | (TB6612FNG)        |  |
|                          |        |  +--------------------+  |
|  +--------------------+  |  MQTT  |  +--------------------+  |
|  | Broker Mosquitto   |--|<-------|  | Sensores:          |  |
|  | Puerto 1883        |  |        |  | LM35 + Bateria ADC |  |
|  +--------------------+  |        |  +--------------------+  |
|                          |        |  +--------------------+  |
|  +--------------------+  |  MQTT  |  | Botones fisicos:   |  |
|  | Scada2.py          |--|<------>|  | ON / OFF / E-Stop  |  |
|  | (Interfaz SCADA)   |  |        |  +--------------------+  |
|  +--------------------+  |        |  +--------------------+  |
|                          |        |  | LEDs indicadores   |  |
+--------------------------+        |  | Izq / Der / Estado |  |
          |                         +--+--------------------+--+
          | MQTT                                   ^
          v                                        |
+--------------------------+                       |
|   PLC SIEMENS S7-1200    |        Node-RED       |
|   (via Node-RED)         |------(S7 + MQTT)------+
|                          |
|  +--------------------+  |
|  | ENTRADAS (botones)  |  |
|  | I0.0 = Arranque     |  |
|  | I0.1 = Paro         |  |
|  | I0.2 = Emergencia   |  |
|  +--------------------+  |
|  +--------------------+  |
|  | SALIDAS (luces)     |  |
|  | Q0.0 = MOVIMIENTO   |  |
|  |        (verde)      |  |
|  | Q0.1 = PARO         |  |
|  |        (roja)       |  |
|  | Q0.2 = EMERGENCIA   |  |
|  |        (amarilla)   |  |
|  +--------------------+  |
+--------------------------+
```

---

### DIAGRAMA 2 — Bloques Internos del AGV (Raspberry Pi + ESP32)

> Detalle de los modulos de software y hardware que componen el AGV.
> **Colores sugeridos:** Vision=cyan, Difuso=morado, MQTT=gris,
> Actuadores=rojo, Sensores=amarillo

```
+====================================================================+
|                    AGV — DIAGRAMA DE BLOQUES                       |
+====================================================================+

  RASPBERRY PI 4B                          ESP32
 +--------------------------------+       +----------------------------+
 |                                |       |                            |
 |  +---------+   +------------+ |       | +------------------------+ |
 |  | Camara  |-->| Conversion | | MQTT  | |   Callback MQTT        | |
 |  | CSI     |   | BGR        | | WiFi  | |   (recibe angulo,      | |
 |  | 640x480 |   +-----+------+ |------>| |    GO, STOP, E-STOP)   | |
 |  +---------+         |        |       | +-----+-----+-----+------+ |
 |                       v        |       |       |     |     |        |
 |               +-------+------+ |       |       v     v     v        |
 |               | Deteccion    | |       | +-----++ +--+--+ +------+ |
 |               | linea blanca | |       | |Servo | |Motor| |LEDs  | |
 |               | (umbral+     | |       | |dir.  | |DC   | |Izq   | |
 |               |  contornos)  | |       | |65-160| |TB6612| |Der  | |
 |               +-------+------+ |       | |GPIO 5| |GPIO | |GPIO  | |
 |                       |        |       | |      | |18,19| |15,16 | |
 |                       v        |       | +------+ |21,22| +------+ |
 |               +-------+------+ |       |          +-----+          |
 |               | Centroide    | |       |                            |
 |               | Posicion     | |       | +------------------------+ |
 |               | normalizada  | |       | |   Lectura sensores     | |
 |               | [0, 1]       | |       | |   (cada 1 segundo)    | |
 |               +-------+------+ |       | |                        | |
 |                       |        |       | | +------+  +---------+  | |
 |                       v        |       | | |LM35  |  |Bateria  |  | |
 |               +-------+------+ |       | | |GPIO36|  |GPIO34   |  | |
 |               | Control      | |       | | |temp C|  |ADC 20x  |  | |
 |               | Difuso       | |       | | +---+--+  +----+----+  | |
 |               | Mamdani      | |       | |     |          |       | |
 |               | (skfuzzy)    | |       | +-----+----------+-------+ |
 |               | 7 reglas     | |       |       |          |         |
 |               | centroide    | |  MQTT |       v          v         |
 |               +-------+------+ |<------| +----------+-----------+  |
 |                       |        |       | |Publica:  |Publica:   |  |
 |                       v        |       | |LM35/uno  |bateria/   |  |
 |               +-------+------+ |       | |          |porcentaje |  |
 |               | Angulo servo | |       | +----------+-----------+  |
 |               | [65 - 160]   | |       |                            |
 |               +-------+------+ |       | +------------------------+ |
 |                       |        |       | |   Seguridad            | |
 |                       v        |       | | +------+ +----------+  | |
 |               +-------+------+ |       | | |E-Stop| |Botones   |  | |
 |               | Deteccion    | |       | | |NC    | |ON / OFF  |  | |
 |               | lineas color | |       | | |GPIO13| |GPIO 12,4 |  | |
 |               | HSV (R,A,B)  | |       | | |ISR HW| |debounce  |  | |
 |               +-------+------+ |       | | +------+ +----------+  | |
 |                       |        |       | +------------------------+ |
 |                       v        |       |                            |
 |               +-------+------+ |       | +------------------------+ |
 |               | Maquina de   | | MQTT  | | Publica estado solo    | |
 |               | estados      | |<------| | cuando cambia:         | |
 |               | (3 estados)  | |       | | arranque/paro ->       | |
 |               | STOP/GO MQTT | |------>| | MOVIMIENTO/PARO/       | |
 |               +--------------+ |       | | EMERGENCIA             | |
 |                                |       | +------------------------+ |
 +--------------------------------+       +----------------------------+
```

---

### DIAGRAMA 3 — Bloques de la Interfaz SCADA

> Detalle de los componentes de la interfaz SCADA (Scada2.py).
> **Colores sugeridos:** Fondo oscuro (#1e1e1e), paneles (#2d2d2d),
> focos con colores reales

```
+====================================================================+
|                   SCADA — DIAGRAMA DE BLOQUES                      |
+====================================================================+

                    +---------------------------+
                    |    Cliente MQTT            |
                    |    (paho-mqtt)             |
                    |    Broker: 192.168.1.68    |
                    |    Puerto: 1883            |
                    +-----+-----------+---------+
                          |           |
               Suscribe:  |           |  Publica:
               arranque/paro          |  esp32/arranque
               LM35/uno              |  (GO, STOP,
               bateria/porcentaje     |   EMERGENCIA,
               agv/estacion           |   RESET_EMERGENCIA)
                          |           |
                    +-----v-----------v---------+
                    |    Logica de Procesamiento |
                    |    (callbacks on_message)  |
                    +---+---+---+---+---+-------+
                        |   |   |   |   |
          +-------------+   |   |   |   +------------+
          |                 |   |   |                 |
          v                 v   |   v                 v
 +--------+------+  +------+-+ | +-+-------+  +------+--------+
 | PANEL ESTADO   |  | PANEL  | | | PANEL   |  | REGISTRO      |
 | DEL AGV        |  | ZONA   | | | BATERIA |  | DE EVENTOS    |
 |                |  | DEL AGV| | |         |  |               |
 | (O) PARO       |  |        | | | Barra   |  | eventos.csv   |
 |     [rojo]     |  | (O) CARGA   | | 0-100%  |  | Fecha, Hora,  |
 | (O) MOVIMIENTO |  |     [azul]  | | Colores:|  | Evento, Origen|
 |     [verde]    |  | (O) DESCARGA| | <20%=rojo  | Temp, Bateria |
 | (O) EMERGENCIA |  |     [rojo]  | | <50%=amar. |               |
 |     [amarillo] |  | (O) REPOSO  | | >50%=cyan  +-------+-------+
 |                |  |     [amari.] |            |        |
 | "Estado: XXX"  |  | (O) EN RUTA |            v        v
 +----------------+  |     [verde] | |         | +-------+-------+
                     |             | |         | | GENERAR       |
                     | "Zona: XXX" | |         | | REPORTE PDF   |
                     +-------------+ |         | |               |
                                     v         | | - Grafica temp|
                          +----------+-------+ | | - Tabla eventos
                          | GRAFICA          | | | - Conteo alarmas
                          | TEMPERATURA      | | +---------------+
                          |                  | |
                          | Historial 60 s   | |
                          | Linea suavizada  | |
                          | Alerta en 40 C   | |
                          | (linea roja)     | |
                          | Punto ultimo val | |
                          | Cuadricula 10 C  | |
                          +------------------+ |
                                               |
                          +--------------------v---+
                          | PANEL DE CONTROL       |
                          |                        |
                          | [  ARRANQUE  ] (verde)  |
                          |   -> publica "GO"      |
                          |                        |
                          | [    PARO    ] (rojo)   |
                          |   -> publica "STOP"    |
                          |                        |
                          | [   E-STOP   ] (amarillo)
                          |   -> publica            |
                          |      "EMERGENCIA"      |
                          |                        |
                          | [ RESET E-STOP ] (gris) |
                          |   -> publica            |
                          |    "RESET_EMERGENCIA"  |
                          |                        |
                          | [GEN. REPORTE] (azul)   |
                          |   -> PDF con grafica   |
                          +------------------------+
```

---

### DIAGRAMA 4 — Bloques del PLC Siemens S7-1200

> El PLC se conecta al sistema via Node-RED (puente S7 <-> MQTT).
> **Colores sugeridos:** Entradas=verde, Salidas=rojo/amarillo/verde,
> Node-RED=naranja, DB1=azul claro

```
+====================================================================+
|               PLC SIEMENS S7-1200 — DIAGRAMA DE BLOQUES            |
+====================================================================+

  ENTRADAS FISICAS                DATA BLOCK (DB1)              SALIDAS FISICAS
 +------------------+         +----------------------+        +-------------------+
 |                  |         |       DB1 - DB_AGV   |        |                   |
 | +------+         |  LAD    |                      |  LAD   |  +-------------+  |
 | | I0.0 |---------|-------->| DBX0.0 btn_arranque  |------->|  | Q0.0        |  |
 | | Boton|         |         |                      |        |  | Luz VERDE   |  |
 | | Arranque       |         | DBX0.1 btn_paro      |------->|  | (MOVIMIENTO)|  |
 | +------+         |         |                      |        |  +-------------+  |
 |                  |         | DBX0.2 btn_emergencia |        |                   |
 | +------+         |         |                      |        |  +-------------+  |
 | | I0.1 |---------|-------->| DBX0.3 btn_reset     |------->|  | Q0.1        |  |
 | | Boton|         |         |                      |        |  | Luz ROJA    |  |
 | | Paro |         |         +----------+-----------+        |  | (PARO)      |  |
 | +------+         |                    |                    |  +-------------+  |
 |                  |                    | Protocolo S7       |                   |
 | +------+         |                    | Puerto 102         |  +-------------+  |
 | | I0.2 |---------|-------->           |                    |  | Q0.2        |  |
 | | Boton|         |              +-----v------+             |  | Luz AMARILLA|  |
 | | Emergencia     |              |            |             |  | (EMERGENCIA)|  |
 | +------+         |              |  Node-RED  |             |  +-------------+  |
 |                  |              |  (Raspi)   |             |                   |
 +------------------+              |            |             +-------------------+
                                   +-----+------+
                                         |
                        +----------------+----------------+
                        |                                 |
                        v                                 v
               Lee botones PLC              Escribe estado AGV
               y publica MQTT:              desde MQTT al PLC:

               I0.0 = true                  "MOVIMIENTO" ->
                -> "GO"                      DBX1.0 = true
               I0.1 = true                   -> Q0.0 ON (verde)
                -> "STOP"
               I0.2 = true                 "PARO" ->
                -> "EMERGENCIA"              DBX1.1 = true
                                              -> Q0.1 ON (roja)
               Topic:
               esp32/arranque              "EMERGENCIA" ->
                        |                    DBX1.2 = true
                        v                     -> Q0.2 ON (amarilla)
                  +-----------+
                  | Broker    |            Topic:
                  | Mosquitto |            arranque/paro
                  | :1883     |
                  +-----------+
```

---

### DIAGRAMA 5 — Flujo de Comunicacion entre los 3 Subsistemas

> Muestra el flujo de datos MQTT entre AGV, SCADA y PLC.
> **Colores sugeridos:** flechas de publicacion=azul,
> flechas de suscripcion=verde, broker=gris central

```
+====================================================================+
|          FLUJO DE COMUNICACION MQTT — SISTEMA COMPLETO             |
+====================================================================+

  +------------+                                      +------------+
  |    AGV     |                                      |   SCADA    |
  | (Raspi +   |                                      | (Raspi     |
  |  ESP32)    |                                      |  Tkinter)  |
  +------+-----+                                      +-----+------+
         |                                                   |
         |  Publica:                          Publica:       |
         |  esp32/servo/control (angulo)      esp32/arranque |
         |  arranque/paro (estado)            (GO/STOP/      |
         |  bateria/porcentaje                EMERGENCIA)    |
         |  LM35/uno                                         |
         |  agv/estacion (zona)                              |
         |                                                   |
         v                                                   v
   +-----+---------------------------------------------------+-----+
   |                                                               |
   |                    BROKER MOSQUITTO                            |
   |                    Puerto 1883                                |
   |                    (Raspberry Pi)                             |
   |                                                               |
   +-----+---------------------------------------------------+-----+
         ^                                                   ^
         |                                                   |
         |  Suscribe:                         Suscribe:      |
         |  esp32/servo/control               arranque/paro  |
         |  esp32/arranque                    LM35/uno       |
         |                                    bateria/%      |
         |                                    agv/estacion   |
         |                                                   |
   +-----+------+                                     +-----+------+
   |    ESP32    |                                     |    PLC     |
   | (Actuacion) |                                     | S7-1200   |
   |             |                                     | (Node-RED)|
   +-------------+                                     +------------+
         ^                                                   |
         |                   Publica:                        |
         +---------------------------------------------------+
                             esp32/arranque
                             (GO/STOP/EMERGENCIA
                              desde botones PLC)
```

---

### DIAGRAMA 6 — Diagrama de Flujo del AGV (ControlLinea_skfuzzys2.py)

> Flujo completo del programa principal del AGV.
> **Colores sugeridos:** Inicio/Fin=negro, Decisiones=amarillo,
> Procesos=azul, MQTT=verde, Estados=naranja

```
+====================================================================+
|            DIAGRAMA DE FLUJO — AGV (ControlLinea_skfuzzys2.py)     |
+====================================================================+

                        ( INICIO )
                            |
                            v
                  +-------------------+
                  | Crear sistema     |
                  | difuso Mamdani    |
                  | (7 reglas,        |
                  |  skfuzzy)         |
                  +--------+----------+
                           |
                           v
                  +-------------------+
                  | Iniciar camara    |
                  | CSI Picamera2     |
                  | 640 x 480         |
                  +--------+----------+
                           |
                           v
                  +-------------------+
                  | Conectar broker   |
                  | MQTT (Mosquitto)  |
                  | Suscribir a       |
                  | arranque/paro     |
                  +--------+----------+
                           |
                           v
                  +-------------------+
                  | Publicar zona:    |
                  | "EN_RUTA"         |
                  +--------+----------+
                           |
                           v
            +=================================+
            |     BUCLE PRINCIPAL (while)     |
            +=================================+
                           |
                           v
                  +-------------------+
                  | Capturar frame    |
                  | Convertir a BGR   |
                  +--------+----------+
                           |
                           v
                /---------------------\
               / estado ==             \
              /  SEGUIR_LINEA ?         \
              \                         /
               \-----------+-----------/
                   |               |
                  SI              NO (ver estados abajo)
                   |
                   v
          /-------------------\
         /  en_cooldown ?      \
         \                     /
          \--------+----------/
              |           |
             SI          NO
              |           |
              v           v
     (no detectar    +-------------------+
      colores)       | Detectar lineas   |
              |      | de color (HSV)    |
              |      | Rojo, Azul,       |
              |      | Amarillo          |
              |      +--------+----------+
              |               |
              |               v
              |      /-------------------\
              |     / Linea color nueva   \
              |    /  (flanco subida)?      \
              |    \                        /
              |     \--------+------------/
              |         |           |
              |        SI          NO
              |         |           |
              |         v           |
              |  +-------------+    |
              |  | contador++  |    |
              |  | del color   |    |
              |  +------+------+    |
              |         |           |
              |         v           |
              |  /-------------\    |
              |  | contador    |    |
              |  | >= 4 ?      |    |
              |  \------+------/    |
              |    |         |      |
              |   SI        NO      |
              |    |         |      |
              |    v         +------+-------+
              |  +----------------+         |
              |  | Enviar STOP    |         |
              |  | via MQTT       |         |
              |  | Publicar zona  |         |
              |  | Reset contador |         |
              |  +-------+--------+         |
              |          |                  |
              |          v                  |
              |  /---------------\          |
              |  | Color ==      |          |
              |  | Amarillo ?    |          |
              |  \-------+------/           |
              |     |         |             |
              |    SI        NO             |
              |     |         |             |
              |     v         v             |
              | (estado=   (estado=         |
              | REPOSO_    ESPERA_          |
              | AMARILLO)  ESTACION)        |
              |     |         |             |
              |     +---------+             |
              |          |                  |
              |      (continue)             |
              |                             |
              +-----------------------------+
                             |
                             v
                  +-------------------+
                  | Detectar linea    |
                  | blanca            |
                  | (grises, umbral,  |
                  |  morfologia,      |
                  |  contornos)       |
                  +--------+----------+
                           |
                           v
                  /-------------------\
                 / Linea blanca        \
                /  detectada?           \
                \                       /
                 \---------+-----------/
                      |           |
                     SI          NO
                      |           |
                      v           v
              +-----------+  +-----------------+
              | Calcular  |  | BUSQUEDA ACTIVA |
              | centroide |  | ultima_pos<0.5? |
              | pos_norm  |  |  SI -> servo=65 |
              | [0, 1]    |  |  NO -> servo=160|
              +-----+-----+  +--------+--------+
                    |                  |
                    v                  |
              +-----------+            |
              | Control   |            |
              | difuso    |            |
              | Mamdani   |            |
              | -> angulo |            |
              +-----+-----+           |
                    |                  |
                    +--------+---------+
                             |
                             v
                    /------------------\
                   / |angulo - ultimo| \
                  /  > 2 grados ?     \
                  \                    /
                   \--------+---------/
                       |          |
                      SI         NO
                       |          |
                       v          v
               +-------------+ (no enviar)
               | Publicar    |
               | angulo MQTT |
               | esp32/servo |
               | /control    |
               +------+------+
                      |
                      v
               +-------------+
               | Dibujar HUD |
               | (OpenCV)    |
               +------+------+
                      |
                      v
               /-------------\
              / ESC presionado?\
              \                /
               \------+------/
                 |         |
                SI        NO
                 |         |
                 v         +---> (volver al BUCLE PRINCIPAL)
              ( FIN )


  =====================================================
  SUBESTADOS (cuando estado != SEGUIR_LINEA):
  =====================================================

  ESPERA_ESTACION (Rojo o Azul):
  +-------------------+
  | Mostrar en HUD:   |
  | "ESTACION - color" |
  | "Esperando: X.Xs" |
  +--------+----------+
           |
           v
  /-------------------\
  | transcurrido       |
  | >= 5 segundos ?    |
  \--------+----------/
      |         |
     SI        NO
      |         |
      v         +---> (mostrar cuenta regresiva, volver al bucle)
  +-------------------+
  | Enviar GO via MQTT|
  | Publicar EN_RUTA  |
  | Activar cooldown  |
  | estado =          |
  |  SEGUIR_LINEA     |
  +-------------------+


  REPOSO_AMARILLO:
  +-------------------+
  | Mostrar en HUD:   |
  | "REPOSO -          |
  |  Estacion Amarilla"|
  | "Esperando comando"|
  +--------+----------+
           |
           v
  /-------------------\
  | estado_esp32 ==    |
  | "MOVIMIENTO" ?     |
  | (SCADA envio GO    |
  |  o boton ON)       |
  \--------+----------/
      |         |
     SI        NO
      |         |
      v         +---> (seguir esperando, volver al bucle)
  +-------------------+
  | Publicar EN_RUTA  |
  | Activar cooldown  |
  | estado =          |
  |  SEGUIR_LINEA     |
  +-------------------+
```

---

### DIAGRAMA 7 — Diagrama de Flujo del SCADA (Scada2.py)

> Flujo del programa SCADA: inicializacion, callbacks MQTT
> y bucle de actualizacion de interfaz.
> **Colores sugeridos:** Inicio=negro, MQTT=verde, UI=azul,
> Alertas=rojo, Reportes=naranja

```
+====================================================================+
|            DIAGRAMA DE FLUJO — SCADA (Scada2.py)                   |
+====================================================================+

                        ( INICIO )
                            |
                            v
                  +-------------------+
                  | Inicializar CSV   |
                  | (crear archivo    |
                  |  si no existe)    |
                  +--------+----------+
                           |
                           v
                  +-------------------+
                  | Conectar broker   |
                  | MQTT              |
                  | Suscribir:        |
                  | - arranque/paro   |
                  | - LM35/uno        |
                  | - bateria/%       |
                  | - agv/estacion    |
                  +--------+----------+
                           |
                           v
                  +-------------------+
                  | Crear ventana     |
                  | Tkinter           |
                  | (tema oscuro)     |
                  +--------+----------+
                           |
                           v
                  +-------------------+
                  | Crear widgets:    |
                  | - 3 focos estado  |
                  | - 4 focos zona    |
                  | - 5 botones       |
                  | - Barra bateria   |
                  | - Grafica temp    |
                  +--------+----------+
                           |
                           v
                  +-------------------+
                  | Iniciar bucle     |
                  | actualizar_       |
                  | interfaz()        |
                  | cada 500 ms       |
                  +--------+----------+
                           |
                           v
            +=================================+
            |         mainloop()              |
            |   (Tkinter + MQTT en paralelo)  |
            +=================================+
                     |               |
                     v               v
         +-----------+--+   +-------+------------+
         | CALLBACKS    |   | ACTUALIZACION      |
         | MQTT         |   | INTERFAZ           |
         | (on_message) |   | (cada 500 ms)      |
         +-----------+--+   +-------+------------+
                     |               |
        +------------+----+         |
        |            |    |         |
        v            v    v         |
  +----------+ +------+ +------+   |
  |arranque/ | |LM35/ | |bat/  |   |
  |paro      | |uno   | |%     |   |
  +----+-----+ +--+---+ +--+---+   |
       |          |         |       |
       v          v         v       |
  +---------+ +-------+ +------+   |
  |Cambio   | |Guardar| |Guardar   |
  |estado?  | |en     | |valor  |  |
  |SI ->    | |deque  | |       |  |
  |Registrar| |(60pts)| |       |  |
  |evento   | +---+---+ +--+---+  |
  |en CSV   |     |         |      |
  |Detectar |     v         v      |
  |origen:  | /-------\ /------\   |
  |VIRTUAL  | |temp   | |bat   |   |
  |o FISICA | |> 40C? | |< 20%?|  |
  +---------+ \---+---/ \--+---/   |
                  |         |      |
                 SI        SI      |
                  |         |      |
                  v         v      |
           +---------+ +--------+  |
           |ALERTA   | |ALERTA  |  |
           |TEMP     | |BAT     |  |
           |messagebox |Registrar  |
           |Registrar| |en CSV  |  |
           |en CSV   | +--------+  |
           +---------+             |
                                   |
                        +----------v----------+
                        | ACTUALIZAR WIDGETS: |
                        |                     |
                        | 1. Focos de estado  |
                        |    segun estado_agv |
                        |    PARO -> rojo ON  |
                        |    MOV  -> verde ON |
                        |    EMER -> amar. ON |
                        |                     |
                        | 2. Focos de zona    |
                        |    segun zona_agv   |
                        |    CARGA -> azul ON |
                        |    DESC  -> rojo ON |
                        |    REPOSO-> amar ON |
                        |    RUTA  -> verde ON|
                        |                     |
                        | 3. Barra bateria    |
                        |    ancho segun %    |
                        |    <20% = rojo      |
                        |    <50% = amarillo  |
                        |    >50% = cyan      |
                        |                     |
                        | 4. Valor temperatura|
                        |    >40C = rojo      |
                        |                     |
                        | 5. Redibujar grafica|
                        |    temperatura      |
                        |    (cuadricula,     |
                        |     linea alerta,   |
                        |     curva, punto)   |
                        +----------+----------+
                                   |
                                   v
                          (esperar 500 ms)
                          (repetir bucle)


  =====================================================
  ACCIONES DE BOTONES (eventos de usuario):
  =====================================================

  [ARRANQUE]       -> mqtt.publish("esp32/arranque", "GO")
  [PARO]           -> mqtt.publish("esp32/arranque", "STOP")
  [E-STOP]         -> mqtt.publish("esp32/arranque", "EMERGENCIA")
  [RESET E-STOP]   -> mqtt.publish("esp32/arranque", "RESET_EMERGENCIA")
  [GEN. REPORTE]   -> generar_reporte():
                       1. Leer eventos.csv
                       2. Contar alarmas
                       3. Generar grafica_temp.png
                       4. Crear PDF con:
                          - Titulo + fecha
                          - Conteo de alarmas
                          - Imagen de grafica
                          - Tabla de eventos
                       5. Guardar Reporte_SCADA_YYYY-MM-DD.pdf
```

---

### DIAGRAMA 8 — Diagrama de Flujo del PLC (Node-RED como puente)

> Flujo de la logica del PLC y Node-RED.
> **Colores sugeridos:** PLC=naranja, Node-RED=verde, MQTT=gris

```
+====================================================================+
|        DIAGRAMA DE FLUJO — PLC S7-1200 + NODE-RED                  |
+====================================================================+

  ==============================
  FLUJO 1: PLC -> AGV (botones)
  ==============================

            +------------------+
            | PLC: Programa    |
            | LAD en OB1       |
            +--------+---------+
                     |
                     v
            /------------------\
           / Boton I0.0         \
          /  presionado?         \
          \                      /
           \--------+-----------/
               |           |
              SI          NO
               |           |
               v           |
       DB1.DBX0.0 = TRUE   |
               |           |
               v           v
            (igual para I0.1 -> DBX0.1,
             I0.2 -> DBX0.2)
                     |
                     v
            +------------------+
            | Node-RED:        |
            | Nodo "s7 in"     |
            | Lee DB1.DBX0.x   |
            | cada 500 ms      |
            +--------+---------+
                     |
                     v
            /------------------\
           / Valor cambio       \
          /  (flanco subida)?    \
          \                      /
           \--------+-----------/
               |           |
              SI          NO
               |           |
               v        (ignorar)
            +------------------+
            | Nodo "function": |
            | DBX0.0 -> "GO"   |
            | DBX0.1 -> "STOP" |
            | DBX0.2 ->        |
            |  "EMERGENCIA"    |
            +--------+---------+
                     |
                     v
            +------------------+
            | Nodo "mqtt out": |
            | Topic:           |
            | esp32/arranque   |
            +------------------+


  ==============================
  FLUJO 2: AGV -> PLC (estado)
  ==============================

            +------------------+
            | Node-RED:        |
            | Nodo "mqtt in"   |
            | Topic:           |
            | arranque/paro    |
            +--------+---------+
                     |
                     v
            +------------------+
            | Nodo "function": |
            | Decodificar:     |
            |                  |
            | "MOVIMIENTO" ->  |
            |   salida 1: true |
            | "PARO" ->        |
            |   salida 2: true |
            | "EMERGENCIA" ->  |
            |   salida 3: true |
            +---+---------+----+
                |    |    |
                v    v    v
            +------------------+
            | 3x Nodo "s7 out":|
            | Escribe en PLC:  |
            |                  |
            | DB1.DBX1.0       |
            |  = MOVIMIENTO    |
            | DB1.DBX1.1       |
            |  = PARO          |
            | DB1.DBX1.2       |
            |  = EMERGENCIA    |
            +--------+---------+
                     |
                     v
            +------------------+
            | PLC: Programa    |
            | LAD en OB1       |
            |                  |
            | DBX1.0 -> Q0.0   |
            |  Luz VERDE       |
            | DBX1.1 -> Q0.1   |
            |  Luz ROJA        |
            | DBX1.2 -> Q0.2   |
            |  Luz AMARILLA    |
            +------------------+
```

---

## 5. RESULTADOS

### Tabla de parametros del sistema

```
+------------------------------------+---------------------------+
| Parametro                          | Valor                     |
+------------------------------------+---------------------------+
| Conjuntos difusos (entrada/salida) | 7 / 7                     |
| Reglas difusas                     | 7 (IF-THEN directas)      |
| Defuzzificacion                    | Centroide                 |
| Rango del servo                    | 65 - 160 (+-48 simetrico)|
| Centro (recto)                     | 113                       |
| Resolucion de camara               | 640 x 480 px              |
| Umbral de envio MQTT               | Cambio > 2 grados         |
| Area minima linea blanca           | 300 px                    |
| Area minima linea color            | 1500 px                   |
| Lineas para estacionar             | 4 por color               |
| Tiempo en estacion Rojo/Azul       | 5 s (automatico)          |
| Estacion Amarillo                  | Indefinido (espera SCADA) |
| Cooldown post-estacion             | 3 s                       |
| Lectura de sensores                | 1 Hz                      |
| Muestras ADC bateria               | 20 (promediadas)          |
| Debounce botones                   | 50 ms                     |
| Respuesta E-Stop                   | Microsegundos (ISR HW)    |
| Alerta temperatura                 | > 40 C                    |
| Alerta bateria                     | < 20%                     |
| Actualizacion SCADA                | Cada 500 ms               |
+------------------------------------+---------------------------+
```

### Pruebas funcionales validadas

```
[OK] Seguimiento de linea blanca en curvas de radio 80 cm y 150 cm
[OK] Transiciones suaves de direccion (sin oscilacion)
     gracias al traslape de conjuntos difusos
[OK] Recuperacion activa al perder la linea
     (busqueda por ultimo lado conocido)
[OK] Deteccion y conteo confiable de 3 colores (HSV)
     con anti-rebote por transicion
[OK] Parada precisa en estaciones tras 4ta linea
[OK] Paro de emergencia dual (fisico NC + virtual SCADA)
     con bloqueo completo del sistema
[OK] Interfaz SCADA con actualizacion en tiempo real
[OK] Generacion de reportes PDF con grafica y tabla de eventos
```

```
[VISUAL] Captura del HUD de OpenCV mostrando:
         - Linea blanca detectada con contorno y centroide
         - Angulo del servo en pantalla
         - Contadores de lineas de color
         (ejecutar ControlLinea_skfuzzys2.py y capturar)
```

---

## 6. CONCLUSIONES Y TRABAJO FUTURO

### Conclusiones

```
- Se logro un sistema AGV funcional que integra vision artificial,
  control difuso Mamdani y comunicacion IoT. La logica difusa con
  7 conjuntos y defuzzificacion por centroide produce una direccion
  suave y estable en geometria Ackermann, eliminando las
  oscilaciones tipicas de un controlador ON/OFF.

- La arquitectura modular (Raspberry Pi + ESP32 + MQTT) separa
  el procesamiento de vision, la actuacion y la supervision,
  facilitando la escalabilidad y el mantenimiento del sistema.

- La maquina de estados con deteccion de estaciones por color
  en espacio HSV permite simular operaciones logisticas reales
  (carga, descarga, reposo) de forma autonoma con intervencion
  remota via SCADA.
```

### Trabajo Futuro

```
- Implementar odometria y fusion sensorial (IMU + encoders)
  para localizacion absoluta del AGV en la pista.

- Integrar PLC Siemens S7-1200 via Node-RED para supervision
  industrial completa (guia ya documentada en el proyecto).

- Agregar planificacion de rutas con algoritmos de grafos
  para navegacion entre multiples estaciones con prioridades
  dinamicas.
```

---

## 7. PIE DE CARTEL / DATOS DEL AUTOR

```
Autor:        Josue
Matricula:    im221466
Correo:       im221466@itsatlixco.edu.mx
Institucion:  Instituto Tecnologico Superior de Atlixco
Carrera:      Ingenieria Mecatronica
Fecha:        Mayo 2026
```

---

## 8. TECNOLOGIAS (para franja inferior o lateral del cartel)

```
Python 3 | OpenCV | scikit-fuzzy | NumPy | MQTT | Mosquitto |
Raspberry Pi 4B | Picamera2 | ESP32 | Arduino | TB6612FNG |
Tkinter | ReportLab | Matplotlib | paho-mqtt | Node-RED
```

> **[NOTA]** Si tu formato de cartel tiene espacio para logos, usa los logos de:
> Python, OpenCV, Raspberry Pi, Arduino/ESP32, MQTT

---

## CHECKLIST DE VISUALES PARA EL CARTEL

```
[ ] Foto del AGV armado (con etiquetas de componentes)
[ ] Foto de la pista (240x240 cm con estaciones de color)
[ ] DIAGRAMA 1: Composicion general del sistema (AGV + SCADA + PLC)
[ ] DIAGRAMA 2: Bloques internos del AGV (Raspi + ESP32)
[ ] DIAGRAMA 3: Bloques de la interfaz SCADA
[ ] DIAGRAMA 4: Bloques del PLC S7-1200 + Node-RED
[ ] DIAGRAMA 5: Flujo de comunicacion MQTT entre subsistemas
[ ] DIAGRAMA 6: Diagrama de flujo del AGV (programa principal)
[ ] DIAGRAMA 7: Diagrama de flujo del SCADA
[ ] DIAGRAMA 8: Diagrama de flujo del PLC / Node-RED
[ ] Grafica de funciones de membresia de ENTRADA (generada con snippet)
[ ] Grafica de funciones de membresia de SALIDA (generada con snippet)
[ ] Captura de pantalla de la interfaz SCADA
[ ] Captura del HUD de OpenCV (deteccion en funcionamiento)
[ ] Tabla de resultados (parametros del sistema)
```

> **[NOTA] Consejo de diseno:**
> - Usa fondo oscuro o blanco limpio, evita degradados excesivos
> - Tipografia sans-serif (Arial, Helvetica, Calibri)
> - Titulo: 72-96 pt | Subtitulos: 36-48 pt | Cuerpo: 24-28 pt
> - Las imagenes deben ocupar al menos 40% del area total del cartel
> - Flujo de lectura: de arriba a abajo, de izquierda a derecha
> - Resalta los numeros clave en negritas o color de acento
> - Los diagramas ASCII de esta guia son PLANTILLAS: redibujalos
>   en PowerPoint, Canva o draw.io con cajas de colores y flechas
> - Prioriza los diagramas 1, 2, 6 y 7 si el espacio es limitado
>   (composicion general, bloques AGV, flujo AGV y flujo SCADA)
