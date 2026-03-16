# Vision-Integrador — Documentación del Proyecto

> **Estado:** Prototipo funcional
> **Versión:** 3.0
> **Autor:** Josue (im221466@itsatlixco.edu.mx)
> **Fecha de inicio:** Marzo 2026

---

## Tabla de Contenidos

1. [¿Qué es el proyecto?](#1-qué-es-el-proyecto)
2. [¿Para qué sirve?](#2-para-qué-sirve)
3. [Arquitectura general](#3-arquitectura-general)
4. [Componentes del sistema](#4-componentes-del-sistema)
   - [4.1 Script de visión (Python)](#41-script-de-visión-pythoncontrol_esp32py)
   - [4.2 Firmware del microcontrolador (ESP32)](#42-firmware-del-microcontrolador-esp32esp32ino)
   - [4.3 Script de control de servo (Python)](#43-script-de-control-de-servo-pythoncontrolservopy)
   - [4.4 Interfaz gráfica (Python/Tkinter)](#44-interfaz-gráfica-pythontkinterinterfazpy)
5. [¿Cómo funciona paso a paso?](#5-cómo-funciona-paso-a-paso)
6. [Sistema de paro de emergencia (E-Stop)](#6-sistema-de-paro-de-emergencia-e-stop)
7. [Registro de eventos (CSV)](#7-registro-de-eventos-csv)
8. [Tecnologías utilizadas](#8-tecnologías-utilizadas)
9. [Configuración de red y hardware](#9-configuración-de-red-y-hardware)
   - [9.1 Red](#91-red)
   - [9.2 Pines GPIO del ESP32](#92-pines-gpio-del-esp32)
   - [9.3 Parámetros de visión](#93-parámetros-de-visión)
10. [Estructura de archivos](#10-estructura-de-archivos)
11. [Flujo de datos](#11-flujo-de-datos)
12. [Requisitos para ejecutar el proyecto](#12-requisitos-para-ejecutar-el-proyecto)
13. [Limitaciones actuales y posibles mejoras](#13-limitaciones-actuales-y-posibles-mejoras)

---

## 1. ¿Qué es el proyecto?

**Vision-Integrador** es un sistema de control para un robot AGV (Automated Guided Vehicle) seguidor de línea con configuración Ackerman que integra:

- **Visión artificial** en tiempo real (procesada en una Raspberry Pi 4B con Python).
- **Comunicación inalámbrica** mediante el protocolo MQTT (broker en la misma Raspberry Pi).
- **Control de hardware físico** a través de un microcontrolador ESP32.
- **Lectura de sensores** (potenciómetro y sensor de temperatura LM35).
- **Sistema de arranque/paro/emergencia** mediante botones físicos, comandos remotos MQTT e interfaz gráfica.
- **Interfaz gráfica de monitoreo** con focos de estado, gráfica de temperatura, barra de potenciómetro y botones de control virtual.
- **Registro de eventos** en archivo CSV para trazabilidad de operaciones.

En términos simples: una cámara detecta hacia dónde se mueve una línea azul respecto a una **zona segura central**, y el sistema mueve o activa dispositivos físicos (LEDs, motores) para corregir la dirección. Cuando la línea está dentro de la zona segura, ambos motores avanzan simultáneamente.

---

## 2. ¿Para qué sirve?

El sistema puede aplicarse en escenarios como:

| Aplicación | Descripción |
|---|---|
| AGV seguidor de línea | Robot con dirección Ackerman que sigue una línea azul corrigiendo dirección |
| Robótica | Controlar un robot que sigue un objeto azul con la vista |
| Automatización | Activar actuadores según la posición de un objeto en cámara |
| Educación | Aprender integración de visión artificial + IoT + microcontroladores |
| Prototipado | Base para sistemas de seguimiento más complejos |

---

## 3. Arquitectura general

El proyecto sigue un modelo **publicador–suscriptor** (pub/sub) basado en MQTT:

```
┌──────────────────────────────────────────────────────────────────┐
│                    RASPBERRY PI 4B                                │
│                                                                  │
│   ┌──────────────┐      ┌───────────────────────────────────┐   │
│   │   Cámara USB │ ───► │  control_esp32.py (Python/OpenCV) │   │
│   │  /dev/video0 │      │  Detección de línea azul          │   │
│   └──────────────┘      │  Zona segura central (30% frame)  │   │
│                         │  Calcula posición: IZQ, DER o     │   │
│                         │  CENTRO                           │   │
│                         │  Publica mensaje MQTT             │   │
│                         └───────────────┬───────────────────┘   │
│                                         │                        │
│   ┌─────────────────────────────────────┼───────────────────┐   │
│   │  interfaz.py (Tkinter)             │                    │   │
│   │  Focos: PARO/MOVIMIENTO/EMERGENCIA │                    │   │
│   │  Botones: ARRANQUE/PARO/E-STOP     │                    │   │
│   │  Gráfica temperatura en tiempo real│                    │   │
│   │  Barra de potenciómetro            │                    │   │
│   │  Registro de eventos → eventos.csv │                    │   │
│   └─────────────────────────────────────┼───────────────────┘   │
│                                         │                        │
│   ┌─────────────────────────────────────┴───────────────────┐   │
│   │           Broker MQTT (Mosquitto)                       │   │
│   │           Topics: esp32/control, esp32/arranque,        │   │
│   │           pot/uno, LM35/uno, arranque/paro              │   │
│   └─────────────────────────────────────┬───────────────────┘   │
└──────────────────────────────────────────┼───────────────────────┘
                                           │ MQTT (WiFi)
                                           ▼
                         ┌──────────────────────────────┐
                         │       ESP32 (Firmware)       │
                         │                              │
                         │  Recibe: "IZQ","DER","CENTRO"│
                         │  Recibe: "GO","STOP",        │
                         │   "EMERGENCIA","RESET_E..."  │
                         │  Botones físicos ON/OFF      │
                         │  E-Stop físico (NC, GPIO 13) │
                         │                              │
                         │  GPIO 16 ── LED Izquierdo    │
                         │  GPIO 15 ── LED Derecho      │
                         │  GPIO 17 ── Motor Izquierdo  │
                         │  GPIO 0  ── Motor Derecho    │
                         │  GPIO 2  ── LED Estado (ON)  │
                         │  GPIO 23 ── LED Estado (PARO)│
                         │  GPIO 18 ── Botón Encender   │
                         │  GPIO 4  ── Botón Apagar     │
                         │  GPIO 13 ── E-Stop (NC)      │
                         │  GPIO 34 ── Potenciómetro    │
                         │  GPIO 36 ── Sensor LM35      │
                         └──────────────────────────────┘
```

---

## 4. Componentes del sistema

### 4.1 Script de visión (Python) — `control_esp32.py`

**Ubicación:** `/control_esp32.py`
**Lenguaje:** Python 3
**Dependencias:** `opencv-python`, `paho-mqtt`, `numpy`

Este script es el cerebro del sistema. Se ejecuta en la Raspberry Pi y realiza las siguientes tareas en un bucle continuo:

#### ¿Qué hace?

1. **Captura video** desde la cámara conectada al sistema (`/dev/video0`).
2. **Convierte** cada fotograma del espacio de color BGR al espacio **HSV**.
3. **Aplica una máscara** de color para aislar únicamente los píxeles azules dentro del rango HSV definido:
   - Tono (H): 100 a 140
   - Saturación (S): 150 a 255
   - Valor (V): 0 a 255
4. **Detecta contornos** en la máscara y descarta los que tienen área menor a 500 píxeles.
5. **Calcula el centroide** del objeto azul encontrado.
6. **Define una zona segura central** (30% del ancho del frame, 15% a cada lado del centro) dibujada como un rectángulo verde semitransparente.
7. **Determina la posición** del centroide respecto a la zona segura:
   - Izquierda de la zona segura → `"IZQ"`
   - Derecha de la zona segura → `"DER"`
   - Dentro de la zona segura → `"CENTRO"`
8. **Publica el comando** al broker MQTT en el topic `esp32/control`, solo si el comando cambió.
9. **Dibuja** sobre el fotograma: zona segura, línea central, rectángulo del objeto, centroide y texto del comando.
10. **Muestra** dos ventanas: el video en tiempo real y la máscara de color azul.

---

### 4.2 Firmware del microcontrolador (ESP32) — `esp32/esp32.ino`

**Ubicación:** `/esp32/esp32.ino`
**Lenguaje:** C++ (Arduino)
**Librerías:** `WiFi.h`, `PubSubClient.h`

El firmware convierte los mensajes MQTT en acciones físicas.

#### ¿Qué hace?

1. **Conecta al WiFi** y al **broker MQTT**.
2. **Se suscribe** a los topics `esp32/control` y `esp32/arranque`.
3. **Sistema de arranque/paro** con tres modos de control:
   - **Botones físicos**: ON (GPIO 18) y OFF (GPIO 4) con debounce de 50ms.
   - **Comandos MQTT remotos**: `"GO"`, `"STOP"`, `"EMERGENCIA"`, `"RESET_EMERGENCIA"`.
   - **E-Stop físico**: Botón NC en GPIO 13 con interrupción de hardware.
4. **Control de dirección** (solo cuando `ledState == true`):
   - `"CENTRO"`: ambos motores encendidos.
   - `"IZQ"`: solo motor izquierdo.
   - `"DER"`: solo motor derecho.
5. **Paro de emergencia** (físico y remoto):
   - Interrupción ISR apaga motores inmediatamente.
   - Bloquea el sistema hasta reset manual.
   - El E-Stop remoto (`"EMERGENCIA"`) usa un flag `flag` para distinguirlo del físico.
   - `"RESET_EMERGENCIA"` solo funciona si el botón físico no está presionado.
6. **Lectura de sensores** cada segundo: potenciómetro (GPIO 34) y LM35 (GPIO 36).
7. **Publica estado** (`"MOVIMIENTO"`, `"PARO"`, `"EMERGENCIA"`) solo cuando cambia.
8. **Reconexión automática** WiFi y MQTT.

---

### 4.3 Script de control de servo (Python) — `ControlServo.py`

**Ubicación:** `/ControlServo.py`
**Lenguaje:** Python 3
**Dependencias:** `paho-mqtt`

Script auxiliar para controlar el servo motor de la dirección Ackerman de forma manual mediante MQTT. Envía ángulos (0-180) al topic `esp32/servo/control`.

---

### 4.4 Interfaz gráfica (Python/Tkinter) — `interfaz.py`

**Ubicación:** `/interfaz.py`
**Lenguaje:** Python 3
**Dependencias:** `tkinter` (incluido en Python), `paho-mqtt`

Panel de control y monitoreo del AGV con tema oscuro.

#### ¿Qué tiene?

1. **3 focos indicadores de estado:**
   - Rojo: PARO
   - Verde: MOVIMIENTO
   - Amarillo: EMERGENCIA
   - Solo uno encendido a la vez, con label del estado actual.

2. **4 botones de control:**
   - **ARRANQUE** (verde): publica `"GO"` al topic `esp32/arranque`.
   - **PARO** (rojo): publica `"STOP"`.
   - **E-STOP** (amarillo): publica `"EMERGENCIA"` (mismo efecto que el botón físico NC).
   - **RESET E-STOP** (gris): publica `"RESET_EMERGENCIA"` para permitir rearranque.

3. **Gráfica de temperatura en tiempo real:**
   - 60 segundos de historial con línea suavizada.
   - Cuadrícula con etiquetas cada 10 C.
   - Línea roja de alerta en 40 C.
   - Punto indicador en el último valor.

4. **Barra de nivel del potenciómetro:**
   - Representación visual de 0 a 3.3V.
   - Marca de referencia en 0.5V.
   - Cambia a rojo cuando el voltaje baja de 0.5V.

5. **Sistema de alertas:**
   - Temperatura > 40 C: alerta de temperatura crítica.
   - Potenciómetro < 0.5V: alerta de voltaje bajo.
   - Solo se muestra una vez hasta que el valor vuelva a rango normal.
   - Las alertas también se registran en el CSV.

6. **Registro de eventos en CSV** (ver sección 7).

---

## 5. ¿Cómo funciona paso a paso?

```
Paso 1: El usuario enciende el ESP32 (ya flasheado con el firmware)
           └─► Se conecta al WiFi y al broker MQTT
           └─► Se suscribe a "esp32/control" y "esp32/arranque"
           └─► LED de paro (GPIO 23) encendido, LED de estado (GPIO 2) apagado
           └─► Verifica estado inicial del E-Stop

Paso 2: El usuario ejecuta la interfaz gráfica en la Raspberry Pi
           └─► Se conecta al broker MQTT
           └─► Suscribe a "arranque/paro", "LM35/uno", "pot/uno"
           └─► Inicializa el archivo eventos.csv

Paso 3: El usuario arranca el sistema (botón físico ON, o ARRANQUE en interfaz)
           └─► ledState = true
           └─► Publica "MOVIMIENTO" al topic "arranque/paro"
           └─► Se registra el evento en eventos.csv con origen FISICA o VIRTUAL

Paso 4: Bucle de visión (control_esp32.py)
           └─► Captura fotograma → HSV → máscara azul → contornos
           └─► Determina posición: IZQ / DER / CENTRO
           └─► Publica por MQTT si cambió

Paso 5: El ESP32 recibe y actúa según dirección

Paso 6: Cada segundo, el ESP32 publica datos de sensores
           └─► pot/uno y LM35/uno
           └─► La interfaz actualiza gráfica, barra y verifica alertas

Paso 7: Ante emergencia (botón NC físico o E-STOP en interfaz)
           └─► Motores se apagan inmediatamente
           └─► Se publica "EMERGENCIA"
           └─► Se registra en eventos.csv
           └─► Sistema bloqueado hasta reset manual
```

---

## 6. Sistema de paro de emergencia (E-Stop)

El sistema cuenta con paro de emergencia dual (físico y virtual):

### E-Stop físico (GPIO 13)

- **Botón normalmente cerrado (NC)**: en operación normal el circuito está cerrado (GPIO lee LOW).
- **Al presionar o si el cable se rompe**: el circuito se abre (GPIO lee HIGH) → **fail-safe**.
- Usa **interrupción de hardware** (`attachInterrupt`, `RISING`) para respuesta en microsegundos.
- La ISR (`isrEmergencia`) apaga motores y LEDs directamente.

### E-Stop virtual (interfaz gráfica)

- El botón **E-STOP** de la interfaz publica `"EMERGENCIA"` al topic `esp32/arranque`.
- El ESP32 activa el flag `emergencia` y el flag `flag` (para distinguir de emergencia física).
- Mismo efecto que el botón físico: apaga todo, bloquea el sistema.

### Reset de emergencia

- **E-Stop físico**: al soltar el botón (NC vuelve a cerrar), el flag se limpia pero NO reanuda automáticamente. Requiere presionar ON o enviar GO.
- **E-Stop virtual**: requiere enviar `"RESET_EMERGENCIA"` Y que el botón físico no esté presionado.

```
                    ┌─────────────────────────┐
                    │   ESTADO: OPERANDO      │
                    └──────────┬──────────────┘
                               │
              ┌────────────────┼────────────────┐
              │                │                │
     [Botón NC físico]  [E-STOP interfaz]  [Cable roto]
     GPIO 13 → HIGH    MQTT "EMERGENCIA"   GPIO 13 → HIGH
              │                │                │
              └────────────────┼────────────────┘
                               │
                               ▼
                    ┌─────────────────────────┐
                    │   ESTADO: EMERGENCIA    │
                    │   - Motores OFF         │
                    │   - Sistema bloqueado   │
                    │   - Registrado en CSV   │
                    └──────────┬──────────────┘
                               │
              ┌────────────────┼────────────────┐
              │                                 │
     [Soltar botón NC]              [RESET_EMERGENCIA]
     (limpia flag, no reanuda)      (solo si NC cerrado)
              │                                 │
              └────────────────┼────────────────┘
                               │
                               ▼
                    ┌─────────────────────────┐
                    │   ESTADO: PARO          │
                    │   Esperando ON o GO     │
                    └─────────────────────────┘
```

---

## 7. Registro de eventos (CSV)

La interfaz gráfica registra automáticamente todos los eventos del sistema en el archivo `eventos.csv`.

### Formato del archivo

| Columna | Descripción |
|---|---|
| `Fecha` | Fecha del evento (YYYY-MM-DD) |
| `Hora` | Hora del evento (HH:MM:SS) |
| `arranque/paro` | Tipo de evento: `MOVIMIENTO`, `PARO`, `EMERGENCIA`, `ALERTA_TEMP`, `ALERTA_POT` |
| `Origen` | Fuente del evento: `VIRTUAL` (interfaz), `FISICA` (botones ESP32), `SENSOR` (alertas) |
| `LM35/uno` | Temperatura al momento del evento (C) |
| `pot/uno` | Voltaje del potenciómetro al momento del evento (V) |

### Ejemplo de archivo generado

```csv
Fecha,Hora,arranque/paro,Origen,LM35/uno,pot/uno
2026-03-16,10:15:30,MOVIMIENTO,VIRTUAL,24.5,1.650
2026-03-16,10:16:45,PARO,FISICA,25.1,1.700
2026-03-16,10:17:00,MOVIMIENTO,FISICA,25.3,1.680
2026-03-16,10:20:12,ALERTA_TEMP,SENSOR,41.2,1.500
2026-03-16,10:22:05,EMERGENCIA,VIRTUAL,42.0,1.450
2026-03-16,10:23:00,ALERTA_POT,SENSOR,38.5,0.350
```

### Eventos registrados

| Evento | Cuándo se registra |
|---|---|
| `MOVIMIENTO` | El sistema pasa de paro/emergencia a movimiento |
| `PARO` | El sistema pasa de movimiento a paro |
| `EMERGENCIA` | Se activa el paro de emergencia (físico o virtual) |
| `ALERTA_TEMP` | La temperatura supera 40 C (una vez hasta que baje) |
| `ALERTA_POT` | El voltaje del potenciómetro baja de 0.5V (una vez hasta que suba) |

### Detección del origen

- **VIRTUAL**: se activa el flag `origen_ultimo_comando` cuando se presiona un botón de la interfaz. Si el siguiente cambio de estado coincide con el flag, se registra como VIRTUAL.
- **FISICA**: si el cambio de estado llega por MQTT sin que la interfaz haya enviado un comando, se registra como FISICA (botones del ESP32 o E-Stop NC).
- **SENSOR**: para alertas de temperatura y potenciómetro.

---

## 8. Tecnologías utilizadas

| Tecnología | Rol | Versión recomendada |
|---|---|---|
| **Python 3** | Lenguaje de visión, interfaz y control | 3.8+ |
| **OpenCV (`cv2`)** | Captura de video, procesamiento de imagen | 4.x |
| **NumPy** | Manejo de arrays para máscaras de color | 1.x |
| **Tkinter** | Interfaz gráfica de monitoreo y control | incluido en Python |
| **paho-mqtt** | Cliente MQTT para Python | 1.6+ |
| **MQTT (protocolo)** | Comunicación ligera publicador-suscriptor | v3.1.1 |
| **Mosquitto** | Broker MQTT (en Raspberry Pi) | 2.x |
| **Raspberry Pi 4B** | Ejecuta visión, interfaz y broker MQTT | — |
| **ESP32** | Microcontrolador con WiFi integrado | — |
| **Arduino IDE** | Entorno de desarrollo para el firmware | 2.x |
| **PubSubClient** | Librería MQTT para Arduino/ESP32 | 2.8+ |
| **WiFi.h** | Librería WiFi nativa del ESP32 | nativa |

---

## 9. Configuración de red y hardware

### 9.1 Red

> **Nota:** Estas configuraciones están hardcodeadas en los archivos. Para cambiarlas hay que editarlas directamente en el código fuente.

| Parámetro | Valor actual |
|---|---|
| SSID WiFi | `JOSUE's Galaxy A52` |
| Contraseña WiFi | `jog18030` |
| IP del broker MQTT | `10.218.99.191` |
| Puerto MQTT | `1883` |
| Topic de control de dirección | `esp32/control` |
| Topic de arranque remoto | `esp32/arranque` |
| Topic de servo | `esp32/servo/control` |
| Topic de potenciómetro | `pot/uno` |
| Topic de temperatura | `LM35/uno` |
| Topic de estado arranque/paro | `arranque/paro` |

### 9.2 Pines GPIO del ESP32

| Pin GPIO | Nombre en código | Función |
|---|---|---|
| GPIO 16 | `ledizq` | LED indicador izquierdo |
| GPIO 17 | `izq` | Motor izquierdo |
| GPIO 15 | `ledder` | LED indicador derecho |
| GPIO 0 | `der` | Motor derecho |
| GPIO 2 | `PIN_LED` | LED de estado encendido |
| GPIO 23 | `PIN_LEDPARO` | LED de estado en paro |
| GPIO 18 | `PIN_BOTON_ON` | Botón de encendido (INPUT_PULLUP) |
| GPIO 4 | `PIN_BOTON_OFF` | Botón de apagado (INPUT_PULLUP) |
| GPIO 13 | `PIN_ESTOP` | Botón E-Stop NC (INPUT_PULLUP, interrupción RISING) |
| GPIO 34 | `pinPot1` | Lectura de potenciómetro (ADC) |
| GPIO 36 | `pinLM35` | Lectura de sensor LM35 (ADC) |

> **Advertencia:** GPIO 0 en el ESP32 es un pin especial (boot). Usarlo como salida puede causar problemas durante el arranque del dispositivo.

### 9.3 Parámetros de visión

| Parámetro | Valor | Descripción |
|---|---|---|
| Índice de cámara | `0` | Primera cámara detectada por el sistema |
| Rango HSV mínimo | `[100, 150, 0]` | Límite inferior del azul |
| Rango HSV máximo | `[140, 255, 255]` | Límite superior del azul |
| Área mínima del objeto | `500 px` | Filtra ruido pequeño |
| Margen zona segura | `15%` | 15% del ancho a cada lado del centro (zona total = 30%) |
| Delay por fotograma | `1 ms` | `cv2.waitKey(1)` — ESC para salir |

---

## 10. Estructura de archivos

```
Vision-Integrador/
│
├── control_esp32.py        # Script principal de visión + MQTT (Python)
├── ControlServo.py         # Script de control manual de servo por MQTT (Python)
├── interfaz.py             # Interfaz gráfica de monitoreo y control (Tkinter)
├── eventos.csv             # Registro de eventos del sistema (generado automáticamente)
│
├── esp32/
│   └── esp32.ino           # Firmware para el ESP32 (C++ / Arduino)
│
└── DOCUMENTACION.md        # Este archivo
```

---

## 11. Flujo de datos

```
[Cámara] → [Fotograma BGR]
                │
                ▼
         [Conversión HSV]
                │
                ▼
      [Máscara de color azul]
                │
                ▼
      [Detección de contornos]
                │
          ¿Hay objeto?
         /           \
        No            Sí
        │              │
   (no envía)    [Calcula centroide]
                        │
               ¿Posición respecto a zona segura?
              /         |          \
         Izquierda   Centro     Derecha
             │          │          │
          "IZQ"     "CENTRO"    "DER"
             │          │          │
             └──────────┼──────────┘
                        │
               ¿Comando cambió?
              /              \
             No               Sí
          (ignora)     [Publica MQTT]
                              │
                    [Broker retransmite]
                              │
                       [ESP32 recibe]
                              │
                  ¿Emergencia activa?
                  /                \
                Sí                 No
          (ignora todo)    ¿ledState == true?
                           /              \
                          No              Sí
                    (ignora todo)    ¿Qué mensaje?
                                  /     |      \
                              "IZQ" "CENTRO"  "DER"
                                │      │        │
                           Solo izq  Ambos  Solo der
                           motores  motores  motores

               [Paralelamente cada segundo]
                          │
              ┌───────────┼───────────┐
              │           │           │
        [Publica      [Publica    [Estado solo
        voltaje       temp LM35   al cambiar]
        pot/uno]      LM35/uno]   arranque/paro

               [Interfaz recibe todo]
                          │
              ┌───────────┼───────────┐
              │           │           │
        [Actualiza    [Grafica    [Registra
        focos y       temp en     evento en
        barra pot]    tiempo      eventos.csv]
                      real]
```

---

## 12. Requisitos para ejecutar el proyecto

### En la Raspberry Pi 4B (visión, interfaz y broker)

```bash
# Instalar broker MQTT
sudo apt install mosquitto mosquitto-clients

# Iniciar broker
sudo systemctl enable mosquitto
sudo systemctl start mosquitto

# Instalar dependencias de Python
pip install opencv-python paho-mqtt numpy

# Ejecutar el script de visión
python3 control_esp32.py

# Ejecutar la interfaz gráfica (en otra terminal)
python3 interfaz.py

# (Opcional) Ejecutar el control de servo
python3 ControlServo.py
```

### En el ESP32 (firmware)

1. Instalar el **IDE de Arduino** (versión 2.x recomendada).
2. Agregar soporte para **ESP32** en el gestor de placas.
3. Instalar la librería **PubSubClient** (versión 2.8+).
4. Abrir `esp32/esp32.ino`.
5. Seleccionar la placa correcta (ej. *ESP32 Dev Module*).
6. Compilar y cargar al ESP32.

---

## 13. Limitaciones actuales y posibles mejoras

### Limitaciones

| Limitación | Descripción |
|---|---|
| Color fijo | Solo detecta el color azul; no configurable en tiempo real |
| Credenciales hardcodeadas | WiFi, IP del broker y contraseñas están en el código fuente |
| Sin manejo de errores de cámara | Si la cámara falla, el script se rompe sin aviso claro |
| GPIO 0 como salida | Pin especial del ESP32, puede causar problemas en el arranque |
| Un solo objeto | Solo procesa el primer contorno > 500 px; no maneja múltiples objetos |

### Posibles mejoras

- [ ] Implementar control por lógica difusa al algoritmo de visión
- [ ] Integrar el control de servo con el sistema de visión para dirección Ackerman
- [ ] Mover la configuración a un archivo `config.json`
- [ ] Permitir seleccionar el color a seguir desde la interfaz
- [ ] Implementar reconexión automática en los scripts Python
- [ ] Proteger la comunicación MQTT con TLS y autenticación

---

*Documentación actualizada el 2026-03-16*
