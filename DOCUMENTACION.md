# Vision-Integrador — Documentación del Proyecto

> **Estado:** Prototipo funcional
> **Versión:** 2.0 (commit `Botones2`)
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
5. [¿Cómo funciona paso a paso?](#5-cómo-funciona-paso-a-paso)
6. [Tecnologías utilizadas](#6-tecnologías-utilizadas)
7. [Configuración de red y hardware](#7-configuración-de-red-y-hardware)
   - [7.1 Red](#71-red)
   - [7.2 Pines GPIO del ESP32](#72-pines-gpio-del-esp32)
   - [7.3 Parámetros de visión](#73-parámetros-de-visión)
8. [Estructura de archivos](#8-estructura-de-archivos)
9. [Flujo de datos](#9-flujo-de-datos)
10. [Requisitos para ejecutar el proyecto](#10-requisitos-para-ejecutar-el-proyecto)
11. [Limitaciones actuales y posibles mejoras](#11-limitaciones-actuales-y-posibles-mejoras)

---

## 1. ¿Qué es el proyecto?

**Vision-Integrador** es un sistema de seguimiento de objetos por color que integra:

- **Visión artificial** en tiempo real (procesada en una computadora o laptop con Python).
- **Comunicación inalámbrica** mediante el protocolo MQTT.
- **Control de hardware físico** a través de un microcontrolador ESP32.
- **Lectura de sensores** (potenciómetro y sensor de temperatura LM35).
- **Sistema de arranque/paro** mediante botones físicos y comandos remotos MQTT.

En términos simples: una cámara detecta hacia dónde se mueve un objeto de color azul respecto a una **zona segura central**, y el sistema mueve o activa dispositivos físicos (LEDs, motores) en la dirección correspondiente. Cuando el objeto está dentro de la zona segura, ambos motores avanzan simultáneamente.

---

## 2. ¿Para qué sirve?

El sistema puede aplicarse en escenarios como:

| Aplicación | Descripción |
|---|---|
| Seguidor de línea | Controlar un robot que sigue una línea azul corrigiendo dirección |
| Robótica | Controlar un robot que sigue un objeto azul con la vista |
| Automatización | Activar actuadores según la posición de un objeto en cámara |
| Educación | Aprender integración de visión artificial + IoT + microcontroladores |
| Prototipado | Base para sistemas de seguimiento más complejos |

---

## 3. Arquitectura general

El proyecto sigue un modelo **publicador–suscriptor** (pub/sub) basado en MQTT:

```
┌──────────────────────────────────────────────────────────────────┐
│                         COMPUTADORA (PC/Laptop)                  │
│                                                                  │
│   ┌──────────────┐      ┌───────────────────────────────────┐   │
│   │   Cámara USB │ ───► │  control_esp32.py (Python/OpenCV) │   │
│   │  /dev/video0 │      │  Detección de objeto azul         │   │
│   └──────────────┘      │  Zona segura central (30% frame)  │   │
│                         │  Calcula posición: IZQ, DER o     │   │
│                         │  CENTRO                           │   │
│                         │  Publica mensaje MQTT             │   │
│                         └───────────────┬───────────────────┘   │
└───────────────────────────────────────────────────────────────── ┘
                                          │ MQTT
                                          ▼
                         ┌────────────────────────────┐
                         │   Broker MQTT (Mosquitto)  │
                         │   IP: 192.168.1.82:1883    │
                         │   Topics:                  │
                         │     esp32/control           │
                         │     esp32/arranque          │
                         │     pot/uno                 │
                         │     LM35/uno                │
                         │     arranque/paro           │
                         └──────────────┬─────────────┘
                                        │ MQTT (WiFi)
                                        ▼
                         ┌──────────────────────────────┐
                         │       ESP32 (Firmware)       │
                         │                              │
                         │  Recibe: "IZQ","DER","CENTRO"│
                         │  Recibe: "GO", "STOP"        │
                         │  Botones físicos ON/OFF      │
                         │                              │
                         │  GPIO 16 ── LED Izquierdo    │
                         │  GPIO 15 ── LED Derecho      │
                         │  GPIO 17 ── Motor Izquierdo  │
                         │  GPIO 0  ── Motor Derecho    │
                         │  GPIO 2  ── LED Estado (ON)  │
                         │  GPIO 23 ── LED Estado (PARO)│
                         │  GPIO 18 ── Botón Encender   │
                         │  GPIO 4  ── Botón Apagar     │
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

Este script es el cerebro del sistema. Se ejecuta en la computadora y realiza las siguientes tareas en un bucle continuo:

#### ¿Qué hace?

1. **Captura video** desde la cámara conectada al sistema (`/dev/video0`).
2. **Convierte** cada fotograma del espacio de color BGR (cómo OpenCV lee cámaras) al espacio **HSV** (Hue-Saturation-Value), que es más fácil para filtrar colores.
3. **Aplica una máscara** de color para aislar únicamente los píxeles azules dentro del rango HSV definido:
   - Tono (H): 100 a 140 → rango del azul en HSV
   - Saturación (S): 150 a 255 → colores vivos, no blancos/grises
   - Valor (V): 0 a 255 → cualquier brillo
4. **Detecta contornos** en la máscara y descarta los que tienen área menor a 500 píxeles (para evitar ruido).
5. **Calcula el centroide** (centro de masa) del objeto azul encontrado.
6. **Define una zona segura central** (30% del ancho del frame, 15% a cada lado del centro) dibujada como un rectángulo verde semitransparente.
7. **Determina la posición** del centroide respecto a la zona segura:
   - Si el centroide está a la **izquierda** de la zona segura → envía `"IZQ"`
   - Si está a la **derecha** de la zona segura → envía `"DER"`
   - Si está **dentro** de la zona segura → envía `"CENTRO"`
8. **Publica el comando** al broker MQTT en el topic `esp32/control`, pero **solo si el comando cambió** respecto al anterior (para no saturar el broker con mensajes repetidos).
9. **Dibuja** sobre el fotograma:
   - Zona segura con fondo semitransparente verde y líneas de borde
   - Línea central tenue de referencia
   - Un rectángulo alrededor del objeto detectado
   - Un punto en el centroide
   - El texto del comando actual (verde para CENTRO, naranja para IZQ/DER)
10. **Muestra** dos ventanas: el video en tiempo real y la máscara de color azul.

#### Detalle del código clave

```python
# Margen de la zona segura: 15% del ancho a cada lado del centro
MARGEN_ZONA_SEGURA = 0.15

# Límites de la zona segura
zona_izq = int(width // 2 - width * MARGEN_ZONA_SEGURA)
zona_der = int(width // 2 + width * MARGEN_ZONA_SEGURA)

# Rango de color azul en HSV
lower_blue = np.array([100, 150, 0])
upper_blue = np.array([140, 255, 255])

# Máscara: solo píxeles dentro del rango azul
mask = cv2.inRange(hsv, lower_blue, upper_blue)

# Determinar dirección respecto a la zona segura
if cx < zona_izq:
    posicion_actual = "IZQ"
elif cx > zona_der:
    posicion_actual = "DER"
else:
    posicion_actual = "CENTRO"
```

---

### 4.2 Firmware del microcontrolador (ESP32) — `esp32/esp32.ino`

**Ubicación:** `/esp32/esp32.ino`
**Lenguaje:** C++ (Arduino)
**Librerías:** `WiFi.h`, `PubSubClient.h`

El firmware convierte los mensajes MQTT en acciones físicas. Se flashea directamente en el ESP32 mediante el IDE de Arduino.

#### ¿Qué hace?

1. **Conecta al WiFi** usando las credenciales configuradas (SSID y contraseña hardcodeados).
2. **Se conecta al broker MQTT** en la IP definida.
3. **Se suscribe** a los topics `esp32/control` (dirección) y `esp32/arranque` (encendido/apagado remoto).
4. **Sistema de arranque/paro** con dos modos de control:
   - **Botones físicos**: Botón ON (GPIO 18) y botón OFF (GPIO 4) con lógica de debounce (50ms) para evitar rebotes.
   - **Comandos MQTT remotos**: `"GO"` para encender y `"STOP"` para apagar, recibidos en el topic `esp32/arranque`.
   - LED indicador de estado: GPIO 2 (encendido) y GPIO 23 (paro).
5. **Control de dirección** (solo cuando el sistema está encendido - `ledState == true`):
   - `"CENTRO"`: activa ambos lados (GPIO 16, 15, 17, 0 → HIGH). Ambos motores avanzan.
   - `"IZQ"`: activa solo el lado izquierdo (GPIO 16, 17 → HIGH; GPIO 15, 0 → LOW).
   - `"DER"`: activa solo el lado derecho (GPIO 15, 0 → HIGH; GPIO 16, 17 → LOW).
   - Si el sistema está apagado, todos los motores y LEDs se desactivan.
6. **Lectura de sensores** cada segundo:
   - **Potenciómetro** (GPIO 34): lee el valor ADC, lo convierte a voltaje (0-3.3V) y lo publica en el topic `pot/uno`.
   - **Sensor de temperatura LM35** (GPIO 36): lee el valor ADC, lo convierte a temperatura y lo publica en el topic `LM35/uno`.
7. **Publica estado de operación** cada segundo al topic `arranque/paro` con valores `"MOVIMIENTO"` o `"PARO"`.
8. **Reconexión automática**: si se pierde la conexión WiFi o MQTT, el firmware intenta reconectarse automáticamente.

#### Detalle del código clave

```cpp
// Pines de dirección
const int ledizq = 16;  // LED izquierdo
const int ledder = 15;  // LED derecho
const int izq    = 17;  // Motor izquierdo
const int der    = 0;   // Motor derecho

// Pines de arranque/paro
const int PIN_BOTON_OFF = 4;   // Botón apagar
const int PIN_BOTON_ON  = 18;  // Botón encender
const int PIN_LED       = 2;   // LED estado encendido
const int PIN_LEDPARO   = 23;  // LED estado paro

// Pines de sensores
const int pinPot1 = 34;  // Potenciómetro
const int pinLM35 = 36;  // Sensor LM35

// Callback al recibir mensaje MQTT
void callback(char* topic, byte* message, unsigned int length) {
  // ...
  if(ledState) {
    if (messageTemp == "CENTRO") {  // Zona segura: ambos motores encendidos
      digitalWrite(ledizq, HIGH); digitalWrite(ledder, HIGH);
      digitalWrite(izq, HIGH);    digitalWrite(der, HIGH);
    }
    if (messageTemp == "IZQ") {     // Corregir hacia la izquierda
      digitalWrite(ledizq, HIGH);  digitalWrite(ledder, LOW);
      digitalWrite(izq, HIGH);     digitalWrite(der, LOW);
    }
    if (messageTemp == "DER") {     // Corregir hacia la derecha
      digitalWrite(ledder, HIGH);  digitalWrite(ledizq, LOW);
      digitalWrite(der, HIGH);     digitalWrite(izq, LOW);
    }
  }

  if (messageTemp == "STOP") { ledState = false; /* apagar todo */ }
  if (messageTemp == "GO")   { ledState = true;  /* encender */   }
}
```

---

### 4.3 Script de control de servo (Python) — `ControlServo.py`

**Ubicación:** `/ControlServo.py`
**Lenguaje:** Python 3
**Dependencias:** `paho-mqtt`

Script auxiliar para controlar un servo motor de forma manual mediante MQTT.

#### ¿Qué hace?

1. Se conecta al broker MQTT en `10.91.115.191:1883`.
2. Solicita al usuario un ángulo (0-180) por consola.
3. Publica el ángulo en el topic `esp32/servo/control`.
4. Permite enviar múltiples ángulos hasta que el usuario escriba `"salir"`.

---

## 5. ¿Cómo funciona paso a paso?

```
Paso 1: El usuario enciende el ESP32 (ya flasheado con el firmware)
           └─► El ESP32 se conecta al WiFi y al broker MQTT
           └─► Se suscribe a los topics "esp32/control" y "esp32/arranque"
           └─► LED de paro (GPIO 23) encendido, LED de estado (GPIO 2) apagado

Paso 2: El usuario presiona el botón ON (GPIO 18) o envía "GO" por MQTT
           └─► ledState = true
           └─► LED de estado (GPIO 2) se enciende
           └─► LED de paro (GPIO 23) se apaga
           └─► El sistema comienza a responder a comandos de dirección

Paso 3: El usuario ejecuta el script Python en la computadora
           └─► Se abre la cámara USB
           └─► Se conecta al mismo broker MQTT

Paso 4: Bucle de visión (se repite continuamente)
           └─► Captura fotograma de la cámara
           └─► Convierte BGR → HSV
           └─► Aplica máscara de color azul
           └─► Dibuja zona segura central (30% del frame)
           └─► Detecta contornos
           └─► Si hay un objeto azul > 500 px:
                   └─► Calcula su centroide (cx, cy)
                   └─► Si cx < zona_izq → comando = "IZQ"
                   └─► Si cx > zona_der → comando = "DER"
                   └─► Si zona_izq ≤ cx ≤ zona_der → comando = "CENTRO"
                   └─► Si el comando cambió → publica por MQTT

Paso 5: El broker MQTT reenvía el mensaje al ESP32

Paso 6: El ESP32 recibe "IZQ", "DER" o "CENTRO"
           └─► Solo actúa si ledState == true (sistema encendido)
           └─► Activa los pines GPIO correspondientes

Paso 7: Cada segundo, el ESP32 publica datos de sensores
           └─► Voltaje del potenciómetro → topic "pot/uno"
           └─► Temperatura del LM35 → topic "LM35/uno"
           └─► Estado arranque/paro → topic "arranque/paro"
```

---

## 6. Tecnologías utilizadas

| Tecnología | Rol | Versión recomendada |
|---|---|---|
| **Python 3** | Lenguaje del script de visión | 3.8+ |
| **OpenCV (`cv2`)** | Captura de video, procesamiento de imagen, detección de color | 4.x |
| **NumPy** | Manejo de arrays para máscaras de color | 1.x |
| **paho-mqtt** | Cliente MQTT para Python | 1.6+ |
| **MQTT (protocolo)** | Comunicación ligera publicador–suscriptor | v3.1.1 |
| **Mosquitto** | Broker MQTT (servidor de mensajes) | 2.x |
| **ESP32** | Microcontrolador con WiFi integrado | — |
| **Arduino IDE** | Entorno de desarrollo para el firmware | 2.x |
| **PubSubClient** | Librería MQTT para Arduino/ESP32 | 2.8+ |
| **WiFi.h** | Librería WiFi nativa del ESP32 | nativa |

---

## 7. Configuración de red y hardware

### 7.1 Red

> **Nota:** Estas configuraciones están hardcodeadas en los archivos. Para cambiarlas hay que editarlas directamente en el código fuente.

| Parámetro | Valor actual |
|---|---|
| SSID WiFi | `INFINITUM6DD1` |
| Contraseña WiFi | `Qm3Gc1Aw4q` |
| IP del broker MQTT (principal) | `192.168.1.82` |
| IP del broker MQTT (servo) | `10.91.115.191` |
| Puerto MQTT | `1883` |
| Topic de control de dirección | `esp32/control` |
| Topic de arranque remoto | `esp32/arranque` |
| Topic de servo | `esp32/servo/control` |
| Topic de potenciómetro | `pot/uno` |
| Topic de temperatura | `LM35/uno` |
| Topic de estado arranque/paro | `arranque/paro` |

### 7.2 Pines GPIO del ESP32

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
| GPIO 34 | `pinPot1` | Lectura de potenciómetro (ADC) |
| GPIO 36 | `pinLM35` | Lectura de sensor LM35 (ADC) |

> **Advertencia:** GPIO 0 en el ESP32 es un pin especial (boot). Usarlo como salida puede causar problemas durante el arranque del dispositivo.

### 7.3 Parámetros de visión

| Parámetro | Valor | Descripción |
|---|---|---|
| Índice de cámara | `0` | Primera cámara detectada por el sistema |
| Rango HSV mínimo | `[100, 150, 0]` | Límite inferior del azul |
| Rango HSV máximo | `[140, 255, 255]` | Límite superior del azul |
| Área mínima del objeto | `500 px` | Filtra ruido pequeño |
| Margen zona segura | `15%` | 15% del ancho a cada lado del centro (zona total = 30%) |
| Delay por fotograma | `1 ms` | `cv2.waitKey(1)` — ESC para salir |

---

## 8. Estructura de archivos

```
Vision-Integrador/
│
├── control_esp32.py        # Script principal de visión + MQTT (Python)
├── ControlServo.py         # Script de control manual de servo por MQTT (Python)
│
├── esp32/
│   └── esp32.ino           # Firmware para el ESP32 (C++ / Arduino)
│
└── DOCUMENTACION.md        # Este archivo
```

---

## 9. Flujo de datos

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
                   ¿ledState == true?
                   /                \
                  No                Sí
            (ignora todo)     ¿Qué mensaje?
                            /     |      \
                        "IZQ" "CENTRO"  "DER"
                          │      │        │
                     Solo izq  Ambos  Solo der
                     motores  motores  motores

                 [Paralelamente cada segundo]
                          │
              ┌───────────┼───────────┐
              │           │           │
        [Publica      [Publica    [Publica
        voltaje       temp LM35   estado
        pot/uno]      LM35/uno]   arranque/paro]
```

---

## 10. Requisitos para ejecutar el proyecto

### En la computadora (script Python)

```bash
# Instalar dependencias de Python
pip install opencv-python paho-mqtt numpy

# Asegurarse de tener una cámara USB conectada

# Ejecutar el script de visión
python3 control_esp32.py

# (Opcional) Ejecutar el control de servo
python3 ControlServo.py
```

### En el ESP32 (firmware)

1. Instalar el **IDE de Arduino** (versión 2.x recomendada).
2. Agregar soporte para **ESP32** en el gestor de placas.
3. Instalar la librería **PubSubClient** (versión 2.8+) desde el gestor de librerías.
4. Abrir `esp32/esp32.ino`.
5. Seleccionar la placa correcta (ej. *ESP32 Dev Module*).
6. Configurar resolución ADC a 12 bits (ya configurado en el código).
7. Compilar y cargar al ESP32.

### Broker MQTT

El broker Mosquitto debe estar corriendo en `192.168.1.82:1883`. Si se desea ejecutar localmente:

```bash
# Instalar Mosquitto
sudo apt install mosquitto mosquitto-clients

# Iniciarlo
sudo systemctl start mosquitto
```

Luego actualizar la IP en los archivos de código.

---

## 11. Limitaciones actuales y posibles mejoras

### Limitaciones

| Limitación | Descripción |
|---|---|
| Color fijo | Solo detecta el color azul; no configurable en tiempo real |
| Credenciales hardcodeadas | WiFi, IP del broker y contraseñas están en el código fuente |
| Sin manejo de errores de cámara | Si la cámara falla, el script se rompe sin aviso claro |
| GPIO 0 como salida | Pin especial del ESP32, puede causar problemas en el arranque |
| Un solo objeto | Solo procesa el primer contorno > 500 px; no maneja múltiples objetos |
| Sin interfaz de configuración | No hay archivo de configuración ni parámetros por línea de comandos |
| IPs distintas para servo | El script ControlServo usa un broker MQTT diferente al principal |

### Posibles mejoras

- [ ] Mover la configuración a un archivo `.env` o `config.json`
- [ ] Permitir seleccionar el color a seguir desde la interfaz o un parámetro
- [ ] Agregar soporte para más direcciones (ARRIBA, ABAJO)
- [ ] Implementar reconexión automática en el script Python
- [ ] Agregar logging a archivo en vez de solo `print()`
- [ ] Crear un panel web simple para monitorear el estado del sistema
- [ ] Proteger la comunicación MQTT con TLS y autenticación
- [ ] Unificar la IP del broker MQTT entre todos los scripts
- [ ] Integrar el control de servo con el sistema de visión

---

*Documentación actualizada el 2026-03-15*
