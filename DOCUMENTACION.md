# Vision-Integrador — Documentación del Proyecto

> **Estado:** Prototipo funcional
> **Versión:** 1.0 (commit `3843fe2`)
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

En términos simples: una cámara detecta hacia dónde se mueve un objeto de color azul, y el sistema mueve o activa dispositivos físicos (LEDs, motores, servos) en la dirección correspondiente.

---

## 2. ¿Para qué sirve?

El sistema puede aplicarse en escenarios como:

| Aplicación | Descripción |
|---|---|
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
│   └──────────────┘      │  Calcula posición: IZQ o DER      │   │
│                         │  Publica mensaje MQTT             │   │
│                         └───────────────┬───────────────────┘   │
└───────────────────────────────────────────────────────────────── ┘
                                          │ MQTT
                                          ▼
                         ┌────────────────────────────┐
                         │   Broker MQTT (Mosquitto)  │
                         │   IP: 10.165.252.191:1883  │
                         │   Topic: esp32/control     │
                         └──────────────┬─────────────┘
                                        │ MQTT (WiFi)
                                        ▼
                         ┌──────────────────────────────┐
                         │       ESP32 (Firmware)       │
                         │   Recibe: "IZQ" o "DER"      │
                         │   Activa pines GPIO          │
                         │                              │
                         │  GPIO 16 ── LED Izquierdo   │
                         │  GPIO 17 ── Motor/Act. Izq  │
                         │  GPIO 4  ── LED Derecho     │
                         │  GPIO 0  ── Motor/Act. Der  │
                         └──────────────────────────────┘
```

---

## 4. Componentes del sistema

### 4.1 Script de visión (Python) — `control_esp32.py`

**Ubicación:** `/control_esp32.py`
**Lenguaje:** Python 3
**Dependencias:** `opencv-python`, `paho-mqtt`

Este script es el cerebro del sistema. Se ejecuta en la computadora y realiza las siguientes tareas en un bucle continuo:

#### ¿Qué hace?

1. **Captura video** desde la cámara conectada al sistema (`/dev/video0`).
2. **Convierte** cada fotograma del espacio de color BGR (cómo OpenCV lee cámaras) al espacio **HSV** (Hue-Saturation-Value), que es más fácil para filtrar colores.
3. **Aplica una máscara** de color para aislar únicamente los píxeles azules dentro del rango HSV definido:
   - Tono (H): 100 a 140 → rango del azul en HSV
   - Saturación (S): 150 a 255 → colores vivos, no blancos/grises
   - Valor (V): 0 a 255 → cualquier brillo
4. **Detecta contornos** en la máscara y descarta los que tienen área menor a 500 píxeles (para evitar ruido).
5. **Calcula el centroide** (centro de masa) del objeto azul más grande encontrado.
6. **Determina la posición** del centroide respecto al centro horizontal del fotograma:
   - Si el centroide está a la **izquierda** del centro → envía `"IZQ"`
   - Si está a la **derecha** → envía `"DER"`
7. **Publica el comando** al broker MQTT en el topic `esp32/control`, pero **solo si el comando cambió** respecto al anterior (para no saturar el broker con mensajes repetidos).
8. **Dibuja** sobre el fotograma:
   - Un rectángulo alrededor del objeto detectado
   - Un punto en el centroide
   - El texto del comando enviado
9. **Muestra** el video en tiempo real en una ventana.

#### Detalle del código clave

```python
# Rango de color azul en HSV
lower_blue = np.array([100, 150, 0])
upper_blue = np.array([140, 255, 255])

# Máscara: solo píxeles dentro del rango azul
mask = cv2.inRange(hsv, lower_blue, upper_blue)

# Encontrar contornos en la máscara
contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

# Calcular centroide usando momentos
M = cv2.moments(c)
cx = int(M["m10"] / M["m00"])  # Centro X
cy = int(M["m01"] / M["m00"])  # Centro Y

# Determinar dirección
if cx < frame_width / 2:
    command = "IZQ"
else:
    command = "DER"
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
3. **Se suscribe** al topic `esp32/control` para recibir comandos.
4. Cuando llega un mensaje:
   - Si es `"IZQ"`: activa los pines GPIO 16 y 17, apaga 4 y 0.
   - Si es `"DER"`: activa los pines GPIO 4 y 0, apaga 16 y 17.
5. **Reconexión automática**: si se pierde la conexión WiFi o MQTT, el firmware intenta reconectarse automáticamente en el loop.
6. También publica periódicamente el mensaje `"hola"` al topic `prueba/uno` (funcionalidad de prueba/heartbeat).

#### Detalle del código clave

```cpp
// Pines de salida
int ledizq = 16;  // LED izquierdo
int ledder = 4;   // LED derecho
int izq    = 17;  // Actuador izquierdo
int der    = 0;   // Actuador derecho

// Callback al recibir mensaje MQTT
void callback(char* topic, byte* payload, unsigned int length) {
  String message = "";
  for (int i = 0; i < length; i++) {
    message += (char)payload[i];
  }

  if (message == "IZQ") {
    digitalWrite(ledizq, HIGH);
    digitalWrite(izq, HIGH);
    digitalWrite(ledder, LOW);
    digitalWrite(der, LOW);
  } else if (message == "DER") {
    digitalWrite(ledder, HIGH);
    digitalWrite(der, HIGH);
    digitalWrite(ledizq, LOW);
    digitalWrite(izq, LOW);
  }
}
```

---

## 5. ¿Cómo funciona paso a paso?

```
Paso 1: El usuario enciende el ESP32 (ya flasheado con el firmware)
           └─► El ESP32 se conecta al WiFi y al broker MQTT
           └─► Se suscribe al topic "esp32/control"

Paso 2: El usuario ejecuta el script Python en la computadora
           └─► Se abre la cámara USB
           └─► Se conecta al mismo broker MQTT

Paso 3: Bucle de visión (se repite ~1000 veces por segundo)
           └─► Captura fotograma de la cámara
           └─► Convierte BGR → HSV
           └─► Aplica máscara de color azul
           └─► Detecta contornos
           └─► Si hay un objeto azul > 500 px:
                   └─► Calcula su centroide (cx, cy)
                   └─► Si cx < ancho/2 → comando = "IZQ"
                   └─► Si cx ≥ ancho/2 → comando = "DER"
                   └─► Si el comando cambió → publica por MQTT

Paso 4: El broker MQTT reenvía el mensaje al ESP32

Paso 5: El ESP32 recibe "IZQ" o "DER"
           └─► Activa los pines GPIO correspondientes
           └─► Los dispositivos físicos responden (LEDs encendidos, motores activos)
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

> **Nota:** Estas configuraciones están hardcodeadas en ambos archivos. Para cambiarlas hay que editarlas directamente en el código fuente.

| Parámetro | Valor actual |
|---|---|
| SSID WiFi | `JOSUE's Galaxy A52` |
| Contraseña WiFi | `jog18030` |
| IP del broker MQTT | `10.165.252.191` |
| Puerto MQTT | `1883` |
| Topic de control | `esp32/control` |
| Topic de prueba | `prueba/uno` |

### 7.2 Pines GPIO del ESP32

| Pin GPIO | Nombre en código | Función |
|---|---|---|
| GPIO 16 | `ledizq` | LED indicador izquierdo |
| GPIO 17 | `izq` | Actuador/motor izquierdo |
| GPIO 4 | `ledder` | LED indicador derecho |
| GPIO 0 | `der` | Actuador/motor derecho |

> **Advertencia:** GPIO 0 en el ESP32 es un pin especial (boot). Usarlo como salida puede causar problemas durante el arranque del dispositivo.

### 7.3 Parámetros de visión

| Parámetro | Valor | Descripción |
|---|---|---|
| Índice de cámara | `0` | Primera cámara detectada por el sistema |
| Rango HSV mínimo | `[100, 150, 0]` | Límite inferior del azul |
| Rango HSV máximo | `[140, 255, 255]` | Límite superior del azul |
| Área mínima del objeto | `500 px` | Filtra ruido pequeño |
| Delay por fotograma | `1 ms` | `cv2.waitKey(1)` |

---

## 8. Estructura de archivos

```
Vision-Integrador/
│
├── control_esp32.py        # Script principal de visión + MQTT (Python)
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
               ¿cx < ancho/2?
               /              \
              Sí               No
              │                │
          "IZQ"             "DER"
              │                │
              └────────┬───────┘
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
                    ¿Mensaje == "IZQ"?
                    /                \
                   Sí                No ("DER")
                   │                  │
          GPIO 16, 17 → HIGH   GPIO 4, 0 → HIGH
          GPIO 4, 0   → LOW    GPIO 16,17 → LOW
```

---

## 10. Requisitos para ejecutar el proyecto

### En la computadora (script Python)

```bash
# Instalar dependencias de Python
pip install opencv-python paho-mqtt numpy

# Asegurarse de tener una cámara USB conectada

# Ejecutar el script
python3 control_esp32.py
```

### En el ESP32 (firmware)

1. Instalar el **IDE de Arduino** (versión 2.x recomendada).
2. Agregar soporte para **ESP32** en el gestor de placas.
3. Instalar la librería **PubSubClient** (versión 2.8+) desde el gestor de librerías.
4. Abrir `esp32/esp32.ino`.
5. Seleccionar la placa correcta (ej. *ESP32 Dev Module*).
6. Compilar y cargar al ESP32.

### Broker MQTT

El broker Mosquitto debe estar corriendo en `10.165.252.191:1883`. Si se desea ejecutar localmente:

```bash
# Instalar Mosquitto
sudo apt install mosquitto mosquitto-clients

# Iniciarlo
sudo systemctl start mosquitto
```

Luego actualizar la IP en ambos archivos de código.

---

## 11. Limitaciones actuales y posibles mejoras

### Limitaciones

| Limitación | Descripción |
|---|---|
| Color fijo | Solo detecta el color azul; no configurable en tiempo real |
| Credenciales hardcodeadas | WiFi, IP del broker y contraseñas están en el código fuente |
| Sin manejo de errores de cámara | Si la cámara falla, el script se rompe sin aviso claro |
| GPIO 0 como salida | Pin especial del ESP32, puede causar problemas en el arranque |
| Un solo objeto | Solo procesa el contorno más grande; no maneja múltiples objetos |
| Sin interfaz de configuración | No hay archivo de configuración ni parámetros por línea de comandos |

### Posibles mejoras

- [ ] Mover la configuración a un archivo `.env` o `config.json`
- [ ] Permitir seleccionar el color a seguir desde la interfaz o un parámetro
- [ ] Agregar soporte para más de 2 direcciones (ARRIBA, ABAJO, CENTRO)
- [ ] Implementar reconexión automática en el script Python
- [ ] Agregar logging a archivo en vez de solo `print()`
- [ ] Crear un panel web simple para monitorear el estado del sistema
- [ ] Proteger la comunicación MQTT con TLS y autenticación

---

*Documentación generada el 2026-03-01*
