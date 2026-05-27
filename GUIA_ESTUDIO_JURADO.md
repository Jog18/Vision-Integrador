# Guia de Estudio para Presentacion ante Jurado

> Resumen ejecutivo del proyecto. Contiene solo lo esencial para entender,
> explicar y defender el proyecto frente al jurado.

---

## Titulo del Proyecto

**Sistema de Control Difuso y Vision Artificial para la Navegacion Autonoma de un AGV con Direccion Ackermann y Supervision SCADA**

**Subtitulo:** Integracion de OpenCV, Logica Difusa Mamdani, MQTT e Interfaz SCADA sobre Raspberry Pi 4B y ESP32

---

## 1. Que es y que hace

Es un vehiculo autonomo guiado (AGV) a escala 1:10 con direccion Ackermann que:

- **Sigue una linea blanca** sobre fondo negro usando vision artificial (camara + OpenCV)
- **Calcula la direccion** con un controlador de logica difusa tipo Mamdani (scikit-fuzzy)
- **Detecta estaciones de color** (Rojo, Azul, Amarillo) y se detiene automaticamente
- **Se comunica inalambricamente** via protocolo MQTT entre Raspberry Pi y ESP32
- **Se monitorea y controla** desde una interfaz SCADA con indicadores, graficas y reportes
- **Soporta integracion con PLC** Siemens S7-1200 via Node-RED

---

## 2. Arquitectura del Sistema (Diagrama Clave)

```
+-------------------+                    +-------------------+
|  RASPBERRY PI 4B  |      WiFi/MQTT     |      ESP32        |
|                   | <================> |                   |
|  - Camara CSI     |   Angulo servo     |  - Servo direccion|
|  - OpenCV         |   GO/STOP          |  - Motor DC       |
|  - Control Difuso |   Temperatura      |  - Sensores       |
|  - SCADA (Tkinter)|   Bateria          |  - Botones        |
|  - Broker MQTT    |   Estado           |  - E-Stop         |
+-------------------+                    +-------------------+
        |
        | Protocolo S7 (opcional)
        v
+-------------------+
| PLC S7-1200       |
| (via Node-RED)    |
+-------------------+
```

**Resumen:** La Raspberry Pi es el cerebro (vision + control difuso + SCADA). El ESP32 es el actuador (motor + servo + sensores). Se comunican por MQTT.

---

## 3. Control Difuso — Lo que debes saber explicar

### Que es y por que se usa

La logica difusa permite tomar decisiones con **grados de pertenencia** en lugar de valores binarios. La posicion de la linea no es solo "izquierda" o "derecha", sino que puede estar "un poco a la izquierda" con un grado de 0.6 y "centro" con un grado de 0.4, produciendo un angulo intermedio.

### Tipo: Mamdani con defuzzificacion por centroide

### Variables

| Variable | Tipo | Universo | Significado |
|---|---|---|---|
| **Posicion** | Entrada | [0, 1] normalizado | Donde esta la linea en el frame (0=izq, 0.5=centro, 1=der) |
| **Servo** | Salida | [65, 160] grados | Angulo de las ruedas (65=izq, 113=recto, 160=der) |

### 7 Conjuntos difusos (entrada y salida)

| Conjunto | Entrada (posicion) | Salida (servo) |
|---|---|---|
| Muy Izquierda | trapecio [0, 0, 0.05, 0.20] | triangulo [65, 65, 81] |
| Med Izquierda | triangulo [0.05, 0.20, 0.35] | triangulo [65, 81, 97] |
| Poco Izquierda | triangulo [0.20, 0.35, 0.50] | triangulo [81, 97, 113] |
| Centro | triangulo [0.35, 0.50, 0.65] | triangulo [97, 113, 129] |
| Poco Derecha | triangulo [0.50, 0.65, 0.80] | triangulo [113, 129, 145] |
| Med Derecha | triangulo [0.65, 0.80, 0.95] | triangulo [129, 145, 160] |
| Muy Derecha | trapecio [0.80, 0.95, 1, 1] | triangulo [145, 160, 160] |

### 7 Reglas (mapeo directo 1 a 1)

```
R1: IF posicion = Muy_Izq    THEN servo = Muy_Izq    (65)
R2: IF posicion = Med_Izq    THEN servo = Med_Izq    (81)
R3: IF posicion = Poco_Izq   THEN servo = Poco_Izq   (97)
R4: IF posicion = Centro     THEN servo = Centro     (113)
R5: IF posicion = Poco_Der   THEN servo = Poco_Der   (129)
R6: IF posicion = Med_Der    THEN servo = Med_Der    (145)
R7: IF posicion = Muy_Der    THEN servo = Muy_Der    (160)
```

### Pipeline: como funciona en cada frame

```
Posicion de la linea (ej: 0.42)
    -> Fuzzificacion: poco_izq=0.533, centro=0.467
    -> Se activan R3 y R4 simultaneamente
    -> Recorte: las MFs de salida se recortan a esos niveles
    -> Agregacion: se combinan con MAX
    -> Defuzzificacion: centroide del area = ~104 grados
    -> Se envia 104 al servo via MQTT
```

**Punto clave:** Las transiciones suaves se logran por el **traslape** entre conjuntos adyacentes, no por mas reglas.

---

## 4. Vision Artificial — Lo que debes saber explicar

### Deteccion de linea blanca

1. Captura frame 640x480 con Picamera2
2. Convierte a escala de grises
3. Umbral binario (> 200 = blanco)
4. Morfologia: apertura + cierre (elimina ruido)
5. Busca contornos, toma el mayor
6. Calcula centroide -> posicion normalizada [0, 1]

### Deteccion de lineas de color (Rojo, Azul, Amarillo)

1. ROI: franja horizontal entre 55% y 85% de la altura del frame
2. Transformacion afin para corregir perspectiva de la camara
3. Filtro Gaussiano 7x7 para reducir ruido
4. Convierte a espacio HSV
5. `cv2.inRange()` con rangos calibrados por color
6. Para **Rojo**: dos mascaras combinadas (hue 0-10 y 170-180) porque el rojo envuelve el espectro HSV
7. Morfologia + contornos -> si area > 1500px = deteccion valida

### Anti-rebote (debouncing)

Solo cuenta cuando una linea **aparece** (transicion False->True), no mientras es visible. Evita contar la misma linea multiples veces.

---

## 5. Maquina de Estados — 3 estados

```
SEGUIR_LINEA  ----[3ra linea Rojo/Azul]---->  ESPERA_ESTACION (5 seg)
              ----[3ra linea Amarillo]----->  REPOSO_AMARILLO (indefinido)

ESPERA_ESTACION  ----[timer 5s]---->  SEGUIR_LINEA (con cooldown 3s)
REPOSO_AMARILLO  ----[GO externo]---->  SEGUIR_LINEA (con cooldown 3s)
```

| Estado | Que hace | Como sale |
|---|---|---|
| SEGUIR_LINEA | Sigue la linea blanca + cuenta lineas de color | Al llegar a 3 lineas del mismo color |
| ESPERA_ESTACION | Detenido en estacion Roja (descarga) o Azul (carga) | Automaticamente despues de 5 segundos |
| REPOSO_AMARILLO | Detenido en estacion Amarilla (reposo) | Comando externo: SCADA o boton fisico |

### Zonas con significado operativo

| Color | Zona | Significado |
|---|---|---|
| Azul | CARGA | Zona de carga de material |
| Rojo | DESCARGA | Zona de descarga de material |
| Amarillo | REPOSO | Zona de espera/reposo |

---

## 6. Comunicacion MQTT — Topics principales

| Topic | Quien publica | Que contiene |
|---|---|---|
| `esp32/servo/control` | Raspberry Pi | Angulo del servo (65-160) |
| `esp32/arranque` | SCADA / Pi | GO, STOP, EMERGENCIA, RESET_EMERGENCIA |
| `arranque/paro` | ESP32 | MOVIMIENTO, PARO, EMERGENCIA |
| `agv/estacion` | Raspberry Pi | CARGA, DESCARGA, REPOSO, EN_RUTA |
| `bateria/porcentaje` | ESP32 | Porcentaje 0-100% |
| `LM35/uno` | ESP32 | Temperatura en grados C |

**Broker:** Mosquitto corriendo en la Raspberry Pi (puerto 1883)

---

## 7. Interfaz SCADA — Que muestra y que hace

| Elemento | Descripcion |
|---|---|
| 3 focos de estado | PARO (rojo), MOVIMIENTO (verde), EMERGENCIA (amarillo) |
| 4 focos de zona | CARGA (azul), DESCARGA (rojo), REPOSO (amarillo), EN RUTA (verde) |
| 4 botones de control | ARRANQUE, PARO, E-STOP, RESET E-STOP |
| 1 boton de reporte | Genera PDF con eventos y grafica de temperatura |
| Grafica de temperatura | Tiempo real, 60 segundos de historial, alerta en 40 C |
| Barra de bateria | 0-100%, colores por nivel, alerta en 20% |
| Registro CSV | Todos los eventos con fecha, hora, origen, sensores |

---

## 8. ESP32 — Hardware y funciones

### Componentes principales

| Componente | Funcion |
|---|---|
| Servo (GPIO 5) | Direccion Ackermann, rango 65-160 grados |
| Motor DC + TB6612FNG | Propulsion, PWM fijo = 50 |
| Sensor LM35 (GPIO 36) | Temperatura |
| Divisor de voltaje (GPIO 34) | Porcentaje de bateria (8V-11.3V) |
| 3 LEDs de bateria | Nivel: bajo/medio/alto |
| LED estado (GPIO 2) | Encendido = en movimiento |
| LED paro (GPIO 23) | Encendido = en paro, parpadea = emergencia |
| Boton ON/OFF | Arranque y paro fisico con debounce |
| E-Stop (GPIO 13) | Boton NC con interrupcion de hardware |

### Paro de emergencia (E-Stop)

- **Fisico:** Boton normalmente cerrado. Si se presiona o el cable se rompe -> emergencia (fail-safe)
- **Virtual:** Boton E-STOP en la interfaz SCADA
- **Reset:** Requiere boton fisico liberado + comando RESET_EMERGENCIA

---

## 9. Integracion con PLC S7-1200

- El PLC no soporta MQTT nativo (firmware v1.0)
- Se usa **Node-RED** en la Raspberry Pi como puente
- Node-RED lee variables del PLC via **protocolo S7** (puerto 102) y las publica como MQTT
- Node-RED recibe MQTT del AGV y escribe en variables del PLC
- El PLC puede controlar el AGV con botones fisicos y mostrar estado con luces
- Requisitos: PUT/GET habilitado, acceso optimizado desactivado en DB1

---

## 10. Tecnologias utilizadas (resumen)

| Capa | Tecnologias |
|---|---|
| Vision | OpenCV, Picamera2, procesamiento HSV, umbrales, contornos, transformacion afin |
| Control | Logica difusa Mamdani (scikit-fuzzy), 7 conjuntos, 7 reglas, centroide |
| Comunicacion | MQTT (Mosquitto broker), WiFi, paho-mqtt, PubSubClient |
| Microcontrolador | ESP32, Arduino C++, ESP32Servo, TB6612FNG, ADC, interrupciones |
| Interfaz | Tkinter (SCADA), graficas en canvas, reportes PDF (ReportLab) |
| Industrial | PLC Siemens S7-1200, Node-RED, protocolo S7 |
| Hardware | Raspberry Pi 4B, ESP32, servo, motor DC, LM35, divisor de voltaje, LEDs |

---

## 11. Datos de la pista

| Parametro | Valor |
|---|---|
| Dimensiones | 240 x 240 cm |
| Linea | Blanca sobre fondo negro |
| Estaciones | 50 x 50 cm, 3 lineas de color perpendiculares antes de cada zona |
| Curvas | Radio 80 cm y 150 cm |

---

## 12. Preguntas frecuentes del jurado

### "Por que logica difusa y no PID?"
La logica difusa no requiere un modelo matematico del sistema. Con 7 reglas intuitivas se obtiene un control suave que se adapta bien a la incertidumbre de la vision por computadora. Un PID requeriria sintonizacion precisa (Kp, Ki, Kd) y es mas sensible al ruido de la camara.

### "Por que Mamdani y no Sugeno?"
Mamdani es mas intuitivo: las reglas producen conjuntos difusos como salida, lo que facilita la interpretacion. Sugeno usa funciones matematicas como salida, lo que es mas eficiente computacionalmente pero menos visual para fines educativos.

### "Como maneja el ruido de la camara?"
Multiples filtros: umbral de area minima (descarta contornos pequenos), morfologia (apertura elimina ruido, cierre llena huecos), filtro Gaussiano, zona muerta de 2 grados para envio de angulo, y anti-rebote por transicion para el conteo de colores.

### "Por que MQTT y no comunicacion directa?"
MQTT es un protocolo ligero tipo publicador-suscriptor ideal para IoT. Permite desacoplar los componentes: la Pi publica angulos sin saber si el ESP32 esta conectado, el SCADA recibe datos sin importar quien los publica. Ademas, facilita la integracion con PLC via Node-RED.

### "Que pasa si se pierde la linea?"
El sistema gira el servo al maximo hacia el ultimo lado donde vio la linea. Si estaba a la izquierda (posicion < 0.5), gira full izquierda (65 grados). Si estaba a la derecha (posicion >= 0.5), gira full derecha (160 grados).

### "Por que direccion Ackermann?"
Es el mecanismo de direccion usado en vehiculos reales (automoviles). A diferencia del diferencial (dos motores), el Ackermann usa un solo servo para dirigir y un motor para traccion. Es mas realista para simular un AGV industrial.

### "Que es el centroide en la defuzzificacion?"
Es el centro de gravedad del area formada por la union de todas las funciones de membresia de salida recortadas. Se calcula como: `SUM(x * mu(x)) / SUM(mu(x))`. Produce un valor unico y continuo como salida.

### "Como funciona el E-Stop?"
Usa un boton normalmente cerrado (NC): en operacion normal el circuito esta cerrado. Si se presiona el boton o el cable se rompe, el circuito se abre y se activa la emergencia. Es **fail-safe**: cualquier falla de hardware activa la parada de emergencia.

### "Que rol cumple el PLC?"
El PLC S7-1200 se integra como una capa adicional de control industrial. Permite controlar el AGV desde botones fisicos del PLC y visualizar el estado del AGV en las salidas del PLC (luces piloto). La comunicacion se hace via Node-RED que traduce entre protocolo S7 y MQTT.

---

*Guia de estudio — Version Final — 2026-05-26*
