# Guia: Conectar PLC Siemens S7-1200 al broker Mosquitto via Node-RED

## Objetivo

Hacer que el PLC S7-1200 (firmware v1.0) se comunique con el broker Mosquitto de tu Raspberry Pi para controlar el AGV (arranque/paro/emergencia). Como el firmware v1.0 **no soporta MQTT nativo**, usaremos **Node-RED** como puente entre el PLC y el broker.

## Arquitectura

```
+----------------+     Protocolo S7     +------------------+      MQTT       +-------------+
|  PLC S7-1200   | <=================> |  Raspberry Pi    | <=============> |   ESP32     |
|  (firmware v1) |     Puerto 102      |                  |   Puerto 1883   |  Ackermann  |
|                |                      |  - Mosquitto     |                 |             |
|  DB1.DBX0.0    |                      |  - Node-RED      |                 |  Recibe:    |
|  = arranque    |                      |                  |                 |  GO / STOP  |
|  DB1.DBX0.1    |                      |  node-red-       |                 |             |
|  = paro        |                      |  contrib-s7      |                 |  Publica:   |
|  DB1.DBX0.2    |                      |  (lee/escribe    |                 |  MOVIMIENTO |
|  = emergencia  |                      |   variables PLC) |                 |  PARO       |
+----------------+                      +------------------+                 +-------------+
```

**Node-RED** corre en la Raspberry Pi y hace dos cosas:
1. **Lee variables del PLC** (botones) via protocolo S7 y las **publica como MQTT** al broker
2. **Recibe mensajes MQTT** del broker (estado del AGV) y los **escribe en variables del PLC**

---

## Requisitos

### Hardware

| Equipo | Descripcion |
|---|---|
| PLC Siemens S7-1200 | Cualquier firmware (incluyendo v1.0) |
| Raspberry Pi 4B | Con Mosquitto ya instalado |
| Cable Ethernet / Switch | PLC y Raspberry en la **misma red** |

### Software

| Software | Donde |
|---|---|
| TIA Portal | PC (cualquier version compatible con tu PLC) |
| Mosquitto | Raspberry Pi (ya lo tienes) |
| Node-RED | Raspberry Pi (se instala en esta guia) |
| node-red-contrib-s7 | Dentro de Node-RED (se instala en esta guia) |

---

## Paso 1: Configurar la red

Todos los equipos deben estar en la misma subred:

| Equipo | IP (ejemplo) | Mascara |
|---|---|---|
| Raspberry Pi | 192.168.0.100 | 255.255.255.0 |
| PLC S7-1200 | 192.168.0.110 | 255.255.255.0 |
| PC con TIA Portal | 192.168.0.50 | 255.255.255.0 |

> Usa las IPs que correspondan a tu red real. Lo importante es que todas esten en la misma subred.

### Configurar IP del PLC en TIA Portal

1. Abrir TIA Portal → tu proyecto
2. Vista de **Dispositivos y redes** → doble click en el PLC
3. Click en el **puerto Ethernet**
4. **Propiedades** → **Direcciones Ethernet**:
   - IP: `192.168.0.110`
   - Mascara: `255.255.255.0`

### Verificar conectividad

```bash
ping 192.168.0.110    # desde la Raspberry al PLC
```

Si responde, la red esta bien.

---

## Paso 2: Configurar el PLC en TIA Portal

### 2.1 Habilitar PUT/GET (OBLIGATORIO)

Esto permite que Node-RED lea y escriba variables del PLC:

1. En TIA Portal, click derecho sobre el **PLC** → **Propiedades**
2. Ir a **Proteccion y seguridad** (o **Protection & Security**)
3. Marcar: **"Permitir acceso con PUT/GET de interlocutores remotos"** (Permit access with PUT/GET from remote partners)
4. En nivel de acceso, seleccionar: **"Acceso completo (sin proteccion)"** (Full access)

### 2.2 Crear el bloque de datos (DB1)

Crea un **Data Block** llamado `DB_AGV` (o `DB1`) con estas variables:

| Nombre | Tipo | Direccion | Descripcion |
|---|---|---|---|
| `boton_arranque` | Bool | DB1.DBX0.0 | PLC envia GO al AGV |
| `boton_paro` | Bool | DB1.DBX0.1 | PLC envia STOP al AGV |
| `boton_emergencia` | Bool | DB1.DBX0.2 | PLC envia EMERGENCIA al AGV |
| `boton_reset` | Bool | DB1.DBX0.3 | PLC envia RESET_EMERGENCIA |
| `estado_movimiento` | Bool | DB1.DBX1.0 | Node-RED escribe: AGV en movimiento |
| `estado_paro` | Bool | DB1.DBX1.1 | Node-RED escribe: AGV en paro |
| `estado_emergencia` | Bool | DB1.DBX1.2 | Node-RED escribe: AGV en emergencia |
| `temperatura` | Real | DB1.DBD2 | Node-RED escribe: temperatura LM35 |
| `bateria` | Real | DB1.DBD6 | Node-RED escribe: porcentaje bateria |

### 2.3 Desactivar "Optimized Block Access" (OBLIGATORIO)

Si no haces esto, Node-RED no podra leer el DB:

1. En TIA Portal, click derecho sobre **DB1** → **Propiedades**
2. En **Atributos** → **desmarcar** "Acceso optimizado al bloque" (Optimized block access)
3. Click en **Aceptar**

> **Importante:** Al desactivar esto, las direcciones absolutas (DBX0.0, DBD2, etc.) se vuelven accesibles. Con acceso optimizado activado, Node-RED no puede leer las variables.

### 2.4 Compilar y cargar

1. **Compilar** todo el proyecto (Ctrl+B)
2. **Cargar** al PLC (Ctrl+L)
3. Poner el PLC en **RUN**

---

## Paso 3: Instalar Node-RED en la Raspberry Pi

### 3.1 Instalar Node-RED

```bash
bash <(curl -sL https://raw.githubusercontent.com/node-red/linux-installers/master/deb/update-nodejs-and-nodered)
```

Responde **Y** a todas las preguntas.

### 3.2 Habilitar Node-RED como servicio

```bash
sudo systemctl enable nodered
sudo systemctl start nodered
```

### 3.3 Verificar que esta corriendo

```bash
sudo systemctl status nodered
```

Ahora puedes acceder al editor desde un navegador:

```
http://192.168.0.100:1880
```

(reemplaza con la IP de tu Raspberry)

### 3.4 Instalar node-red-contrib-s7

1. En el editor de Node-RED (navegador), click en el **menu** (tres lineas horizontales, arriba a la derecha)
2. Click en **Manage palette**
3. Ir a la pestana **Install**
4. Buscar: `node-red-contrib-s7`
5. Click en **Install**
6. Esperar a que termine y cerrar

Ahora tendras los nodos **s7 in** y **s7 out** disponibles en la paleta izquierda.

---

## Paso 4: Verificar que Mosquitto acepta conexiones

En la Raspberry Pi:

```bash
sudo nano /etc/mosquitto/mosquitto.conf
```

Verifica que tenga:

```
listener 1883
allow_anonymous true
```

Reiniciar:

```bash
sudo systemctl restart mosquitto
```

---

## Paso 5: Crear el flow en Node-RED

Abre el editor: `http://192.168.0.100:1880`

### 5.1 Configurar el Endpoint S7 (conexion al PLC)

1. Arrastra un nodo **s7 in** al lienzo
2. Doble click para abrirlo
3. En **Connection**, click en el **lapiz** (editar) para crear una nueva conexion
4. Configurar:

| Campo | Valor |
|---|---|
| **Transport** | ISO-on-TCP |
| **Address** | 192.168.0.110 (IP de tu PLC) |
| **Port** | 102 |
| **Rack** | 0 |
| **Slot** | 1 |
| **Cycle Time (ms)** | 500 |
| **Timeout (ms)** | 1500 |

> **Rack y Slot:** Para S7-1200, normalmente es Rack=0, Slot=1. Puedes verificarlo en TIA Portal → Vista de dispositivos → click en el PLC → ver las propiedades del slot.

5. Ir a la pestana **Variables** y agregar:

| Name | Address |
|---|---|
| `boton_arranque` | DB1,X0.0 |
| `boton_paro` | DB1,X0.1 |
| `boton_emergencia` | DB1,X0.2 |
| `boton_reset` | DB1,X0.3 |

> **Formato de direcciones en Node-RED:**
> - `DB1,X0.0` = Bit 0 del byte 0 en DB1 (equivale a DB1.DBX0.0 en TIA Portal)
> - `DB1,REAL2` = Real en byte 2 de DB1 (equivale a DB1.DBD2)
> - `M0.0` = Marca M0.0
> - `I0.0` = Entrada I0.0
> - `Q0.0` = Salida Q0.0

6. Click en **Add** / **Done** para guardar

### 5.2 Flow 1: PLC → MQTT (botones del PLC controlan el AGV)

Este flow lee los botones del PLC y publica comandos MQTT:

```
[s7 in: boton_arranque] → [function: "GO"] → [mqtt out: esp32/arranque]
[s7 in: boton_paro]     → [function: "STOP"] → [mqtt out: esp32/arranque]
[s7 in: boton_emergencia] → [function: "EMERGENCIA"] → [mqtt out: esp32/arranque]
[s7 in: boton_reset]    → [function: "RESET_EMERGENCIA"] → [mqtt out: esp32/arranque]
```

**Paso a paso:**

**a) Nodo s7 in (boton_arranque):**
1. Arrastra un nodo **s7 in** al lienzo
2. Doble click → seleccionar conexion PLC creada antes
3. Mode: **Single Variable**
4. Variable: `boton_arranque`
5. Marcar **Emit only when value changes (diff)**

**b) Nodo function (convertir a comando MQTT):**
1. Arrastra un nodo **function** y conectalo a la salida del s7 in
2. Doble click → poner este codigo:

```javascript
// Solo enviar cuando el boton se activa (flanco de subida)
if (msg.payload === true || msg.payload === 1) {
    msg.payload = "GO";
    return msg;
}
return null;  // No enviar nada si el boton se desactiva
```

**c) Nodo mqtt out:**
1. Arrastra un nodo **mqtt out** y conectalo a la salida del function
2. Doble click → configurar:
   - **Server**: click en el lapiz → agregar:
     - Server: `localhost` (o `127.0.0.1`, porque Mosquitto corre en la misma Raspberry)
     - Port: `1883`
   - **Topic**: `esp32/arranque`
   - **QoS**: 0

**d) Repetir para los otros botones:**

Repite los pasos a-c para cada boton, cambiando:

| Nodo s7 in | Codigo en function | Topic mqtt out |
|---|---|---|
| `boton_arranque` | `msg.payload = "GO";` | `esp32/arranque` |
| `boton_paro` | `msg.payload = "STOP";` | `esp32/arranque` |
| `boton_emergencia` | `msg.payload = "EMERGENCIA";` | `esp32/arranque` |
| `boton_reset` | `msg.payload = "RESET_EMERGENCIA";` | `esp32/arranque` |

### 5.3 Flow 2: MQTT → PLC (estado del AGV se muestra en el PLC)

Este flow recibe el estado del AGV por MQTT y lo escribe en variables del PLC:

```
[mqtt in: arranque/paro] → [function: decodificar] → [s7 out: estado_*]
```

**a) Nodo mqtt in:**
1. Arrastra un nodo **mqtt in** al lienzo
2. Doble click → configurar:
   - **Server**: el mismo `localhost:1883`
   - **Topic**: `arranque/paro`
   - **QoS**: 0

**b) Nodo function (decodificar estado):**
1. Arrastra un nodo **function** y conectalo
2. Codigo (3 salidas):

```javascript
// Decodificar estado del AGV y separar en 3 salidas
var mov = { payload: false };
var par = { payload: false };
var emg = { payload: false };

if (msg.payload === "MOVIMIENTO") {
    mov.payload = true;
} else if (msg.payload === "PARO") {
    par.payload = true;
} else if (msg.payload === "EMERGENCIA") {
    emg.payload = true;
}

return [mov, par, emg];
```

3. En la configuracion del function, cambiar **Outputs** a **3**

**c) Tres nodos s7 out:**

Conectar cada salida del function a un nodo **s7 out**:

| Salida | Nodo s7 out → Variable |
|---|---|
| Salida 1 | `estado_movimiento` (DB1,X1.0) |
| Salida 2 | `estado_paro` (DB1,X1.1) |
| Salida 3 | `estado_emergencia` (DB1,X1.2) |

Para cada s7 out:
1. Arrastra un nodo **s7 out**
2. Doble click → seleccionar la conexion PLC
3. Variable: la correspondiente

### 5.4 Flow 3: MQTT → PLC (temperatura y bateria)

**a) Temperatura:**

```
[mqtt in: LM35/uno] → [function: convertir] → [s7 out: temperatura]
```

Function:
```javascript
msg.payload = parseFloat(msg.payload);
return msg;
```

s7 out → Variable: `temperatura` (DB1,REAL2)

**b) Bateria:**

```
[mqtt in: bateria/porcentaje] → [function: convertir] → [s7 out: bateria]
```

Function:
```javascript
msg.payload = parseFloat(msg.payload);
return msg;
```

s7 out → Variable: `bateria` (DB1,REAL6)

### 5.5 Agregar variables de escritura al Endpoint S7

Vuelve a la configuracion del Endpoint S7 (doble click en cualquier nodo s7 → click en el lapiz de Connection) y agrega las variables de escritura en la pestana **Variables**:

| Name | Address |
|---|---|
| `estado_movimiento` | DB1,X1.0 |
| `estado_paro` | DB1,X1.1 |
| `estado_emergencia` | DB1,X1.2 |
| `temperatura` | DB1,REAL2 |
| `bateria` | DB1,REAL6 |

---

## Paso 6: Deploy y probar

### 6.1 Desplegar el flow

Click en el boton rojo **Deploy** (arriba a la derecha del editor).

Los nodos s7 deben mostrar un punto **verde** indicando conexion exitosa con el PLC.

### 6.2 Probar PLC → AGV

1. En TIA Portal, abre una **Watch Table** (tabla de observacion)
2. Agrega `DB1.DBX0.0` (boton_arranque)
3. Cambia el valor a `TRUE`
4. En la Raspberry, verifica con:
   ```bash
   mosquitto_sub -t "esp32/arranque" -v
   ```
5. Deberia aparecer: `esp32/arranque GO`

### 6.3 Probar AGV → PLC

1. En la Raspberry, publica un mensaje de prueba:
   ```bash
   mosquitto_pub -t "arranque/paro" -m "MOVIMIENTO"
   ```
2. En TIA Portal, en la Watch Table, verifica que `DB1.DBX1.0` (estado_movimiento) se ponga en `TRUE`

---

## Paso 7: Programar logica en el PLC (opcional)

Ahora puedes usar las variables del DB1 en tu programa del PLC (OB1) para encender luces, activar salidas, etc.

Ejemplo en LAD:

```
|  DB1.DBX1.0    |       Q0.0        |
|---| |----------|------( )----------|    // Si AGV en movimiento → encender luz verde
|                                     |
|  DB1.DBX1.1    |       Q0.1        |
|---| |----------|------( )----------|    // Si AGV en paro → encender luz roja
|                                     |
|  DB1.DBX1.2    |       Q0.2        |
|---| |----------|------( )----------|    // Si AGV en emergencia → encender luz amarilla
```

Para los botones, puedes usar entradas fisicas del PLC mapeadas al DB:

```
|  I0.0          |    DB1.DBX0.0     |
|---| |----------|------( )----------|    // Boton fisico → arranque
|                                     |
|  I0.1          |    DB1.DBX0.1     |
|---| |----------|------( )----------|    // Boton fisico → paro
```

---

## Resumen del flujo completo

```
Boton fisico PLC (I0.0)
        |
        v
DB1.DBX0.0 = TRUE (programa LAD del PLC)
        |
        v
Node-RED lee DB1.DBX0.0 via protocolo S7 (puerto 102)
        |
        v
Node-RED publica "GO" en topic "esp32/arranque" via MQTT
        |
        v
Mosquitto (broker) retransmite el mensaje
        |
        v
ESP32 recibe "GO" → enciende motor y LED
        |
        v
ESP32 publica "MOVIMIENTO" en topic "arranque/paro"
        |
        v
Mosquitto retransmite
        |
        v
Node-RED recibe "MOVIMIENTO" y escribe DB1.DBX1.0 = TRUE en el PLC
        |
        v
PLC enciende luz verde en Q0.0 (programa LAD)
```

---

## Solucion de problemas

### Node-RED no conecta al PLC (punto rojo en nodos s7)

- **Verificar IP**: ping al PLC desde la Raspberry
- **Verificar PUT/GET**: debe estar habilitado en TIA Portal
- **Verificar Rack/Slot**: para S7-1200 normalmente es Rack=0, Slot=1
- **Verificar Optimized Block Access**: debe estar **desactivado** en DB1

### Error: "This service is not implemented on the module"

- PUT/GET no esta habilitado en las propiedades del PLC
- El nivel de acceso no esta en "Full access"

### Node-RED conecta pero no lee variables

- Las direcciones en Node-RED usan formato diferente a TIA Portal:
  - TIA Portal: `DB1.DBX0.0` → Node-RED: `DB1,X0.0`
  - TIA Portal: `DB1.DBD2` → Node-RED: `DB1,REAL2`
- Verificar que "Optimized block access" esta desactivado

### Los mensajes MQTT no llegan al ESP32

- Verificar que Mosquitto tiene `listener 1883` y `allow_anonymous true`
- Verificar que el topic es exactamente `esp32/arranque` (case sensitive)
- Probar con: `mosquitto_sub -t "#" -v` para ver todos los mensajes

---

## Tabla de direcciones Node-RED vs TIA Portal

| Node-RED | TIA Portal | Tipo | Tamano |
|---|---|---|---|
| `DB1,X0.0` | DB1.DBX0.0 | Bool | 1 bit |
| `DB1,X0.1` | DB1.DBX0.1 | Bool | 1 bit |
| `DB1,BYTE1` | DB1.DBB1 | Byte | 1 byte |
| `DB1,INT2` | DB1.DBW2 | Int | 2 bytes |
| `DB1,DINT4` | DB1.DBD4 | DInt | 4 bytes |
| `DB1,REAL2` | DB1.DBD2 | Real | 4 bytes |
| `DB1,S10.20` | DB1.DBB10 (string len 20) | String | variable |
| `I0.0` | I0.0 | Bool | 1 bit |
| `Q0.0` | Q0.0 | Bool | 1 bit |
| `M0.0` | M0.0 | Bool | 1 bit |

---

## Referencias

1. FlowFuse - [Integrating Siemens S7 PLCs with Node-RED](https://flowfuse.com/blog/2025/01/integrating-siemens-s7-plcs-with-node-red-guide/)
2. node-red-contrib-s7 - [NPM package](https://www.npmjs.com/package/node-red-contrib-s7)
3. node-red-contrib-s7 - [GitHub](https://github.com/st-one-io/node-red-contrib-s7)
4. Programacion Siemens - [MQTT en TIA Portal](https://programacionsiemens.com/como-enviar-mensajes-mqtt-en-tia-portal/)
5. SolisPLC - [PLC MQTT Communication](https://www.solisplc.com/tutorials/plc-mqtt-communication-using-tia-portal-mosquitto-and-node-red)
6. Didactronica - [ESP32 con Siemens por Node-RED](https://didactronica.microlsb.es/docs/esp32-con-siemens-por-node-red-con-mqtt-preconfigurado)
7. Siemens Support - [SIMATIC como cliente MQTT](https://support.industry.siemens.com/cs/document/109748872)
