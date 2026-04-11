# Control Difuso para Dirección Ackermann – Sistema de Visión

## Descripción General

Este sistema implementa un **controlador difuso tipo Mamdani** que utiliza visión por computadora para dirigir un carro a escala 1:10 con dirección **Ackermann**. La cámara detecta un objeto azul en el frame y, en función de su posición horizontal, calcula el ángulo óptimo del servo de dirección mediante lógica difusa.

### Flujo del sistema

```
Cámara → Detección de color → Posición del objeto → Control Difuso → Ángulo servo → MQTT → ESP32 → Servo
```

### Diferencia respecto al control diferencial anterior

| Aspecto | Diferencial (anterior) | Ackermann (actual) |
|---------|----------------------|-------------------|
| Dirección | Dos motores a distinta velocidad | Un servo de dirección |
| Salida | Comandos discretos: IZQ, DER, CENTRO | Ángulo continuo: 80° – 135° (recto = 113°) |
| Control | Bang-bang (3 estados) | Difuso (salida continua suave) |
| Comunicación | `esp32/control` → texto | `esp32/servo/control` → ángulo numérico |

---

## Variable de Entrada

### Posición normalizada del objeto en el frame

- **Universo de discurso:** [0, 1]
- **0.0** = borde izquierdo del frame
- **0.5** = centro del frame
- **1.0** = borde derecho del frame

Se calcula como:

```
posicion_norm = centro_x_objeto / ancho_frame
```

### Conjuntos difusos de entrada (3)

| Conjunto | Tipo | Parámetros | Descripción |
|----------|------|-----------|-------------|
| **Izquierda** | Trapezoidal | [0.0, 0.0, 0.15, 0.45] | Máxima pertenencia de 0 a 0.15, decrece hasta 0.45 |
| **Centro** | Triangular | [0.25, 0.5, 0.75] | Pico en 0.5, se extiende de 0.25 a 0.75 |
| **Derecha** | Trapezoidal | [0.55, 0.85, 1.0, 1.0] | Crece desde 0.55, máxima de 0.85 a 1.0 |

### Gráfica de los conjuntos de entrada

```
μ
1.0 ┤ ████                                          ████
    │ █████                 ▲                      █████
0.8 ┤ ██████               ╱ ╲                    ██████
    │ ███████             ╱   ╲                  ███████
0.6 ┤ ████████           ╱     ╲                ████████
    │ █████████         ╱       ╲              █████████
0.4 ┤ ██████████       ╱         ╲            ██████████
    │ ███████████     ╱           ╲          ███████████
0.2 ┤ ████████████   ╱             ╲        ████████████
    │ █████████████ ╱               ╲      █████████████
0.0 ┼──────────────┼────────────────┼──────────────────┤
    0.0    0.15  0.25    0.45 0.5  0.55  0.75  0.85  1.0

    ◄── Izquierda ──►   ◄── Centro ──►   ◄── Derecha ──►
```

**Nota:** Las funciones se traslapan intencionalmente en las zonas [0.25, 0.45] y [0.55, 0.75] para generar transiciones suaves y activar múltiples reglas simultáneamente.

---

## Variable de Salida

### Ángulo del servo de dirección

- **Universo de discurso:** [80, 135] grados (rango mecánico del Ackermann)
- **80°**  = giro máximo a la izquierda
- **113°** = línea recta (sin giro)
- **135°** = giro máximo a la derecha

> El rango es **asimétrico**: 33° disponibles a la izquierda de 113°
> (80–113) y solo 22° a la derecha (113–135). Los conjuntos difusos de
> salida se distribuyen de forma asimétrica para cubrir ambos lados.

### Conjuntos difusos de salida (7)

| Conjunto | Tipo | Parámetros [a, b, c] | Rango efectivo | Centro |
|----------|------|---------------------|----------------|--------|
| **Muy Izquierda**  | Triangular | [80, 80, 91]    | 80° – 91°    | 80°  |
| **Med Izquierda**  | Triangular | [80, 91, 102]   | 80° – 102°   | 91°  |
| **Poco Izquierda** | Triangular | [91, 102, 113]  | 91° – 113°   | 102° |
| **Centro**         | Triangular | [102, 113, 120] | 102° – 120°  | 113° |
| **Poco Derecha**   | Triangular | [113, 120, 127] | 113° – 127°  | 120° |
| **Med Derecha**    | Triangular | [120, 127, 135] | 120° – 135°  | 127° |
| **Muy Derecha**    | Triangular | [127, 135, 135] | 127° – 135°  | 135° |

### Distribución visual de los conjuntos de salida

```
        Muy    Med    Poco         Poco   Med    Muy
        Izq    Izq    Izq  Centro  Der    Der    Der
         ▲      ▲      ▲     ▲      ▲      ▲      ▲
        ╱╲    ╱  ╲   ╱  ╲   ╱ ╲   ╱  ╲  ╱  ╲    ╱╲
       ╱  ╲  ╱    ╲ ╱    ╲ ╱   ╲ ╱    ╲╱    ╲  ╱  ╲
      ╱    ╲╱      ╳      ╳     ╳      ╳     ╲╱    ╲
     ╱      ╳     ╱ ╲    ╱ ╲   ╱ ╲    ╱ ╲    ╳      ╲
    ╱      ╱ ╲   ╱   ╲  ╱   ╲ ╱   ╲  ╱   ╲  ╱ ╲      ╲
   ┼──────┼───┼──┼────┼─┼────┼┼────┼─┼────┼─┼───┼──────┤
   80°    91°   102°    113°   120°   127°     135°
```

---

## Base de Conocimiento: 9 Reglas Difusas

Las reglas utilizan los operadores:
- **AND** → operador mínimo: `min(μA, μB)`
- **NOT** → complemento: `1 - μA`
- **OR** (agregación de reglas con mismo consecuente) → operador máximo: `max(r_i, r_j)`

### Tabla de reglas

| # | Antecedente | Consecuente | Propósito |
|---|------------|-------------|-----------|
| **R1** | IF Izquierda AND NOT Centro | Muy Izquierda | Objeto lejos a la izquierda → giro fuerte izquierdo |
| **R2** | IF Izquierda | Med Izquierda | Objeto a la izquierda → giro medio izquierdo |
| **R3** | IF Izquierda AND Centro | Poco Izquierda | Objeto entre izquierda y centro → corrección leve izquierda |
| **R4** | IF Centro AND NOT Izquierda AND NOT Derecha | Centro | Objeto centrado, sin ambigüedad → recto |
| **R5** | IF Centro | Centro | Objeto en zona central → mantener recto |
| **R6** | IF Derecha AND Centro | Poco Derecha | Objeto entre centro y derecha → corrección leve derecha |
| **R7** | IF Derecha | Med Derecha | Objeto a la derecha → giro medio derecho |
| **R8** | IF Derecha AND NOT Centro | Muy Derecha | Objeto lejos a la derecha → giro fuerte derecho |
| **R9** | IF NOT Izquierda AND NOT Derecha AND NOT Centro | Centro | Sin detección clara → mantener recto (seguridad) |

### Cálculo de activación de cada regla

```
r1 = min(μ_izq, 1 - μ_cen)
r2 = μ_izq
r3 = min(μ_izq, μ_cen)
r4 = min(μ_cen, 1 - μ_izq, 1 - μ_der)
r5 = μ_cen
r6 = min(μ_der, μ_cen)
r7 = μ_der
r8 = min(μ_der, 1 - μ_cen)
r9 = min(1 - μ_izq, 1 - μ_der, 1 - μ_cen)
```

### Agregación por conjunto de salida

Cuando múltiples reglas apuntan al mismo conjunto de salida, se usa OR (máximo):

```
Muy_Izquierda = r1
Med_Izquierda = r2
Poco_Izquierda = r3
Centro = max(r4, r5, r9)
Poco_Derecha = r6
Med_Derecha = r7
Muy_Derecha = r8
```

### Lógica detrás de las reglas

- **R1 y R8** (NOT Centro): Se activan fuertemente solo cuando el objeto está muy alejado del centro, produciendo los giros más agresivos.
- **R2 y R7** (simples): Proporcionan una respuesta proporcional base para cualquier detección lateral.
- **R3 y R6** (AND Centro): Se activan en la zona de transición, generando correcciones suaves cuando el objeto está entre el centro y un lateral.
- **R4** (centro puro): Refuerza la posición recta cuando el objeto está claramente centrado.
- **R5** (centro general): Respuesta proporcional base para mantener la dirección.
- **R9** (seguridad): Garantiza que si ningún conjunto tiene activación significativa, el servo se mantiene recto.

---

## Proceso de Inferencia

Se utiliza el **método de inferencia Mamdani**:

### Paso 1: Fuzzificación

Se evalúa la posición normalizada del objeto contra las 3 funciones de membresía de entrada, obteniendo los grados de pertenencia:

```
μ_izq, μ_cen, μ_der = fuzzificar(posicion_normalizada)
```

### Paso 2: Evaluación de reglas

Cada regla produce un nivel de activación (un valor entre 0 y 1). Este nivel se usa para **recortar** (clip) la función de membresía del conjunto de salida correspondiente:

```
MF_recortada = min(MF_original, nivel_activación)
```

### Paso 3: Agregación

Todas las funciones de membresía recortadas se combinan usando el operador **máximo** (unión difusa):

```
MF_agregada(x) = max(MF_recortada_1(x), MF_recortada_2(x), ..., MF_recortada_7(x))
```

### Paso 4: Defuzzificación – Centro de Gravedad

Se calcula el centroide del área bajo la curva agregada:

```
            Σ (x_i × μ_agregada(x_i))
ángulo = ─────────────────────────────
              Σ μ_agregada(x_i)
```

Donde `x_i` recorre el universo discretizado de 80 a 135 (56 puntos con resolución de 1°).

Si el área total es cero (sin detección), se retorna **113°** (recto) como valor por defecto.

---

## Ejemplo de Funcionamiento

### Escenario: Objeto detectado en posición normalizada = 0.3

1. **Fuzzificación:**
   - μ_izq = trapmf(0.3, [0, 0, 0.15, 0.45]) = (0.45 - 0.3) / (0.45 - 0.15) = **0.50**
   - μ_cen = trimf(0.3, [0.25, 0.5, 0.75]) = (0.3 - 0.25) / (0.5 - 0.25) = **0.20**
   - μ_der = trapmf(0.3, [0.55, 0.85, 1.0, 1.0]) = **0.00**

2. **Activación de reglas:**
   - R1: min(0.50, 1 - 0.20) = min(0.50, 0.80) = **0.50** → Muy Izquierda
   - R2: 0.50 = **0.50** → Med Izquierda
   - R3: min(0.50, 0.20) = **0.20** → Poco Izquierda
   - R4: min(0.20, 1 - 0.50, 1 - 0.00) = min(0.20, 0.50, 1.00) = **0.20** → Centro
   - R5: 0.20 = **0.20** → Centro
   - R6: min(0.00, 0.20) = **0.00** → Poco Derecha
   - R7: 0.00 = **0.00** → Med Derecha
   - R8: min(0.00, 1 - 0.20) = **0.00** → Muy Derecha
   - R9: min(1 - 0.50, 1 - 0.00, 1 - 0.20) = min(0.50, 1.00, 0.80) = **0.50** → Centro

3. **Agregación por salida:**
   - Muy Izquierda = 0.50
   - Med Izquierda = 0.50
   - Poco Izquierda = 0.20
   - Centro = max(0.20, 0.20, 0.50) = 0.50
   - Poco/Med/Muy Derecha = 0.00

4. **Defuzzificación (centroide):** El área resultante se concentra mayormente en la mitad izquierda del rango (80°–113°), con el centroide desplazado hacia la izquierda respecto a 113°, indicando un giro moderado a la izquierda.

---

## Comunicación con el ESP32

- **Protocolo:** MQTT
- **Broker:** `10.91.115.191:1883`
- **Tópico:** `esp32/servo/control`
- **Mensaje:** Ángulo entero (80–135, recto = 113) como string
- **Frecuencia:** Solo se envía cuando el cambio es mayor a 2° respecto al último ángulo enviado, evitando sobrecarga en la comunicación.

---

## Detección por Visión

- **Espacio de color:** HSV
- **Color detectado:** Azul
  - Lower: `[100, 150, 0]`
  - Upper: `[140, 255, 255]`
- **Área mínima de contorno:** 500 px²
- **Selección:** Se usa el primer contorno que supere el área mínima

---

## Interfaz Visual (HUD)

La ventana de visualización muestra:

1. **Frame con overlay:** Zona central (tercio medio) resaltada en verde semitransparente
2. **Detección:** Rectángulo azul alrededor del objeto, punto rojo en el centroide
3. **Posición normalizada:** Texto junto al objeto detectado
4. **Ángulo del servo:** Valor numérico en la esquina superior izquierda
5. **Dirección:** Texto IZQUIERDA / CENTRO / DERECHA con código de color
6. **Barra indicadora:** Línea roja inferior que muestra visualmente la posición del servo, con marcas en 80°, 113° y 135°

---

## Dependencias

```
opencv-python (cv2)
numpy
paho-mqtt
```

## Ejecución

```bash
python ControlDifuso.py
```

Presionar **ESC** para finalizar.
