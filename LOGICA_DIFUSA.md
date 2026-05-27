# Logica Difusa del Controlador — Guia Completa

> Guia tecnica detallada del sistema de control difuso tipo Mamdani implementado
> con scikit-fuzzy (skfuzzy) para el AGV Ackermann seguidor de linea.
>
> **Archivo fuente:** `ControlLinea_skfuzzys2.py`, funcion `crear_sistema_difuso()`

---

## 1. Que es la logica difusa y por que se usa aqui

La logica difusa permite manejar **incertidumbre** y **gradualidad** en la toma de decisiones. A diferencia de la logica clasica (0 o 1), un valor puede pertenecer parcialmente a varios conjuntos al mismo tiempo.

En este proyecto, la camara detecta la posicion de una linea blanca en el frame. Esa posicion no es simplemente "izquierda" o "derecha" — puede estar "un poco a la izquierda" o "entre el centro y la derecha". La logica difusa maneja estas situaciones de forma natural, produciendo un angulo de servo **suave y continuo** en lugar de saltos bruscos.

### Tipo de controlador: Mamdani

Se usa el metodo de inferencia **Mamdani**, que es el mas comun en control difuso. Sus caracteristicas:
- Las reglas tienen la forma: `IF entrada ES conjunto THEN salida ES conjunto`
- La salida de cada regla es un **conjunto difuso** (no un numero)
- Se combinan todas las salidas y se **defuzzifica** para obtener un valor concreto
- Metodo de defuzzificacion: **centroide** (centro de gravedad del area resultante)

---

## 2. Libreria utilizada: scikit-fuzzy (skfuzzy)

### Instalacion

```bash
pip install scikit-fuzzy
```

> Dependencias automaticas: `numpy`, `scipy`, `networkx`

### Modulos importados

```python
import numpy as np
import skfuzzy as fuzz
from skfuzzy import control as ctrl
```

| Modulo | Uso |
|---|---|
| `skfuzzy` (`fuzz`) | Funciones de membresia: `trimf`, `trapmf` |
| `skfuzzy.control` (`ctrl`) | Clases de alto nivel: `Antecedent`, `Consequent`, `Rule`, `ControlSystem`, `ControlSystemSimulation` |
| `numpy` | Creacion de universos con `np.linspace` y arrays para parametros |

---

## 3. Construccion del Universo de Discurso

### 3.1 Variable de Entrada: Posicion normalizada

```python
posicion = ctrl.Antecedent(np.linspace(0, 1, 1001), 'posicion')
```

| Elemento | Descripcion |
|---|---|
| `ctrl.Antecedent` | Clase que define una **variable de entrada** del sistema difuso |
| `np.linspace(0, 1, 1001)` | Crea el **universo de discurso**: 1001 puntos equidistantes de 0.0 a 1.0 |
| `'posicion'` | Nombre identificador de la variable |

**Significado fisico:**
- `0.0` = la linea esta en el borde izquierdo del frame
- `0.5` = la linea esta en el centro del frame
- `1.0` = la linea esta en el borde derecho del frame

**Como se calcula:**
```python
posicion_norm = centro_x_del_contorno / ancho_del_frame  # cx / 640
```

### 3.2 Variable de Salida: Angulo del servo

```python
servo = ctrl.Consequent(np.linspace(65, 160, 96), 'servo', defuzzify_method='centroid')
```

| Elemento | Descripcion |
|---|---|
| `ctrl.Consequent` | Clase que define una **variable de salida** del sistema difuso |
| `np.linspace(65, 160, 96)` | Universo de discurso: 96 puntos de 65 a 160 (resolucion ~1 grado) |
| `'servo'` | Nombre identificador de la variable |
| `defuzzify_method='centroid'` | Metodo de defuzzificacion: **centro de gravedad** |

**Significado fisico:**
- `65 grados` = giro maximo a la **izquierda**
- `113 grados` = ruedas **rectas** (sin giro)
- `160 grados` = giro maximo a la **derecha**
- Rango simetrico: 48 grados a cada lado de 113

---

## 4. Funciones de Membresia (Membership Functions)

Las funciones de membresia definen **como** un valor pertenece a un conjunto difuso. Devuelven un grado de pertenencia `mu` entre 0.0 (no pertenece) y 1.0 (pertenece completamente).

### 4.1 Funcion Triangular — `fuzz.trimf`

```python
fuzz.trimf(universo, [a, b, c])
```

**Forma:**
```
        b (pico, mu=1.0)
       / \
      /   \
     /     \
    /       \
---a---------c---  (base, mu=0)
```

**Formula matematica:**
```
trimf(x, [a, b, c]):
    si x <= a:         mu = 0
    si a < x <= b:     mu = (x - a) / (b - a)    # rampa de subida
    si b < x < c:      mu = (c - x) / (c - b)    # rampa de bajada
    si x >= c:          mu = 0
```

**Ejemplo:** `fuzz.trimf(universo, [0.20, 0.35, 0.50])`
- En x=0.20: mu=0 (inicio)
- En x=0.275: mu=0.5 (mitad de la subida)
- En x=0.35: mu=1.0 (pico)
- En x=0.425: mu=0.5 (mitad de la bajada)
- En x=0.50: mu=0 (fin)

### 4.2 Funcion Trapezoidal — `fuzz.trapmf`

```python
fuzz.trapmf(universo, [a, b, c, d])
```

**Forma:**
```
     b-----------c  (meseta, mu=1.0)
    /             \
   /               \
  /                 \
-a-------------------d-  (base, mu=0)
```

**Formula matematica:**
```
trapmf(x, [a, b, c, d]):
    si x <= a:         mu = 0
    si a < x < b:      mu = (x - a) / (b - a)    # rampa de subida
    si b <= x <= c:     mu = 1.0                   # meseta
    si c < x < d:       mu = (d - x) / (d - c)    # rampa de bajada
    si x >= d:          mu = 0
```

**Caso especial (bordes):** Cuando `a == b` o `c == d`, la funcion se convierte en un medio trapecio:
```
# trapmf([0.0, 0.0, 0.05, 0.20]) — borde izquierdo
##########
##########\
##########  \
##########    \
0.0  0.05     0.20
```

---

## 5. Conjuntos Difusos de Entrada (7 conjuntos)

### 5.1 Definicion en codigo

```python
posicion['muy_izq']  = fuzz.trapmf(posicion.universe, [0.0, 0.0, 0.05, 0.20])
posicion['med_izq']  = fuzz.trimf(posicion.universe,  [0.05, 0.20, 0.35])
posicion['poco_izq'] = fuzz.trimf(posicion.universe,  [0.20, 0.35, 0.50])
posicion['centro']   = fuzz.trimf(posicion.universe,  [0.35, 0.50, 0.65])
posicion['poco_der'] = fuzz.trimf(posicion.universe,  [0.50, 0.65, 0.80])
posicion['med_der']  = fuzz.trimf(posicion.universe,  [0.65, 0.80, 0.95])
posicion['muy_der']  = fuzz.trapmf(posicion.universe, [0.80, 0.95, 1.0, 1.0])
```

### Sintaxis explicada

```python
posicion['nombre_conjunto'] = fuzz.trimf(posicion.universe, [a, b, c])
```

- `posicion['nombre']` — Asigna un conjunto difuso a la variable `posicion` con la etiqueta `'nombre'`
- `posicion.universe` — El array de puntos del universo (el que se creo con `np.linspace`)
- `fuzz.trimf(...)` — Retorna un array del mismo tamano que el universo, con el grado de pertenencia calculado para cada punto

### 5.2 Tabla de conjuntos de entrada

| # | Nombre | Tipo | Parametros | Pico | Rango efectivo |
|---|--------|------|-----------|------|----------------|
| 1 | `muy_izq` | Trapezoidal | [0.0, 0.0, 0.05, 0.20] | 0.0 - 0.05 (meseta) | 0.0 - 0.20 |
| 2 | `med_izq` | Triangular | [0.05, 0.20, 0.35] | 0.20 | 0.05 - 0.35 |
| 3 | `poco_izq` | Triangular | [0.20, 0.35, 0.50] | 0.35 | 0.20 - 0.50 |
| 4 | `centro` | Triangular | [0.35, 0.50, 0.65] | 0.50 | 0.35 - 0.65 |
| 5 | `poco_der` | Triangular | [0.50, 0.65, 0.80] | 0.65 | 0.50 - 0.80 |
| 6 | `med_der` | Triangular | [0.65, 0.80, 0.95] | 0.80 | 0.65 - 0.95 |
| 7 | `muy_der` | Trapezoidal | [0.80, 0.95, 1.0, 1.0] | 0.95 - 1.0 (meseta) | 0.80 - 1.0 |

### 5.3 Distribucion y traslape

```
mu
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

**Espaciado entre picos:** 0.15 (uniforme)

**Traslape:** Cada conjunto se traslapa con sus vecinos inmediatos. Esto garantiza que para cualquier valor de entrada, al menos un conjunto (y frecuentemente dos) tengan grado de pertenencia > 0. Sin traslape, habria "huecos" donde ninguna regla se activa.

**Por que 7 conjuntos:** Con 7 conjuntos se obtiene una granularidad suficiente para distinguir entre correcciones finas (poco_izq, poco_der), medias (med_izq, med_der) y extremas (muy_izq, muy_der), produciendo un control suave.

---

## 6. Conjuntos Difusos de Salida (7 conjuntos)

### 6.1 Definicion en codigo

```python
servo['muy_izq']  = fuzz.trimf(servo.universe, [65,  65,  81])
servo['med_izq']  = fuzz.trimf(servo.universe, [65,  81,  97])
servo['poco_izq'] = fuzz.trimf(servo.universe, [81,  97,  113])
servo['centro']   = fuzz.trimf(servo.universe, [97,  113, 129])
servo['poco_der'] = fuzz.trimf(servo.universe, [113, 129, 145])
servo['med_der']  = fuzz.trimf(servo.universe, [129, 145, 160])
servo['muy_der']  = fuzz.trimf(servo.universe, [145, 160, 160])
```

### 6.2 Tabla de conjuntos de salida

| # | Nombre | Parametros [a, b, c] | Centro (pico) | Rango efectivo |
|---|--------|---------------------|---------------|----------------|
| 1 | `muy_izq` | [65, 65, 81] | 65 | 65 - 81 |
| 2 | `med_izq` | [65, 81, 97] | 81 | 65 - 97 |
| 3 | `poco_izq` | [81, 97, 113] | 97 | 81 - 113 |
| 4 | `centro` | [97, 113, 129] | 113 | 97 - 129 |
| 5 | `poco_der` | [113, 129, 145] | 129 | 113 - 145 |
| 6 | `med_der` | [129, 145, 160] | 145 | 129 - 160 |
| 7 | `muy_der` | [145, 160, 160] | 160 | 145 - 160 |

**Nota:** Los conjuntos extremos (`muy_izq` con [65,65,81] y `muy_der` con [145,160,160]) son triangulares con el pico en el borde, creando efectivamente un medio triangulo.

**Espaciado entre centros:** 16 grados (uniforme)

```
    Muy    Med    Poco         Poco   Med    Muy
    Izq    Izq    Izq  Centro  Der    Der    Der
     ^      ^      ^     ^      ^      ^      ^
    / \    /  \   /  \   / \   /  \  /  \    / \
   /   \  /    \ /    \ /   \ /    \/    \  /   \
  /     \/      X      X     X      X     \/     \
 /      /\     / \    / \   / \    / \    /\      \
+------+---+--+----+-+-----++----+-+----+-+---+------+
65     81   97    113     129   145     160
```

---

## 7. Base de Reglas (7 reglas)

### 7.1 Definicion en codigo

```python
r1 = ctrl.Rule(posicion['muy_izq'],  servo['muy_izq'])
r2 = ctrl.Rule(posicion['med_izq'],  servo['med_izq'])
r3 = ctrl.Rule(posicion['poco_izq'], servo['poco_izq'])
r4 = ctrl.Rule(posicion['centro'],   servo['centro'])
r5 = ctrl.Rule(posicion['poco_der'], servo['poco_der'])
r6 = ctrl.Rule(posicion['med_der'],  servo['med_der'])
r7 = ctrl.Rule(posicion['muy_der'],  servo['muy_der'])
```

### Sintaxis de `ctrl.Rule`

```python
ctrl.Rule(antecedente, consecuente)
```

- **antecedente**: `posicion['nombre']` — condicion de entrada
- **consecuente**: `servo['nombre']` — accion de salida
- Internamente se lee: `IF posicion ES nombre THEN servo ES nombre`

### 7.2 Tabla de reglas con interpretacion fisica

| Regla | Condicion (IF) | Accion (THEN) | Significado |
|-------|---------------|---------------|-------------|
| R1 | posicion = muy_izq | servo = muy_izq (65) | Linea en extremo izquierdo -> giro maximo izquierdo |
| R2 | posicion = med_izq | servo = med_izq (81) | Linea a la izquierda -> giro medio izquierdo |
| R3 | posicion = poco_izq | servo = poco_izq (97) | Linea ligeramente izquierda -> correccion leve izq |
| R4 | posicion = centro | servo = centro (113) | Linea centrada -> mantener recto |
| R5 | posicion = poco_der | servo = poco_der (129) | Linea ligeramente derecha -> correccion leve der |
| R6 | posicion = med_der | servo = med_der (145) | Linea a la derecha -> giro medio derecho |
| R7 | posicion = muy_der | servo = muy_der (160) | Linea en extremo derecho -> giro maximo derecho |

### 7.3 Por que funciona con solo 7 reglas

El mapeo es **1 a 1** (cada conjunto de entrada corresponde a exactamente un conjunto de salida). La magia esta en el **traslape de los conjuntos**: cuando la posicion cae en una zona de traslape (por ejemplo, entre `poco_izq` y `centro`), **dos reglas se activan simultaneamente** con diferentes grados, y la defuzzificacion por centroide produce automaticamente un angulo intermedio. Esto genera la respuesta suave y continua sin necesidad de mas reglas.

---

## 8. Creacion del Sistema de Control

### 8.1 Codigo completo

```python
sistema = ctrl.ControlSystem([r1, r2, r3, r4, r5, r6, r7])
simulacion = ctrl.ControlSystemSimulation(sistema)
```

| Clase | Descripcion |
|---|---|
| `ctrl.ControlSystem` | Contenedor de las reglas. Define la **estructura** del sistema difuso |
| `ctrl.ControlSystemSimulation` | Motor de ejecucion. Permite **evaluar** entradas y obtener salidas |

### 8.2 Evaluacion (uso en cada frame)

```python
# 1. Asignar entrada
simulacion.input['posicion'] = posicion_norm    # valor entre 0.0 y 1.0

# 2. Ejecutar el pipeline completo
simulacion.compute()    # fuzzificacion -> reglas -> inferencia -> defuzzificacion

# 3. Leer salida
angulo = simulacion.output['servo']    # valor entre 65 y 160
```

`compute()` ejecuta internamente:
1. **Fuzzificacion**: evalua `posicion_norm` contra los 7 conjuntos de entrada
2. **Evaluacion de reglas**: determina el nivel de activacion de cada regla
3. **Recorte (clipping)**: recorta las funciones de membresia de salida al nivel de activacion
4. **Agregacion**: combina todas las salidas recortadas con operador MAX
5. **Defuzzificacion**: calcula el centroide del area agregada

---

## 9. Pipeline Completo: Fuzzificacion -> Inferencia -> Defuzzificacion

### 9.1 Fuzzificacion

Dado un valor de entrada (por ejemplo, `posicion_norm = 0.42`), se evalua contra los 7 conjuntos:

```
mu_muy_izq  = trapmf(0.42, [0.0, 0.0, 0.05, 0.20])  = 0.0
mu_med_izq  = trimf(0.42,  [0.05, 0.20, 0.35])       = 0.0
mu_poco_izq = trimf(0.42,  [0.20, 0.35, 0.50])       = 0.533
mu_centro   = trimf(0.42,  [0.35, 0.50, 0.65])       = 0.467
mu_poco_der = trimf(0.42,  [0.50, 0.65, 0.80])       = 0.0
mu_med_der  = trimf(0.42,  [0.65, 0.80, 0.95])       = 0.0
mu_muy_der  = trapmf(0.42, [0.80, 0.95, 1.0, 1.0])   = 0.0
```

**Resultado:** Dos reglas se activan — R3 (poco_izq) con fuerza 0.533 y R4 (centro) con fuerza 0.467.

### 9.2 Inferencia Mamdani

**Paso 1 — Recorte (clipping):**
Cada funcion de membresia de salida se recorta al nivel de activacion de su regla:

```
MF_recortada_R3(x) = min(trimf(x, [81, 97, 113]), 0.533)
MF_recortada_R4(x) = min(trimf(x, [97, 113, 129]), 0.467)
```

Los demas (R1, R2, R5, R6, R7) tienen activacion 0, asi que no contribuyen.

**Paso 2 — Agregacion:**
Se combinan todas las funciones recortadas usando el operador MAX (union difusa):

```
MF_agregada(x) = max(MF_recortada_R3(x), MF_recortada_R4(x))
```

### 9.3 Defuzzificacion — Centroide

Se calcula el centro de gravedad del area bajo la curva agregada:

```
            SUM(x_i * mu_agregada(x_i))
angulo = -----------------------------------
              SUM(mu_agregada(x_i))
```

Donde `x_i` recorre los 96 puntos del universo de salida (65 a 160).

Para el ejemplo (`posicion = 0.42`):
- R3 (poco_izq, centro=97) tiene mas fuerza (0.533) que R4 (centro, centro=113) con 0.467
- El centroide se desplaza hacia la izquierda de 113
- Resultado aproximado: **~104 grados** (ligeramente a la izquierda)

### 9.4 Comportamiento al perder la linea

Cuando la camara no detecta ningun contorno valido, el sistema **no usa el controlador difuso**. En su lugar, aplica una busqueda activa:

```python
if ultima_posicion < 0.5:
    angulo = 65     # giro maximo izquierda
else:
    angulo = 160    # giro maximo derecha
```

Esto gira el servo al maximo hacia el lado donde se vio la linea por ultima vez para intentar reencontrarla.

---

## 10. Resumen de funciones skfuzzy utilizadas

| Funcion / Clase | Modulo | Uso en el proyecto |
|---|---|---|
| `ctrl.Antecedent(universo, nombre)` | `skfuzzy.control` | Crear variable de entrada (posicion) |
| `ctrl.Consequent(universo, nombre, defuzzify_method)` | `skfuzzy.control` | Crear variable de salida (servo) |
| `fuzz.trimf(universo, [a, b, c])` | `skfuzzy` | Funcion de membresia triangular |
| `fuzz.trapmf(universo, [a, b, c, d])` | `skfuzzy` | Funcion de membresia trapezoidal |
| `ctrl.Rule(antecedente, consecuente)` | `skfuzzy.control` | Definir regla IF-THEN |
| `ctrl.ControlSystem(lista_reglas)` | `skfuzzy.control` | Crear sistema de control con las reglas |
| `ctrl.ControlSystemSimulation(sistema)` | `skfuzzy.control` | Crear motor de simulacion ejecutable |
| `simulacion.input['nombre'] = valor` | — | Asignar valor de entrada |
| `simulacion.compute()` | — | Ejecutar pipeline completo |
| `simulacion.output['nombre']` | — | Leer resultado de salida |
| `np.linspace(inicio, fin, puntos)` | `numpy` | Crear universo de discurso discretizado |
| `np.array([...])` | `numpy` | Crear arrays para rangos HSV (no difuso, pero relacionado) |

---

## 11. Codigo completo de la funcion

```python
def crear_sistema_difuso():
    """
    Crea el sistema de control difuso Mamdani.
    Entrada: posicion normalizada [0, 1]
    Salida: angulo del servo [65, 160] (simetrico, centro=113, +-48)
    """
    # --- Universos de discurso ---
    posicion = ctrl.Antecedent(np.linspace(0, 1, 1001), 'posicion')
    servo = ctrl.Consequent(np.linspace(65, 160, 96), 'servo', defuzzify_method='centroid')

    # --- 7 Conjuntos de entrada ---
    posicion['muy_izq']  = fuzz.trapmf(posicion.universe, [0.0, 0.0, 0.05, 0.20])
    posicion['med_izq']  = fuzz.trimf(posicion.universe, [0.05, 0.20, 0.35])
    posicion['poco_izq'] = fuzz.trimf(posicion.universe, [0.20, 0.35, 0.50])
    posicion['centro']   = fuzz.trimf(posicion.universe, [0.35, 0.50, 0.65])
    posicion['poco_der'] = fuzz.trimf(posicion.universe, [0.50, 0.65, 0.80])
    posicion['med_der']  = fuzz.trimf(posicion.universe, [0.65, 0.80, 0.95])
    posicion['muy_der']  = fuzz.trapmf(posicion.universe, [0.80, 0.95, 1.0, 1.0])

    # --- 7 Conjuntos de salida ---
    servo['muy_izq']  = fuzz.trimf(servo.universe, [65,  65,  81])
    servo['med_izq']  = fuzz.trimf(servo.universe, [65,  81,  97])
    servo['poco_izq'] = fuzz.trimf(servo.universe, [81,  97,  113])
    servo['centro']   = fuzz.trimf(servo.universe, [97,  113, 129])
    servo['poco_der'] = fuzz.trimf(servo.universe, [113, 129, 145])
    servo['med_der']  = fuzz.trimf(servo.universe, [129, 145, 160])
    servo['muy_der']  = fuzz.trimf(servo.universe, [145, 160, 160])

    # --- 7 Reglas ---
    r1 = ctrl.Rule(posicion['muy_izq'], servo['muy_izq'])
    r2 = ctrl.Rule(posicion['med_izq'], servo['med_izq'])
    r3 = ctrl.Rule(posicion['poco_izq'], servo['poco_izq'])
    r4 = ctrl.Rule(posicion['centro'], servo['centro'])
    r5 = ctrl.Rule(posicion['poco_der'], servo['poco_der'])
    r6 = ctrl.Rule(posicion['med_der'], servo['med_der'])
    r7 = ctrl.Rule(posicion['muy_der'], servo['muy_der'])

    # --- Construir sistema ---
    sistema = ctrl.ControlSystem([r1, r2, r3, r4, r5, r6, r7])
    return ctrl.ControlSystemSimulation(sistema)
```

### Como se usa en el bucle principal

```python
# Crear una vez al inicio
simulacion = crear_sistema_difuso()

# En cada frame:
posicion_norm = max(0.0, min(1.0, posicion_norm))  # clamp
simulacion.input['posicion'] = posicion_norm
simulacion.compute()
angulo = simulacion.output['servo']  # resultado: 65-160 grados
```

---

## 12. Diagrama de flujo del pipeline difuso

```
Posicion normalizada (0.0 - 1.0)
            |
            v
+---------------------------+
|      FUZZIFICACION        |
|  Evaluar contra 7 MFs     |
|  de entrada (trimf/trapmf)|
+---------------------------+
            |
    7 grados de pertenencia
    [mu1, mu2, ..., mu7]
            |
            v
+---------------------------+
|   EVALUACION DE REGLAS    |
|  R1: mu1 -> servo_muy_izq |
|  R2: mu2 -> servo_med_izq |
|  ...                       |
|  R7: mu7 -> servo_muy_der |
+---------------------------+
            |
    7 niveles de activacion
            |
            v
+---------------------------+
|    RECORTE (CLIPPING)     |
|  Cada MF de salida se     |
|  recorta al nivel de      |
|  activacion de su regla   |
+---------------------------+
            |
    7 MFs recortadas
            |
            v
+---------------------------+
|      AGREGACION           |
|  Combinar con operador    |
|  MAX (union difusa)       |
+---------------------------+
            |
    1 funcion agregada
            |
            v
+---------------------------+
|    DEFUZZIFICACION        |
|  Centroide (centro de     |
|  gravedad del area)       |
+---------------------------+
            |
            v
    Angulo del servo (65-160)
```

---

*Documento generado el 2026-05-26 — Version Final*
