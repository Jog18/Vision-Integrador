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

#### 3.1.1 Por que la entrada se normaliza a [0, 1] y no se usan pixeles directamente

La camara captura imagenes de 640 pixeles de ancho. Cuando OpenCV detecta la linea blanca, calcula el centroide (el punto central) del contorno y obtiene su coordenada horizontal `cx`, que es un numero entero entre 0 y 640.

**El problema de usar pixeles directamente:**

Si los conjuntos difusos de entrada estuvieran definidos en pixeles, tendriamos algo como:

```python
# EJEMPLO HIPOTETICO — NO es lo que se usa
posicion['centro'] = fuzz.trimf(universo, [224, 320, 416])
```

Esto funcionaria **solo** para una resolucion de 640 pixeles de ancho. Si algun dia se cambiara la camara a una de 320 pixeles, o a 1280, habria que recalcular todos los parametros de todos los conjuntos difusos. El controlador estaria "amarrado" a una resolucion especifica.

**La solucion — normalizar:**

En lugar de pasar `cx` directamente (por ejemplo, 200), se divide entre el ancho del frame:

```
posicion_norm = cx / ancho_frame
```

Ejemplo paso a paso:
```
La linea esta en el pixel 200 de un frame de 640 pixeles de ancho.

posicion_norm = 200 / 640 = 0.3125

Ese 0.3125 es un numero entre 0.0 y 1.0 que significa:
"la linea esta al 31.25% del ancho del frame, contando desde la izquierda"
```

Ahora los conjuntos difusos se definen sobre el rango [0, 1], por ejemplo:
```python
posicion['centro'] = fuzz.trimf(universo, [0.35, 0.50, 0.65])
```

Ese `[0.35, 0.50, 0.65]` significa "el centro va del 35% al 65% del frame, con pico en el 50%". Esto funciona **sin importar** si el frame mide 320, 640 o 1920 pixeles de ancho.

#### 3.1.2 Que es el universo de discurso y por que tiene 1001 puntos

El **universo de discurso** es el conjunto de todos los valores posibles que la variable puede tomar. Para la posicion normalizada, el universo va de 0.0 a 1.0.

Ahora bien, una computadora no puede trabajar con **todos** los numeros reales entre 0 y 1 (son infinitos). Necesita una version discretizada: una lista finita de puntos distribuidos uniformemente entre 0 y 1. Eso es lo que hace `np.linspace(0, 1, 1001)`:

```
np.linspace(0, 1, 1001) genera:
[0.000, 0.001, 0.002, 0.003, ..., 0.998, 0.999, 1.000]
 ^^^^                                              ^^^^
 punto 1                                        punto 1001

Paso entre puntos: 1/1000 = 0.001
```

Skfuzzy usa estos 1001 puntos internamente para:
1. **Evaluar las funciones de membresia:** calcula el grado de pertenencia en cada uno de los 1001 puntos.
2. **Calcular el centroide:** para la defuzzificacion, necesita sumar areas, y lo hace punto por punto sobre esta rejilla.

**Por que 1001 y no otro numero:**

La camara tiene 640 pixeles de ancho, asi que las posiciones reales posibles despues de normalizar son:

```
cx = 0    -> 0/640 = 0.00000
cx = 1    -> 1/640 = 0.00156
cx = 2    -> 2/640 = 0.00313
...
cx = 320  -> 320/640 = 0.50000
...
cx = 640  -> 640/640 = 1.00000

Total: 641 valores posibles, separados por 1/640 ≈ 0.00156
```

El universo de 1001 puntos tiene un paso de 0.001, que es **mas fino** que el paso real de la camara (0.00156):

```
Resolucion de la camara:      1/640  = 0.00156  (641 valores posibles)
Resolucion del universo:      1/1000 = 0.00100  (1001 puntos)
                                        ^^^^^^
                        El universo es mas fino que la camara
```

Esto significa que el universo difuso tiene mas puntos que los que la camara puede realmente producir. Nunca se pierde precision por culpa de la discretizacion del universo — la limitacion de precision siempre viene de la camara, no del sistema difuso.

**Podria usarse otro numero de puntos?**

Si. Aqui una comparacion:

| Puntos | Paso | Resultado |
|---|---|---|
| 11 | 0.100 | Muy grueso. Perderia matices entre conjuntos. El centroide seria impreciso. |
| 101 | 0.010 | Aceptable. Funcionaria bien en la practica. |
| 641 | 0.00156 | Coincide exactamente con la resolucion de la camara. Valido. |
| **1001** | **0.001** | **Mas fino que la camara. Garantiza que el universo nunca sea el cuello de botella.** |
| 10001 | 0.0001 | Innecesariamente fino. Consumiria mas memoria y CPU sin beneficio. |

Se eligio 1001 porque:
1. Da un paso limpio de exactamente 0.001 (facil de razonar y depurar).
2. Es mas fino que la resolucion real de la camara (0.00156), asi que nunca limita la precision.
3. No es excesivo — 1001 floats ocupan ~8 KB de memoria, que es insignificante.
4. Es una convencion comun en sistemas difusos con universos normalizados [0, 1].

En resumen: **1001 no tiene relacion con el ancho del frame**. El frame mide 640 pixeles, pero la posicion se normaliza a [0, 1] antes de entrar al sistema difuso. Los 1001 puntos son la rejilla interna que skfuzzy usa para hacer sus calculos, y se eligio ese numero para tener precision suficiente sin desperdiciar recursos.

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

## 13. Justificacion de las Decisiones de Diseno del Sistema Difuso

Esta seccion detalla el **por que** de cada decision tomada en el diseno del controlador difuso, respondiendo al criterio de evaluacion del proyecto integrador.

### 13.1 Por que logica difusa y no un controlador clasico (PID, ON-OFF)

| Criterio | ON-OFF | PID | Logica Difusa (Mamdani) |
|---|---|---|---|
| Suavidad de respuesta | Saltos bruscos entre estados | Suave, pero requiere modelo matematico | Suave, sin necesidad de modelo |
| Necesidad de modelo matematico | No | Si (funcion de transferencia) | No |
| Manejo de incertidumbre | No | Limitado | Excelente (grados de pertenencia) |
| Facilidad de ajuste intuitivo | Trivial pero limitado | Requiere sintonizacion Kp, Ki, Kd | Ajuste con terminos linguisticos naturales |
| Comportamiento en zonas de transicion | Oscilaciones | Puede tener overshoot | Transiciones graduales por traslape |

**Justificacion:** La posicion de la linea detectada por la camara tiene **ruido inherente** (variaciones de iluminacion, vibraciones del chasis, resoluciones de pixel). Un controlador ON-OFF produciria oscilaciones constantes. Un PID requeriria un modelo matematico de la cinematica Ackermann y sintonizacion experimental de 3 parametros. La logica difusa permite disenar el controlador usando **conocimiento intuitivo** ("si la linea esta un poco a la izquierda, gira un poco a la izquierda") sin necesidad de modelar la planta, y produce una respuesta suave de forma natural gracias al traslape de conjuntos y la defuzzificacion por centroide.

### 13.2 Por que el metodo de inferencia Mamdani (y no Sugeno o Tsukamoto)

| Metodo | Salida de reglas | Interpretabilidad | Complejidad computacional |
|---|---|---|---|
| **Mamdani** | Conjunto difuso (forma completa) | Alta — entrada y salida son conjuntos linguisticos | Media |
| Sugeno | Funcion lineal o constante | Baja — la salida es una ecuacion, no un termino linguistico | Baja |
| Tsukamoto | Valor crisp por funcion monotona | Media | Baja |

**Justificacion:** Se eligio Mamdani porque:

1. **Interpretabilidad total:** Tanto la entrada como la salida se expresan en terminos linguisticos (`IF posicion ES poco_izquierda THEN servo ES poco_izquierda`). Esto permite que cualquier miembro del equipo entienda y modifique las reglas sin conocimientos avanzados de control.
2. **Visualizacion clara:** El proceso de recorte y agregacion de las funciones de membresia de salida se puede graficar paso a paso, lo cual es valioso para la exposicion y depuracion.
3. **Compatibilidad con scikit-fuzzy:** La libreria `skfuzzy.control` implementa nativamente Mamdani con su API de alto nivel (`Antecedent`, `Consequent`, `Rule`), lo que simplifica la implementacion.
4. **Requisito del proyecto:** El objetivo explicito es disenar un controlador difuso con variables linguisticas de entrada y salida, lo cual es la definicion de Mamdani.

Sugeno seria mas eficiente computacionalmente, pero su salida es un numero (no un conjunto difuso), lo que pierde la interpretabilidad linguistica que el proyecto requiere demostrar.

### 13.3 Por que 7 conjuntos difusos (y no 3, 5 o 9)

| Cantidad | Granularidad | Reglas | Suavidad | Riesgo |
|---|---|---|---|---|
| 3 (izq, centro, der) | Baja | 3 | Pobre — saltos grandes entre correcciones | Oscilaciones en curvas suaves |
| 5 | Media | 5 | Aceptable | Puede faltar precision en extremos |
| **7** | **Alta** | **7** | **Excelente — distingue 3 niveles de correccion por lado** | **Ninguno significativo** |
| 9+ | Muy alta | 9+ | Marginalmente mejor que 7 | Complejidad innecesaria, mas reglas sin beneficio real |

**Justificacion:** Con 7 conjuntos se obtienen **3 niveles de correccion a cada lado** del centro:
- **Poco** (correccion leve): para mantener la linea cuando esta ligeramente desviada — produce movimientos suaves en rectas y curvas amplias.
- **Medio** (correccion moderada): para curvas de radio medio (~150 cm en la pista) — el AGV responde con giro apreciable pero controlado.
- **Muy** (correccion maxima): para curvas cerradas (~80 cm) o recuperacion cuando la linea se acerca al borde — giro al maximo mecanico del servo.

Con 3 o 5 conjuntos, el sistema no distinguiria entre una desviacion leve y una moderada, produciendo correcciones excesivas en rectas o insuficientes en curvas. Con 9 o mas, los conjuntos adicionales tendrian rangos tan estrechos que no aportarian diferencia practica, y el numero de reglas creceria sin beneficio.

La pista del proyecto tiene curvas de **80 cm y 150 cm de radio**, lo que requiere al menos 3 niveles de correccion distintos para manejar ambos radios adecuadamente.

### 13.4 Por que funciones triangulares y trapezoidales (y no gaussianas, campana o sigmoidales)

| Tipo de MF | Forma | Costo computacional | Control sobre los limites |
|---|---|---|---|
| **Triangular** | Lineal, pico puntual | Muy bajo (2 comparaciones) | Total — limites exactos definidos por [a, b, c] |
| **Trapezoidal** | Lineal con meseta | Muy bajo (3 comparaciones) | Total — meseta y limites definidos por [a, b, c, d] |
| Gaussiana | Campana suave | Mayor (exponencial) | Parcial — nunca llega exactamente a 0, cola infinita |
| Sigmoidal | S-curve | Mayor | Solo define un borde, no un rango completo |

**Justificacion:**

1. **Eficiencia computacional:** El controlador se ejecuta en cada frame de video (~30 fps) sobre una Raspberry Pi 4B. Las funciones triangulares y trapezoidales son operaciones lineales simples, lo que minimiza la carga de CPU y permite dedicar recursos al procesamiento de imagen (OpenCV).
2. **Limites exactos y predecibles:** Con `trimf` y `trapmf`, cada conjunto tiene un rango efectivo definido con precision. La pertenencia es exactamente 0 fuera de `[a, c]` o `[a, d]`. Con funciones gaussianas, las colas se extienden al infinito y un conjunto "muy_izquierda" tendria una pertenencia residual (pequena pero no nula) incluso en el extremo derecho, lo que activaria reglas irrelevantes.
3. **Traslape controlado:** Al definir los parametros de conjuntos adyacentes, se controla con precision cuanto se traslapan. Los conjuntos triangulares con el patron [0.20, 0.35, 0.50] y [0.35, 0.50, 0.65] generan un traslape exacto del 50% entre picos adyacentes, garantizando que siempre haya al menos un conjunto activo y como maximo dos.
4. **Los extremos usan trapezoidales:** Los conjuntos `muy_izq` [0.0, 0.0, 0.05, 0.20] y `muy_der` [0.80, 0.95, 1.0, 1.0] tienen una meseta en el borde del universo. Esto asegura que cualquier posicion en el extremo (0.0 a 0.05, o 0.95 a 1.0) tenga pertenencia maxima (1.0), evitando que el sistema "se quede sin respuesta" cuando la linea esta en el borde del frame.

### 13.5 Por que defuzzificacion por centroide (y no bisector, MOM, SOM, LOM)

| Metodo | Descripcion | Comportamiento |
|---|---|---|
| **Centroide (COG)** | Centro de gravedad del area agregada | Suave, considera la forma completa de la distribucion |
| Bisector (BOA) | Punto que divide el area en dos mitades iguales | Similar al centroide pero puede ser discontinuo |
| MOM (Mean of Maximum) | Promedio de los puntos con mayor pertenencia | Ignora la forma general, solo los picos |
| SOM / LOM | Menor / Mayor de los maximos | Produce saltos, favorece un lado |

**Justificacion:** El centroide es el metodo mas adecuado porque:

1. **Produce una salida continua y suave:** Al calcular el centro de gravedad de toda el area agregada, el centroide responde proporcionalmente a la contribucion de cada regla activa. Si dos reglas se activan con diferentes fuerzas, el resultado se desplaza gradualmente hacia la regla con mayor activacion. Esto es esencial para un servo de direccion donde los saltos bruscos causan inestabilidad mecanica.
2. **Sensible a cambios graduales de entrada:** Un cambio pequeno en la posicion de la linea produce un cambio proporcionalmente pequeno en el angulo de salida. Los metodos MOM/SOM/LOM pueden producir saltos discretos cuando la regla dominante cambia.
3. **Es el estandar de la industria para Mamdani:** La combinacion Mamdani + centroide es la configuracion mas documentada y validada en la literatura de control difuso.

### 13.6 Por que entrada normalizada [0, 1] (y no en pixeles)

**Justificacion:** La posicion de la linea se normaliza dividiendo `cx / ancho_frame` por las siguientes razones:

1. **Independencia de resolucion:** Si se cambia la resolucion de la camara (de 640x480 a 320x240, por ejemplo), el controlador difuso sigue funcionando sin modificar ningun parametro. Los conjuntos difusos siempre operan en [0, 1].
2. **Simetria natural:** El centro del frame siempre corresponde a 0.5, independientemente del ancho en pixeles. Esto simplifica el diseno simetrico de los conjuntos.
3. **Portabilidad:** El mismo sistema difuso podria reutilizarse con otra camara o resolucion sin recalibrar las funciones de membresia.

### 13.7 Por que rango de salida [65, 160] con centro en 113

**Justificacion:** Estos valores no son arbitrarios — provienen de las **restricciones mecanicas** del servo y la geometria Ackermann:

1. **Centro en 113 grados:** Este es el angulo donde las ruedas del chasis Ackermann 1:10 quedan perfectamente rectas. Se determino empiricamente montando el servo y midiendo la posicion neutra.
2. **Limite inferior 65 grados:** Es el angulo minimo que el servo puede alcanzar antes de que la geometria Ackermann alcance su tope mecanico (radio de giro minimo). Ir mas alla forzaria el mecanismo.
3. **Limite superior 160 grados:** Mismo principio, tope mecanico en la direccion opuesta.
4. **Simetria (113 - 65 = 48, 160 - 113 = 47):** El rango es practicamente simetrico (~48 grados a cada lado), lo que permite que los conjuntos de salida se distribuyan uniformemente con espaciado de 16 grados entre centros (95/6 ≈ 16).

### 13.8 Por que mapeo 1:1 en las reglas (y no reglas cruzadas)

El sistema usa un mapeo directo: cada conjunto de entrada corresponde exactamente a un conjunto de salida con el mismo nombre. No hay reglas cruzadas como "IF muy_izquierda THEN med_izquierda".

**Justificacion:**

1. **Principio de correspondencia directa:** Si la linea esta en una posicion extrema, la correccion debe ser extrema. Si esta ligeramente desviada, la correccion debe ser leve. El mapeo 1:1 refleja esta logica intuitiva directamente.
2. **Las transiciones suaves se logran por el traslape, no por las reglas:** Cuando la posicion cae entre dos conjuntos (por ejemplo, entre `poco_izq` y `centro`), ambas reglas se activan parcialmente. La defuzzificacion por centroide produce automaticamente un angulo intermedio. No se necesitan reglas adicionales para cubrir estas zonas de transicion.
3. **Simplicidad y mantenibilidad:** Con 7 reglas directas, el comportamiento del sistema es completamente predecible y facil de depurar. Agregar reglas cruzadas aumentaria la complejidad sin beneficio, ya que el traslape de conjuntos ya genera la interpolacion necesaria.
4. **Minimo necesario y suficiente:** Con solo 7 reglas, el sistema produce una curva de transferencia entrada-salida continua y monotona (a mayor posicion a la derecha, mayor angulo de giro a la derecha), que es exactamente el comportamiento deseado para un seguidor de linea.

### 13.9 Por que distribucion uniforme de conjuntos (espaciado constante)

Los picos de los conjuntos de entrada estan separados 0.15 unidades (0.05, 0.20, 0.35, 0.50, 0.65, 0.80, 0.95) y los de salida 16 grados (65, 81, 97, 113, 129, 145, 160).

**Justificacion:**

1. **Respuesta proporcional uniforme:** Con espaciado constante, la sensibilidad del controlador es la misma en todo el rango. Una desviacion de 0.15 a la izquierda produce la misma magnitud de correccion que 0.15 a la derecha. Esto es deseable porque la pista tiene curvas en ambas direcciones y el AGV debe responder simetricamente.
2. **Traslape uniforme:** Al mantener el mismo espaciado, todos los conjuntos adyacentes se traslapan en la misma proporcion (~50% de su ancho). Esto garantiza que la suavidad de las transiciones sea uniforme en todo el rango de operacion.
3. **Alternativa descartada — conjuntos mas densos en el centro:** Se considero concentrar mas conjuntos cerca del centro (0.5) para mayor precision en rectas. Se descarto porque la pista tiene curvas frecuentes de radio pequeno (80 cm) donde la linea puede llegar a los extremos del frame, y se necesita la misma granularidad de respuesta en todo el rango.

### 13.10 Por que una sola variable de entrada (y no dos o mas)

El sistema usa **una unica variable de entrada** (posicion horizontal normalizada). Se podria haber agregado una segunda variable como la velocidad, el area del contorno, o la pendiente de la linea.

**Justificacion:**

1. **Suficiencia de una variable:** Para un seguidor de linea a velocidad constante, la posicion horizontal de la linea en el frame contiene toda la informacion necesaria para calcular la correccion de direccion. La relacion es directa: posicion de la linea -> angulo de giro.
2. **Velocidad constante:** El motor DC opera a PWM fijo (50/255), por lo que la velocidad no es una variable que el controlador deba compensar. Si la velocidad fuera variable, se necesitaria una segunda entrada para ajustar la magnitud de la correccion.
3. **Explosion combinatoria:** Con 2 entradas de 7 conjuntos cada una, la base de reglas creceria a 7x7 = 49 reglas. Con 3 entradas, a 343 reglas. La complejidad crece exponencialmente sin beneficio proporcional para este caso de uso.
4. **Principio de parsimonia:** El diseno mas simple que resuelve el problema es el mejor. Agregar variables de entrada sin necesidad introduce complejidad, posibilidad de errores en las reglas, y mayor carga computacional.

### 13.11 Resumen de decisiones y alternativas descartadas

| Decision | Elegido | Alternativas descartadas | Razon principal |
|---|---|---|---|
| Tipo de controlador | Logica difusa | PID, ON-OFF | No requiere modelo matematico, maneja incertidumbre |
| Metodo de inferencia | Mamdani | Sugeno, Tsukamoto | Interpretabilidad linguistica total |
| Numero de conjuntos | 7 (por variable) | 3, 5, 9 | 3 niveles de correccion por lado, sin complejidad innecesaria |
| Forma de MFs | Triangular + Trapezoidal | Gaussiana, Campana, Sigmoidal | Eficiencia computacional, limites exactos |
| Defuzzificacion | Centroide (COG) | Bisector, MOM, SOM, LOM | Salida continua y suave, sensible a contribuciones parciales |
| Entrada | Posicion normalizada [0,1] | Posicion en pixeles | Independencia de resolucion |
| Salida | Angulo [65, 160], centro 113 | Otro rango | Restricciones mecanicas del servo Ackermann |
| Reglas | 7 reglas directas (1:1) | Reglas cruzadas, ponderadas | Simplicidad; el traslape genera interpolacion automatica |
| Distribucion | Uniforme (equidistante) | Concentrada al centro | Respuesta simetrica uniforme en todo el rango |
| Variables de entrada | 1 (posicion) | 2+ (posicion + velocidad/area) | Velocidad constante, una variable es suficiente |

---

*Documento generado el 2026-05-26 — Actualizado 2026-05-29 con justificacion de decisiones de diseno*
