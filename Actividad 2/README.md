# Actividad 2 - Consola de mandos ESP32 para Baxter

Práctica de la asignatura **Micros**: controlar en tiempo real el brazo del robot
**Baxter** (simulado en PyBullet) desde una consola de mandos con **ESP32** — 3
pulsadores para mover el efector final en X/Y/Z de forma continua ("jog"), y
abrir/cerrar la pinza para coger y mover un objeto.

El enunciado completo está en [`enunciado.md`](./enunciado.md). Basado en el ejemplo
[`baxter_ik_demo.py`](https://github.com/erwincoumans/pybullet_robots/blob/master/baxter_ik_demo.py)
del repositorio [pybullet_robots](https://github.com/erwincoumans/pybullet_robots).

## Idea general

El ESP32 lee 3 pulsadores y envía continuamente por serial la posición objetivo del
efector final y el estado deseado de la pinza:

```
POS,<x>,<y>,<z>,<grip>      grip: 1 = abierta, 0 = cerrada
```

En el PC, `baxter_consola.py` recibe cada mensaje, calcula la **cinemática inversa**
(IK) del brazo de Baxter hacia esa posición (igual que en `baxter_ik_demo.py`, con
`p.calculateInverseKinematics`) y controla la pinza. Se coloca un cubo pequeño sobre
una plataforma cerca del punto de inicio del brazo, para practicar tomarlo y moverlo.

```
┌────────────┐   USB / Serial    ┌────────────────────┐   PyBullet   ┌──────────────┐
│   ESP32    │ ────────────────▶ │  baxter_consola.py  │ ───────────▶ │ Brazo Baxter  │
│ (3 botones)│  POS,0.2,0,-0.1,1 │  (IK en tiempo real) │              │  + cubo       │
└────────────┘                   └────────────────────┘              └──────────────┘
```

## Contenido de la carpeta

| Archivo | Dónde corre | Qué hace |
|---|---|---|
| [`baxter_consola.py`](./baxter_consola.py) | PC (VS Code) | Lee la consola del ESP32 por serial, calcula IK, mueve el brazo y controla el agarre del cubo. |
| [`esp32_consola_baxter_3botones.ino`](./esp32_consola_baxter_3botones.ino) | ESP32 (Arduino IDE) | Lee los 3 pulsadores y envía la posición objetivo por serial. |
| [`evidencias/`](./evidencias) | — | Captura o video de la simulación funcionando. |

## Hardware: 3 pulsadores (sin joystick)

No se usó joystick; el control se hizo con 3 pulsadores en modo "jog", como el panel de
una impresora 3D:

| Botón | Pin ESP32 | Función |
|---|---|---|
| SELECCIONAR | GPIO 27 | Pulsación corta: cambia el eje activo (X → Y → Z → X...). Pulsación larga (>0.8 s): abre/cierra la pinza. |
| MENOS (–) | GPIO 26 | Mantenido: disminuye la coordenada del eje activo, de forma continua |
| MÁS (+) | GPIO 25 | Mantenido: la aumenta, de forma continua |

Cada botón va de su pin a GND; se usan las resistencias pull-up internas del ESP32
(`INPUT_PULLUP`), sin necesitar resistencias externas.

## Requisitos

- El mismo entorno `drones` de la Actividad 1 (ya tiene `pybullet` y `pyserial`
  instalados, por ser dependencias de `gym-pybullet-drones`).
- `git` instalado en el entorno conda (si se recreó el entorno, instalarlo de nuevo con
  `conda install git -y`).
- Clonar el repositorio con los modelos del robot Baxter:

```bash
conda activate drones
git clone https://github.com/erwincoumans/pybullet_robots.git
```

- Copiar `baxter_consola.py` **dentro** de la carpeta `pybullet_robots` (el script
  necesita la ruta relativa `baxter_common/baxter_description/urdf/toms_baxter.urdf`
  que trae ese repositorio).

## Ejecutar la práctica

Con el ESP32 conectado, el sketch `esp32_consola_baxter_3botones.ino` ya cargado, el
Monitor Serie **cerrado**, y el entorno `drones` activo:

```bash
cd pybullet_robots
python baxter_consola.py
```

Se abre la ventana de PyBullet con Baxter y un cubo pequeño sobre una plataforma gris,
cerca de donde arranca el brazo. Con SELECCIONAR eliges el eje (X, Y o Z), con MENOS/MÁS
te desplazas por él, y con una pulsación larga de SELECCIONAR abres o cierras la pinza.

## Cómo funciona el agarre del objeto

El brazo se mueve mediante IK "cinemática" (se recalculan y aplican los ángulos de las
articulaciones directamente en cada paso, sin depender solo de motores físicos). Esto es
ideal para posicionar el efector final con precisión, pero la fricción real entre los
dedos y el cubo no es suficiente para sostenerlo mientras el brazo se desplaza así.

Para resolverlo, cuando la pinza se cierra estando a menos de 6 cm del cubo, el script
crea un **acople rígido temporal** (`p.createConstraint`) entre la pinza y el cubo —
quedan unidos mientras la pinza esté cerrada, y se sueltan en cuanto se vuelve a abrir.
Es la técnica estándar en PyBullet para simular un agarre confiable en demos de este
tipo.

## Formato del mensaje serial

```
POS,<x>,<y>,<z>,<grip>
```

Ejemplo: `POS,0.200,0.000,-0.100,1` → mover el efector final a (0.2, 0.0, -0.1) m, con
la pinza abierta.

## Evidencia

_Agregar aquí una captura o video de la simulación funcionando, en `evidencias/`._

## Repositorio de referencia

- https://github.com/erwincoumans/pybullet_robots
- https://github.com/erwincoumans/pybullet_robots/blob/master/baxter_ik_demo.py
