# Actividad 1 - Real-to-Sim (drones + ESP32)

Práctica de la asignatura **Micros**: mover un dron simulado (gym-pybullet-drones) por
tres puntos A → B → C, donde el control de la secuencia se gestiona desde una **ESP32**.

El enunciado completo está en [`enunciado.md`](./enunciado.md).

## Idea general

El ESP32 actúa como el "cerebro" que decide a qué punto debe ir el dron. No controla un
dron real: envía por USB/serial el punto objetivo (`WP,<nombre>,<x>,<y>,<z>`) hacia un
script de Python que corre en el PC y mueve el dron dentro del simulador
[gym-pybullet-drones](https://github.com/utiasDSL/gym-pybullet-drones).

```
┌────────────┐   USB / Serial   ┌──────────────────────┐   PyBullet   ┌───────────────┐
│   ESP32    │ ───────────────▶ │  puente_esp32.py      │ ───────────▶ │  Dron simulado │
│ (control)  │  WP,B,1.0,1.0,1.0│  (PC, entorno drones)  │              │  (gym-pybullet) │
└────────────┘                  └──────────────────────┘              └───────────────┘
```

- Al presionar el botón **BOOT** de la placa, el ESP32 envía el siguiente punto de la
  secuencia (A → B → C → A → ...).
- `puente_esp32.py` escucha el puerto serial, interpreta el mensaje y se lo pasa al
  controlador PID del simulador como posición objetivo.

## Contenido de la carpeta

| Archivo | Dónde corre | Qué hace |
|---|---|---|
| [`sim_abc_prueba.py`](./sim_abc_prueba.py) | PC (VS Code) | Prueba base: mueve el dron A → B → C **sin** ESP32, para validar que el simulador funciona. |
| [`puente_esp32.py`](./puente_esp32.py) | PC (VS Code) | Lee los mensajes del ESP32 por serial y mueve el dron simulado según el punto recibido. |
| [`esp32/esp32_arduino/esp32_arduino.ino`](./esp32/esp32_arduino/esp32_arduino.ino) | ESP32 (Arduino IDE) | Envía A/B/C por serial al presionar BOOT (o en modo automático). |
| [`esp32/esp32_micropython/main.py`](./esp32/esp32_micropython/main.py) | ESP32 (Thonny / MicroPython) | Versión equivalente en MicroPython, mismo formato de mensajes. |
| [`docs/capturas/`](./docs/capturas) | — | Capturas de la simulación funcionando. |

Solo se necesita **una** de las dos versiones del ESP32 (Arduino o MicroPython), según lo
que use tu placa. En este taller se usó la versión de **Arduino IDE**.

## Requisitos

- Python 3.12 (el repo `gym-pybullet-drones` no soporta 3.10 en su versión actual).
- [Miniconda](https://www.anaconda.com/download) (recomendado en Windows, evita problemas
  al compilar `pybullet`).
- Arduino IDE (o Thonny, si se usa MicroPython).
- Placa ESP32 y cable USB.

## Instalación (PC)

```bash
conda create -n drones python=3.12 -y
conda activate drones

git clone https://github.com/utiasDSL/gym-pybullet-drones.git
cd gym-pybullet-drones
pip install --upgrade pip
pip install -e .
pip install pyserial
cd ..
```

Si `pip install -e .` falla compilando `pybullet`, instalarlo primero desde conda-forge:

```bash
conda install -c conda-forge pybullet -y
pip install -e .
```

Prueba que el simulador funciona, sin el ESP32:

```bash
python sim_abc_prueba.py
```

Debe abrirse la ventana de PyBullet con el dron despegando y visitando A, B y C.

## Cargar el código en el ESP32

### Opción A: Arduino IDE (la usada en este taller)

1. Abrir `esp32/esp32_arduino/esp32_arduino.ino`.
2. En **Herramientas → Placa** elegir *ESP32 Dev Module* (o el modelo correspondiente).
3. En **Herramientas → Puerto** elegir el puerto COM de la placa.
4. Subir el sketch. Si se queda en `Connecting...`, mantener presionado el botón **BOOT**.
5. Cerrar el Monitor Serie / Arduino IDE antes de ejecutar el puente en Python (si no,
   el puerto queda ocupado).

### Opción B: MicroPython (Thonny)

1. Instalar MicroPython en la placa desde
   **Herramientas → Instalar o actualizar MicroPython (esptool)**.
2. Abrir `esp32/esp32_micropython/main.py` y guardarlo en la placa con
   **Archivo → Guardar como → Dispositivo MicroPython**, con el nombre `main.py`.
3. Cerrar Thonny antes de ejecutar el puente en Python.

## Ejecutar la práctica completa

Con el ESP32 conectado y el entorno `drones` activo:

```bash
conda activate drones
python puente_esp32.py
```

Se abre la ventana del simulador. Cada vez que se presiona **BOOT** en el ESP32, el dron
se mueve al siguiente punto de la secuencia A → B → C. `Ctrl+C` para terminar.

## Formato del mensaje serial

```
WP,<nombre>,<x>,<y>,<z>
```

Ejemplo: `WP,B,1.0,1.0,1.0` → mover el dron al punto B, en (1.0, 1.0, 1.0) m.

Los tres puntos usados en esta práctica:

| Punto | x (m) | y (m) | z (m) |
|---|---|---|---|
| A | 0.0 | 0.0 | 1.0 |
| B | 1.0 | 1.0 | 1.0 |
| C | 2.0 | 0.0 | 1.0 |

## Evidencia

![Simulación de drones en PyBullet](./docs/capturas/simulacion_drones.png)

## Repositorio de referencia

- Simulador: https://github.com/utiasDSL/gym-pybullet-drones
