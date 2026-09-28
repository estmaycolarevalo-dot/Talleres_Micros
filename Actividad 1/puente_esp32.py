"""
Puente ESP32 -> simulación (gym-pybullet-drones).
Lee por serial las líneas  WP,<nombre>,<x>,<y>,<z>  que envía el ESP32
y las usa como posición objetivo del dron simulado.

Ejecutar desde la terminal con el entorno (drones) activo:
    python puente_esp32.py
Detener con Ctrl+C.

IMPORTANTE: cierra Thonny (o desconéctalo) antes de ejecutar, porque si no el puerto COM está ocupado.
"""
import time
import numpy as np
import serial
import serial.tools.list_ports

from gym_pybullet_drones.envs.CtrlAviary import CtrlAviary
from gym_pybullet_drones.control.DSLPIDControl import DSLPIDControl
from gym_pybullet_drones.utils.enums import DroneModel, Physics
from gym_pybullet_drones.utils.utils import sync

# ---------------- Parámetros ----------------
PUERTO = None          # ej. "COM5". Con None intenta detectarlo solo
BAUDIOS = 115200
DRONE = DroneModel.CF2X
LIMITE_XY = 3.0        # m: seguridad, recorta las coordenadas recibidas
Z_MIN, Z_MAX = 0.2, 2.5


def abrir_serial():
    puertos = list(serial.tools.list_ports.comports())
    if PUERTO is not None:
        return serial.Serial(PUERTO, BAUDIOS, timeout=0)
    for p in puertos:
        desc = (p.description or "").upper()
        if any(k in desc for k in ("CP210", "CH340", "CH910", "USB SERIAL", "USB-SERIAL", "SILICON LABS")):
            print(f"Usando puerto {p.device} ({p.description})")
            return serial.Serial(p.device, BAUDIOS, timeout=0)
    print("No pude detectar el ESP32. Puertos disponibles:")
    for p in puertos:
        print(f"  {p.device}: {p.description}")
    raise SystemExit("Pon el puerto correcto en la variable PUERTO (ej. 'COM5').")


def parsear(linea):
    """Devuelve (nombre, np.array([x, y, z])) o None si la línea no es un WP válido."""
    inicio_msg = linea.find("WP,")       # ignora basura que llegue antes del mensaje
    if inicio_msg < 0:
        return None
    partes = linea[inicio_msg:].strip().split(",")
    if len(partes) != 5 or partes[0] != "WP":
        return None
    try:
        x, y, z = float(partes[2]), float(partes[3]), float(partes[4])
    except ValueError:
        return None
    x = float(np.clip(x, -LIMITE_XY, LIMITE_XY))
    y = float(np.clip(y, -LIMITE_XY, LIMITE_XY))
    z = float(np.clip(z, Z_MIN, Z_MAX))
    return partes[1], np.array([x, y, z])


ser = abrir_serial()
time.sleep(2)  # al abrir el puerto el ESP32 puede reiniciarse; esperamos que arranque
buffer = b""

env = CtrlAviary(
    drone_model=DRONE,
    num_drones=1,
    initial_xyzs=np.array([[0.0, 0.0, 0.1]]),
    initial_rpys=np.zeros((1, 3)),
    physics=Physics.PYB,
    pyb_freq=240,
    ctrl_freq=48,
    gui=True,
    record=False,
)
ctrl = DSLPIDControl(drone_model=DRONE)

obs, info = env.reset()
objetivo = np.array([0.0, 0.0, 1.0])   # despega y espera la primera orden del ESP32
action = np.zeros((1, 4))
inicio = time.time()
i = 0
print("Simulación lista. Esperando órdenes del ESP32 (Ctrl+C para salir)...")

try:
    while True:
        # 1) Leer del ESP32 sin bloquear la simulación
        if ser.in_waiting:
            buffer += ser.read(ser.in_waiting)
            while b"\n" in buffer:
                linea, buffer = buffer.split(b"\n", 1)
                texto = linea.decode(errors="ignore").strip()
                resultado = parsear(texto)
                if resultado is not None:
                    nombre, objetivo = resultado
                    print(f"ESP32 -> punto {nombre}: {objetivo}")
                elif texto:
                    print(f"[ESP32] {texto}")   # mensajes informativos (arranque, etc.)

        # 2) Avanzar la simulación y controlar el dron hacia el objetivo
        obs, reward, terminated, truncated, info = env.step(action)
        action[0, :], _, _ = ctrl.computeControlFromState(
            control_timestep=env.CTRL_TIMESTEP,
            state=obs[0],
            target_pos=objetivo,
            target_rpy=np.zeros(3),
        )
        sync(i, inicio, env.CTRL_TIMESTEP)
        i += 1
except KeyboardInterrupt:
    print("\nDetenido por el usuario.")
finally:
    env.close()
    ser.close()
