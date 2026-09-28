"""
Paso 1 - Prueba de simulación (sin ESP32)
Mueve un dron Crazyflie simulado: despegue -> A -> B -> C.
Requiere: gym-pybullet-drones instalado (pip install -e .)
"""
import time
import numpy as np

from gym_pybullet_drones.envs.CtrlAviary import CtrlAviary
from gym_pybullet_drones.control.DSLPIDControl import DSLPIDControl
from gym_pybullet_drones.utils.enums import DroneModel, Physics
from gym_pybullet_drones.utils.utils import sync

# ---------------- Parámetros ----------------
DRONE = DroneModel.CF2X
PUNTOS = {            # coordenadas (x, y, z) en metros
    "A": [0.0, 0.0, 1.0],
    "B": [1.0, 1.0, 1.0],
    "C": [2.0, 0.0, 1.0],
}
TOLERANCIA = 0.10     # m: distancia para considerar que "llegó"
ESPERA_EN_PUNTO = 2.0 # s: tiempo que se queda en cada punto
DURACION_MAX = 60     # s: tope de seguridad de la simulación
GUI = True

# ---------------- Entorno y controlador ----------------
env = CtrlAviary(
    drone_model=DRONE,
    num_drones=1,
    initial_xyzs=np.array([[0.0, 0.0, 0.1]]),
    initial_rpys=np.zeros((1, 3)),
    physics=Physics.PYB,
    pyb_freq=240,
    ctrl_freq=48,
    gui=GUI,
    record=False,
)
ctrl = DSLPIDControl(drone_model=DRONE)

obs, info = env.reset()
nombres = list(PUNTOS.keys())
idx = 0
objetivo = np.array(PUNTOS[nombres[idx]], dtype=float)
t_llegada = None
action = np.zeros((1, 4))
inicio = time.time()

print(f"Yendo al punto {nombres[idx]}: {objetivo}")

# ---------------- Lazo principal ----------------
for i in range(int(DURACION_MAX * env.CTRL_FREQ)):
    obs, reward, terminated, truncated, info = env.step(action)

    t_sim = i / env.CTRL_FREQ
    pos = obs[0][0:3]

    # ¿Llegó al punto actual?
    if np.linalg.norm(pos - objetivo) < TOLERANCIA and t_llegada is None:
        t_llegada = t_sim
        print(f"  Llegó a {nombres[idx]} (t = {t_sim:.1f} s)")

    # Después de esperar, pasa al siguiente punto
    if t_llegada is not None and (t_sim - t_llegada) > ESPERA_EN_PUNTO:
        if idx < len(nombres) - 1:
            idx += 1
            objetivo = np.array(PUNTOS[nombres[idx]], dtype=float)
            t_llegada = None
            print(f"Yendo al punto {nombres[idx]}: {objetivo}")
        else:
            print("Recorrido A -> B -> C completado.")
            break

    # Controlador PID: calcula las RPM de los 4 motores
    action[0, :], _, _ = ctrl.computeControlFromState(
        control_timestep=env.CTRL_TIMESTEP,
        state=obs[0],
        target_pos=objetivo,
        target_rpy=np.zeros(3),
    )

    if GUI:
        sync(i, inicio, env.CTRL_TIMESTEP)  # mantiene la simulación en tiempo real

env.close()
