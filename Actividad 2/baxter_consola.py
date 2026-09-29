"""
Actividad 2 - Consola de mandos ESP32 para el robot Baxter (simulado)
Basado en baxter_ik_demo.py de https://github.com/erwincoumans/pybullet_robots

Requisitos:
    - pybullet y pyserial ya instalados (mismo entorno "drones" de la Actividad 1)
    - Clonar el repo con los modelos del robot:
          git clone https://github.com/erwincoumans/pybullet_robots.git
      y ejecutar este script DESDE DENTRO de esa carpeta (o ajustar URDF_BAXTER abajo
      con la ruta completa a "baxter_common/baxter_description/urdf/toms_baxter.urdf").

Qué hace:
    - El ESP32 envía continuamente por serial la posición objetivo del efector final
      y el estado deseado de la pinza:
          POS,<x>,<y>,<z>,<grip>      grip: 1 = abierta, 0 = cerrada
    - Este script usa cinemática inversa (IK) para mover el brazo de Baxter hacia esa
      posición en tiempo real y abre/cierra la pinza según el botón del ESP32.
    - Se coloca un cubo pequeño cerca del punto de inicio para practicar "coger y mover"
      el objeto cerrando la pinza sobre él.

Ejecutar (con el entorno drones activo y el ESP32 conectado):
    python baxter_consola.py
Detener con Ctrl+C.
"""
import time
from time import sleep

import numpy as np
import pybullet as p
import pybullet_data
import serial
import serial.tools.list_ports

# ---------------- Parámetros ----------------
PUERTO = None                 # ej. "COM5". Con None intenta detectarlo solo
BAUDIOS = 115200
URDF_BAXTER = "baxter_common/baxter_description/urdf/toms_baxter.urdf"
END_EFFECTOR_ID = 48          # link "left gripper left finger" (según baxter_ik_demo.py)

LIMITE_XY = 0.6                # m: recorte de seguridad de lo recibido por serial
Z_MIN, Z_MAX = -0.5, 0.5

GRIP_ABIERTO = 0.06            # pinza más abierta -> más margen para acercarse al cubo
GRIP_CERRADO = 0.0             # posición articular de "pinza cerrada"
GRIP_FUERZA = 80               # fuerza del motor de la pinza (antes 20) -> agarre más firme

POS_INICIAL_OBJETO = [0.2, 0.0, -0.18]   # justo debajo del punto de arranque del brazo
ESCALA_CUBO = 0.6                        # cubo más pequeño -> más margen para no chocarlo al acercarse


# ---------------- Conexión con el ESP32 ----------------
def abrir_serial():
    puertos = list(serial.tools.list_ports.comports())
    if PUERTO is not None:
        return serial.Serial(PUERTO, BAUDIOS, timeout=0)
    for puerto in puertos:
        desc = (puerto.description or "").upper()
        if any(k in desc for k in ("CP210", "CH340", "CH910", "USB SERIAL", "USB-SERIAL", "SILICON LABS")):
            print(f"Usando puerto {puerto.device} ({puerto.description})")
            return serial.Serial(puerto.device, BAUDIOS, timeout=0)
    print("No pude detectar el ESP32. Puertos disponibles:")
    for puerto in puertos:
        print(f"  {puerto.device}: {puerto.description}")
    raise SystemExit("Pon el puerto correcto en la variable PUERTO (ej. 'COM5').")


def parsear(linea):
    """Devuelve (np.array([x, y, z]), abierto: bool) o None si la línea no es válida."""
    inicio = linea.find("POS,")
    if inicio < 0:
        return None
    partes = linea[inicio:].strip().split(",")
    if len(partes) != 5 or partes[0] != "POS":
        return None
    try:
        x, y, z, grip = float(partes[1]), float(partes[2]), float(partes[3]), int(partes[4])
    except ValueError:
        return None
    x = float(np.clip(x, -LIMITE_XY, LIMITE_XY))
    y = float(np.clip(y, -LIMITE_XY, LIMITE_XY))
    z = float(np.clip(z, Z_MIN, Z_MAX))
    return np.array([x, y, z]), bool(grip)


# ---------------- Mundo y robot (adaptado de baxter_ik_demo.py) ----------------
def set_up_world(initial_sim_steps=100):
    p.resetSimulation()
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.loadURDF("plane.urdf", [0, 0, -1], useFixedBase=True)
    sleep(0.1)
    p.configureDebugVisualizer(p.COV_ENABLE_RENDERING, 0)
    baxter_id = p.loadURDF(URDF_BAXTER, useFixedBase=True)
    p.resetBasePositionAndOrientation(baxter_id, [0.5, -0.8, 0.0], [0., 0., -1., -1.])

    # El piso real queda muy por debajo de la zona donde se mueve el brazo (target ~ -0.1 a -0.3),
    # así que sin apoyo el cubo caería en caída libre. Se crea una pequeña plataforma fija justo
    # debajo del punto donde debe quedar el cubo, para que repose ahí sin caer.
    CUBO_MEDIO_ALTO = 0.025 * ESCALA_CUBO   # cube_small.urdf mide ~5 cm de lado a escala 1.0
    PLATAFORMA_MEDIO_ALTO = 0.01
    plataforma_top_z = POS_INICIAL_OBJETO[2] - CUBO_MEDIO_ALTO
    plataforma_col = p.createCollisionShape(
        p.GEOM_BOX, halfExtents=[0.06, 0.06, PLATAFORMA_MEDIO_ALTO]
    )
    plataforma_vis = p.createVisualShape(
        p.GEOM_BOX, halfExtents=[0.06, 0.06, PLATAFORMA_MEDIO_ALTO], rgbaColor=[0.6, 0.6, 0.6, 1]
    )
    p.createMultiBody(
        baseMass=0,
        baseCollisionShapeIndex=plataforma_col,
        baseVisualShapeIndex=plataforma_vis,
        basePosition=[POS_INICIAL_OBJETO[0], POS_INICIAL_OBJETO[1], plataforma_top_z - PLATAFORMA_MEDIO_ALTO],
    )
    objeto_id = p.loadURDF("cube_small.urdf", POS_INICIAL_OBJETO, globalScaling=ESCALA_CUBO)
    p.changeDynamics(objeto_id, -1, lateralFriction=1.5)   # más fricción -> más fácil sostenerlo
    p.configureDebugVisualizer(p.COV_ENABLE_RENDERING, 1)
    p.setGravity(0., 0., -10.)
    for _ in range(initial_sim_steps):
        p.stepSimulation()
    return baxter_id, objeto_id


def encontrar_joints_gripper(body_id):
    """Busca por nombre las articulaciones de la pinza (dedos)."""
    indices = []
    for i in range(p.getNumJoints(body_id)):
        info = p.getJointInfo(body_id, i)
        nombre = info[1].decode("utf-8").lower()
        if "finger" in nombre or "gripper" in nombre:
            indices.append(i)
    return indices


def get_joint_ranges(body_id, include_fixed=False):
    lower_limits, upper_limits, joint_ranges, rest_poses = [], [], [], []
    for i in range(p.getNumJoints(body_id)):
        joint_info = p.getJointInfo(body_id, i)
        if include_fixed or joint_info[3] > -1:
            lower_limits.append(-2)
            upper_limits.append(2)
            joint_ranges.append(2)
            rest_poses.append(p.getJointState(body_id, i)[0])
    return lower_limits, upper_limits, joint_ranges, rest_poses


def accurate_ik(body_id, end_effector_id, target_position, gripper_joints=(), max_iter=5, threshold=1e-3):
    """Versión simplificada de accurateIK de baxter_ik_demo.py, con pocas iteraciones
    para no bloquear el lazo en tiempo real.

    IMPORTANTE: las articulaciones de la pinza (gripper_joints) se excluyen del reset,
    porque si no, cada iteración las "teletransporta" a lo que dio la IK y se pierde el
    cierre que aplica set_gripper() -> la pinza nunca sostiene el objeto con firmeza.
    """
    close_enough, it, dist = False, 0, 1e30
    num_joints = p.getNumJoints(body_id)
    joint_poses = None
    while not close_enough and it < max_iter:
        joint_poses = p.calculateInverseKinematics(body_id, end_effector_id, target_position)
        for i in range(num_joints):
            if i in gripper_joints:
                continue
            q_index = p.getJointInfo(body_id, i)[3]
            if q_index > -1:
                p.resetJointState(body_id, i, joint_poses[q_index - 7])
        new_pos = p.getLinkState(body_id, end_effector_id)[4]
        diff = [target_position[j] - new_pos[j] for j in range(3)]
        dist = np.sqrt(sum(d * d for d in diff))
        close_enough = dist < threshold
        it += 1
    return joint_poses


def set_motors(body_id, joint_poses):
    for i in range(p.getNumJoints(body_id)):
        q_index = p.getJointInfo(body_id, i)[3]
        if q_index > -1:
            p.setJointMotorControl2(
                bodyIndex=body_id,
                jointIndex=i,
                controlMode=p.POSITION_CONTROL,
                targetPosition=joint_poses[q_index - 7],
            )


def set_gripper(body_id, gripper_joints, abierto):
    objetivo = GRIP_ABIERTO if abierto else GRIP_CERRADO
    for j in gripper_joints:
        p.setJointMotorControl2(
            bodyIndex=body_id,
            jointIndex=j,
            controlMode=p.POSITION_CONTROL,
            targetPosition=objetivo,
            force=GRIP_FUERZA,
        )


# ---------------- Programa principal ----------------
def main():
    ser = abrir_serial()
    time.sleep(2)  # el ESP32 puede reiniciarse al abrir el puerto
    buffer = b""

    p.connect(p.GUI)
    p.resetDebugVisualizerCamera(2.0, 180, 0.0, [0.52, 0.2, np.pi / 4.0])

    baxter_id, objeto_id = set_up_world()
    gripper_joints = encontrar_joints_gripper(baxter_id)
    print("Articulaciones de la pinza detectadas:", gripper_joints)
    for j in gripper_joints:
        p.changeDynamics(baxter_id, j, lateralFriction=1.5)   # más fricción en los dedos

    target_position = [0.2, 0.0, -0.1]
    abierto = True
    p.addUserDebugText("TARGET", target_position, textColorRGB=[1, 0, 0], textSize=1.5)
    p.addUserDebugText("CUBO", POS_INICIAL_OBJETO, textColorRGB=[0, 0.6, 1], textSize=1.5)

    print("Simulación lista. Esperando la consola del ESP32 (Ctrl+C para salir)...")
    print(f"Consejo: el cubo está en {POS_INICIAL_OBJETO}. Baja el eje Z ~0.08 m y cierra la pinza.")

    ultimo_aviso = time.time()
    constraint_id = None          # id del "acople" pinza-cubo mientras está agarrado
    DISTANCIA_AGARRE = 0.06       # m: qué tan cerca debe estar la pinza del cubo para poder agarrarlo

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
                        target_position, abierto = resultado
                    elif texto:
                        print(f"[ESP32] {texto}")

            # 2) Cinemática inversa hacia el objetivo y control de la pinza
            joint_poses = accurate_ik(baxter_id, END_EFFECTOR_ID, target_position, gripper_joints)
            set_motors(baxter_id, joint_poses)
            set_gripper(baxter_id, gripper_joints, abierto)

            # 3) Agarre "mecánico": como el brazo se mueve por teletransporte (IK cinemática)
            # y no por física pura, la fricción real de los dedos no basta para sostener el
            # cubo mientras el brazo se desplaza. Se simula el agarre creando un acople rígido
            # temporal entre la pinza y el cubo mientras la pinza está cerrada y suficientemente
            # cerca; se suelta en cuanto se vuelve a abrir.
            ef_pos, ef_orn = p.getLinkState(baxter_id, END_EFFECTOR_ID)[4:6]
            distancia_cubo = np.linalg.norm(np.array(ef_pos) - np.array(p.getBasePositionAndOrientation(objeto_id)[0]))

            if not abierto and constraint_id is None and distancia_cubo < DISTANCIA_AGARRE:
                obj_pos, obj_orn = p.getBasePositionAndOrientation(objeto_id)
                inv_pos, inv_orn = p.invertTransform(ef_pos, ef_orn)
                rel_pos, rel_orn = p.multiplyTransforms(inv_pos, inv_orn, obj_pos, obj_orn)
                constraint_id = p.createConstraint(
                    parentBodyUniqueId=baxter_id,
                    parentLinkIndex=END_EFFECTOR_ID,
                    childBodyUniqueId=objeto_id,
                    childLinkIndex=-1,
                    jointType=p.JOINT_FIXED,
                    jointAxis=[0, 0, 0],
                    parentFramePosition=rel_pos,
                    childFramePosition=[0, 0, 0],
                    parentFrameOrientation=rel_orn,
                    childFrameOrientation=[0, 0, 0, 1],
                )
                print("Cubo agarrado.")
            elif abierto and constraint_id is not None:
                p.removeConstraint(constraint_id)
                constraint_id = None
                print("Cubo soltado.")

            # Aviso periódico de qué tan cerca está el efector final del cubo
            if time.time() - ultimo_aviso > 1.0:
                ultimo_aviso = time.time()
                estado = "ABIERTA" if abierto else "CERRADA"
                agarrado = " (agarrado)" if constraint_id is not None else ""
                print(f"Distancia al cubo: {distancia_cubo:.3f} m   |   Pinza: {estado}{agarrado}")

            p.stepSimulation()
    except KeyboardInterrupt:
        print("\nDetenido por el usuario.")
    finally:
        ser.close()
        p.disconnect()


if __name__ == "__main__":
    main()