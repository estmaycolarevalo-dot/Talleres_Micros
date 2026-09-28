"""
ESP32 (MicroPython) - "cerebro" del control real-to-sim.
Guardar en el ESP32 con el nombre main.py (Thonny: Archivo > Guardar como > Dispositivo MicroPython).

Envía por USB/serial la posición objetivo del dron con el formato:
    WP,<nombre>,<x>,<y>,<z>
Ejemplo: WP,B,1.0,1.0,1.0

MODO = "boton": cada vez que pulsas el botón BOOT (GPIO 0) envía el siguiente punto (A -> B -> C -> A ...)
MODO = "auto" : envía el siguiente punto solo, cada INTERVALO_AUTO_S segundos
"""
from machine import Pin
import time

# ---------------- Configuración ----------------
PUNTOS = [            # (nombre, x, y, z) en metros
    ("A", 0.0, 0.0, 1.0),
    ("B", 1.0, 1.0, 1.0),
    ("C", 2.0, 0.0, 1.0),
]
MODO = "boton"        # "boton" o "auto"
INTERVALO_AUTO_S = 8  # solo se usa en modo "auto"

boton = Pin(0, Pin.IN, Pin.PULL_UP)  # botón BOOT de la placa (activo en 0)
led = Pin(2, Pin.OUT)                # LED integrado en la mayoría de DevKit


def enviar(punto):
    nombre, x, y, z = punto
    print("WP,{},{},{},{}".format(nombre, x, y, z))  # sale por el puerto serial
    led.value(1)
    time.sleep_ms(150)
    led.value(0)


time.sleep(2)  # da tiempo a que el PC abra el puerto
print("ESP32 listo. Modo:", MODO)

i = 0
while True:
    if MODO == "boton":
        if boton.value() == 0:                 # botón presionado
            enviar(PUNTOS[i])
            i = (i + 1) % len(PUNTOS)
            while boton.value() == 0:          # espera a que lo sueltes
                time.sleep_ms(20)
            time.sleep_ms(200)                 # antirrebote
    else:
        enviar(PUNTOS[i])
        i = (i + 1) % len(PUNTOS)
        time.sleep(INTERVALO_AUTO_S)
    time.sleep_ms(10)
