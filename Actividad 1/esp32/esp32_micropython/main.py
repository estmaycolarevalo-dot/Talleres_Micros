"""
ESP32 (MicroPython) - "cerebro" del control real-to-sim.
Envía por USB/serial la posición objetivo del dron con el formato:
    WP,<nombre>,<x>,<y>,<z>
Ejemplo: WP,B,1.0,1.0,1.0
MODO = "boton": cada vez que pulsas el botón BOOT (GPIO 0) envía el siguiente punto (A -> B -> C -> A ...)
MODO = "auto" : envía el siguiente punto solo, cada INTERVALO_AUTO_S segundos
"""
from machine import Pin
import time
PUNTOS = [            
    ("A", 0.0, 0.0, 1.0),
    ("B", 1.0, 1.0, 1.0),
    ("C", 2.0, 0.0, 1.0),
]
MODO = "boton"        
INTERVALO_AUTO_S = 8  
boton = Pin(0, Pin.IN, Pin.PULL_UP)  
led = Pin(2, Pin.OUT)                
def enviar(punto):
    nombre, x, y, z = punto
    print("WP,{},{},{},{}".format(nombre, x, y, z))  
    led.value(1)
    time.sleep_ms(150)
    led.value(0)
time.sleep(2)  
print("ESP32 listo. Modo:", MODO)
i = 0
while True:
    if MODO == "boton":
        if boton.value() == 0:                 
            enviar(PUNTOS[i])
            i = (i + 1) % len(PUNTOS)
            while boton.value() == 0:          
                time.sleep_ms(20)
            time.sleep_ms(200)                
    else:
        enviar(PUNTOS[i])
        i = (i + 1) % len(PUNTOS)
        time.sleep(INTERVALO_AUTO_S)
    time.sleep_ms(10)
