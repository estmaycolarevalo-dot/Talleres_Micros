// ESP32 (Arduino IDE) - "cerebro" del control real-to-sim.
// Envía por USB/serial la posición objetivo del dron con el formato:
//     WP,<nombre>,<x>,<y>,<z>      Ejemplo: WP,B,1.0,1.0,1.0
// MODO_BOTON = true : cada vez que pulsas el botón BOOT (GPIO 0) envía el siguiente punto (A -> B -> C -> A ...)
// MODO_BOTON = false: envía el siguiente punto solo, cada INTERVALO_AUTO_MS milisegundos
const bool MODO_BOTON = true;
const unsigned long INTERVALO_AUTO_MS = 8000;
const int PIN_BOTON = 0; 
const int PIN_LED = 2; 
struct Punto {
  const char* nombre;
  float x, y, z;
};
Punto puntos[] = {
  {"A", 0.0, 0.0, 1.0},
  {"B", 1.0, 1.0, 1.0},
  {"C", 2.0, 0.0, 1.0},
};
const int N_PUNTOS = sizeof(puntos) / sizeof(puntos[0]);
int indice = 0;
unsigned long ultimoEnvio = 0;
void enviar(const Punto &p) {
  Serial.printf("WP,%s,%.1f,%.1f,%.1f\n", p.nombre, p.x, p.y, p.z);
  digitalWrite(PIN_LED, HIGH);
  delay(150);
  digitalWrite(PIN_LED, LOW);
}
void setup() {
  Serial.begin(115200);
  pinMode(PIN_BOTON, INPUT_PULLUP);
  pinMode(PIN_LED, OUTPUT);
  delay(2000);
  Serial.println("ESP32 listo");
}
void loop() {
  if (MODO_BOTON) {
    if (digitalRead(PIN_BOTON) == LOW) {
      enviar(puntos[indice]);
      indice = (indice + 1) % N_PUNTOS;
      while (digitalRead(PIN_BOTON) == LOW) {
        delay(20);
      }
      delay(200);
    }
  } else if (millis() - ultimoEnvio >= INTERVALO_AUTO_MS) {
    ultimoEnvio = millis();
    enviar(puntos[indice]);
    indice = (indice + 1) % N_PUNTOS;
  }
  delay(10);
}
