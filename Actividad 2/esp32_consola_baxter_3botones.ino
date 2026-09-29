// ESP32 (Arduino IDE) - Consola de mandos para el brazo Baxter (Actividad 2)
// Versión con 3 PULSADORES (sin joystick), estilo "jog" (como el panel de una impresora 3D).
//
// Envía continuamente la posición objetivo del efector final y el estado de la pinza:
//     POS,<x>,<y>,<z>,<grip>      grip: 1 = abierta, 0 = cerrada
//
// Controles:
//   - Botón SELECCIONAR (pulsación corta) -> cambia el eje activo: X -> Y -> Z -> X ...
//   - Botón SELECCIONAR (mantenido > 800 ms) -> alterna abrir/cerrar la pinza
//   - Botón MENOS  (mantenido) -> disminuye la coordenada del eje activo, de forma continua
//   - Botón MAS    (mantenido) -> aumenta la coordenada del eje activo, de forma continua
//
// Ajusta los pines según tu montaje. Se usan resistencias pull-up internas: cada botón
// va de su pin a GND, y se lee en LOW cuando está presionado.

const int PIN_BTN_SELECCIONAR = 27;
const int PIN_BTN_MENOS = 26;
const int PIN_BTN_MAS = 25;

// Rango del objetivo en metros. Debe coincidir con los límites del script de Python
// (LIMITE_XY, Z_MIN, Z_MAX en baxter_consola.py).
const float RANGO_XY = 0.5;    // x, y quedan en [-0.5, 0.5]
const float Z_MIN = -0.3;
const float Z_MAX = 0.3;

const float VELOCIDAD = 0.15;           // m/s mientras se mantiene presionado + o -
const unsigned long PERIODO_CICLO_MS = 50;   // ~20 Hz: movimiento fluido
const unsigned long TIEMPO_PULSACION_LARGA_MS = 800;

unsigned long ultimoCiclo = 0;

// Posición actual del objetivo (empieza centrada, dentro del área de trabajo)
float pos[3] = {0.0, 0.0, 0.0};
int ejeActivo = 0;   // 0 = X, 1 = Y, 2 = Z
const char nombresEje[3] = {'X', 'Y', 'Z'};

bool gripAbierto = true;

// Antirrebote y detección de pulsación larga para el botón SELECCIONAR
bool estadoAnteriorSel = HIGH;
unsigned long inicioPulsacionSel = 0;
bool pulsacionLargaProcesada = false;
const unsigned long ANTIRREBOTE_MS = 40;
unsigned long ultimoCambioSel = 0;

float limiteMin(int eje) { return (eje == 2) ? Z_MIN : -RANGO_XY; }
float limiteMax(int eje) { return (eje == 2) ? Z_MAX : RANGO_XY; }

void setup() {
  Serial.begin(115200);
  pinMode(PIN_BTN_SELECCIONAR, INPUT_PULLUP);
  pinMode(PIN_BTN_MENOS, INPUT_PULLUP);
  pinMode(PIN_BTN_MAS, INPUT_PULLUP);
  delay(2000);   // da tiempo a que el PC abra el puerto
  Serial.println("ESP32 listo (consola Baxter, 3 pulsadores)");
}

void loop() {
  unsigned long ahora = millis();

  // ---------- Botón SELECCIONAR: corto = cambia eje, largo = pinza ----------
  bool sel = digitalRead(PIN_BTN_SELECCIONAR);
  if (sel != estadoAnteriorSel && (ahora - ultimoCambioSel) > ANTIRREBOTE_MS) {
    ultimoCambioSel = ahora;
    if (sel == LOW) {                      // se presionó
      inicioPulsacionSel = ahora;
      pulsacionLargaProcesada = false;
    } else {                               // se soltó
      unsigned long duracion = ahora - inicioPulsacionSel;
      if (duracion < TIEMPO_PULSACION_LARGA_MS) {
        ejeActivo = (ejeActivo + 1) % 3;    // pulsación corta -> siguiente eje
        Serial.print("Eje activo: ");
        Serial.println(nombresEje[ejeActivo]);
      }
    }
    estadoAnteriorSel = sel;
  }
  // Mientras se mantiene presionado, revisar si ya se cumplió la pulsación larga
  if (sel == LOW && !pulsacionLargaProcesada && (ahora - inicioPulsacionSel) >= TIEMPO_PULSACION_LARGA_MS) {
    gripAbierto = !gripAbierto;
    pulsacionLargaProcesada = true;
    Serial.println(gripAbierto ? "Pinza: ABIERTA" : "Pinza: CERRADA");
  }

  // ---------- Botones MENOS / MAS: mueven el eje activo mientras se mantienen ----------
  if (ahora - ultimoCiclo >= PERIODO_CICLO_MS) {
    float delta = VELOCIDAD * (PERIODO_CICLO_MS / 1000.0);
    bool menos = digitalRead(PIN_BTN_MENOS) == LOW;
    bool mas = digitalRead(PIN_BTN_MAS) == LOW;

    if (menos && !mas) {
      pos[ejeActivo] -= delta;
    } else if (mas && !menos) {
      pos[ejeActivo] += delta;
    }
    pos[ejeActivo] = constrain(pos[ejeActivo], limiteMin(ejeActivo), limiteMax(ejeActivo));

    ultimoCiclo = ahora;
    Serial.printf("POS,%.3f,%.3f,%.3f,%d\n", pos[0], pos[1], pos[2], gripAbierto ? 1 : 0);
  }
}
