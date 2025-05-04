// === Configuración PWM y Encoder ===
#define PWM_PIN         18
#define PWM_FREQ        500
#define PWM_RESOLUTION  8
#define ENCODER_PIN     16
#define PULSOS_POR_REV  20

// === Estructura para guardar datos ===
struct DataPoint {
  unsigned long timestamp;
  float rpm;
  int pwm;
};

const int bufferSize = 5000;
DataPoint dataBuffer[bufferSize];
DataPoint* bufferIndex = dataBuffer;

volatile int pulseCount = 0;
unsigned long lastSampleMillis = 0;
unsigned long lastPrintMillis = 0;
unsigned long lastStepMillis = 0;


bool capturando = false;
bool modo_manual = true;
int pwm_manual = 0;
int pwm_actual = 0;
int delta_pwm = 0;
int pasoPWM = 0; // índice del paso actual
int numPasos = 0;

float last_rpm = 0;


// --- ISR del encoder ---
void handleEncoder() {
  pulseCount++;
}

// --- Inicialización ---
void setup() {
  Serial.begin(115200);
  analogWriteFreq(PWM_FREQ);
  analogWriteResolution(PWM_RESOLUTION);

  pinMode(ENCODER_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENCODER_PIN), handleEncoder, RISING);

  analogWrite(PWM_PIN, 0);
  lastSampleMillis = millis();
  lastPrintMillis = millis();
  lastStepMillis = millis();

  Serial.println("Sistema listo. Use comandos: START <valor> o PWM <valor>");
}

// --- LOOP PRINCIPAL ---
void loop() {
  leerComandosSerial();

  unsigned long ahora = millis();

  // Medición de RPM cada 4 ms
  if (ahora - lastSampleMillis >= 4) {
    lastSampleMillis = ahora;
    noInterrupts();
    int count = pulseCount;
    pulseCount = 0;
    interrupts();
    float rpm = count * (15000.0 / PULSOS_POR_REV);
    float avg_rpm = abs((rpm + last_rpm)/2);
    last_rpm = rpm;
    if (capturando && bufferIndex < dataBuffer + bufferSize) {
      bufferIndex->timestamp = ahora;
      bufferIndex->rpm = avg_rpm;
      bufferIndex->pwm = pwm_actual;
      bufferIndex++;
    }

    // En modo manual, imprimir cada 500 ms
    if (modo_manual && ahora - lastPrintMillis >= 500) {
      lastPrintMillis = ahora;
      Serial.print("PWM: ");
      Serial.print(pwm_actual);
      Serial.print(" | RPM: ");
      Serial.println(avg_rpm);
    }
  }

  // Manejo de escalones en modo captura
  if (capturando && ahora - lastStepMillis >= 2000) {
    lastStepMillis = ahora;
    pasoPWM++;

    if (pasoPWM < numPasos) {
      pwm_actual = pasoPWM * delta_pwm;
      analogWrite(PWM_PIN, pwm_actual);
    } else if (pasoPWM < 2 * numPasos) {
      // fase descendente
      pwm_actual = (2 * numPasos - pasoPWM - 1) * delta_pwm;
      analogWrite(PWM_PIN, pwm_actual);
    } else {
      // fin de captura
      analogWrite(PWM_PIN, 0);
      capturando = false;
      modo_manual = true;

      Serial.println("---- FIN DE CAPTURA ----");
      for (int i = 0; i < bufferIndex - dataBuffer; i++) {
        Serial.print(dataBuffer[i].timestamp);
        Serial.print(",");
        Serial.print(dataBuffer[i].pwm);
        Serial.print(",");
        Serial.println(dataBuffer[i].rpm);
      }
    }
  }
}

// --- Función para leer comandos seriales ---
void leerComandosSerial() {
  if (Serial.available()) {
    String input = Serial.readStringUntil('\n');
    input.trim();

    if (input.startsWith("START")) {
      if (capturando) return;

      int espacio = input.indexOf(' ');
      if (espacio == -1) return;

      delta_pwm = input.substring(espacio + 1).toInt();
      if (delta_pwm <= 0 || delta_pwm >= 100) {
        Serial.println("ERROR: Valor de PWM inválido (1-99)");
        return;
      }

      // Calcular número de pasos y reiniciar buffer
      numPasos = 100 / delta_pwm;
      pasoPWM = 0;
      bufferIndex = dataBuffer;
      pwm_actual = 0;
      analogWrite(PWM_PIN, pwm_actual);
      lastStepMillis = millis();
      capturando = true;
      modo_manual = false;
      Serial.println(">> INICIANDO CAPTURA...");

    } else if (input.startsWith("PWM")) {
      if (capturando) return;

      int espacio = input.indexOf(' ');
      if (espacio == -1) return;

      pwm_manual = input.substring(espacio + 1).toInt();
      if (pwm_manual < 0 || pwm_manual > 100) {
        Serial.println("ERROR: PWM debe estar entre 0 y 100");
        return;
      }

      pwm_actual = pwm_manual;
      analogWrite(PWM_PIN, pwm_actual);
      modo_manual = true;
      Serial.print("PWM manual fijado a ");
      Serial.println(pwm_actual);
    }
  }
}
