// === Configuración PWM usando defines ===
#define PWM_PIN         18      // Pin GPIO para salida PWM
#define PWM_FREQ        500     // Frecuencia del PWM en Hz
#define PWM_RESOLUTION  8       // Resolución del PWM (bits)
#define DUTY_CYCLE      0       // Ciclo de trabajo inicial (0-255)

const float pulsesPerRevolution = 20;
const int encoderPin = 16;
float dutty_cycle_variable = DUTY_CYCLE;

// --- Estructura para guardar tiempo y rpm ---
struct DataPoint {
  unsigned long timestamp;
  float rpm;
};

const int bufferSize = 5;
DataPoint dataBuffer[bufferSize + 1]; // Se permite puntero hasta posición bufferSize
DataPoint* bufferIndex = &dataBuffer[0];

volatile int pulseCount = 0;
unsigned long lastMillis = 0;

// --- Interrupción para encoder ---
void handleEncoder() {
  pulseCount++;
}

void setup() {
  analogWriteFreq(PWM_FREQ);                  // Establecer frecuencia
  analogWriteResolution(PWM_RESOLUTION);      // Establecer resolución
  analogWrite(PWM_PIN, DUTY_CYCLE);           // Salida PWM

  Serial.begin(115200);
  pinMode(encoderPin, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(encoderPin), handleEncoder, RISING);
}

void loop() {
  if (millis() - lastMillis >= 1000) {
    noInterrupts();
    int count = pulseCount;
    pulseCount = 0;
    interrupts();

    lastMillis = millis();

    // Guardar dato en buffer
    bufferIndex->timestamp = millis();
    bufferIndex->rpm = count * 60.0 / pulsesPerRevolution;
    bufferIndex++;

    // Si llegamos al final del buffer
    if (bufferIndex == &dataBuffer[bufferSize]) {
      for (int i = 0; i < bufferSize; i++) {
        Serial.print(dataBuffer[i].timestamp);
        Serial.print(",");
        Serial.println(dataBuffer[i].rpm);
      }

      // Reiniciar puntero y avanzar PWM
      bufferIndex = &dataBuffer[0];

      dutty_cycle_variable += (255 / 10);  // Incremento escalonado
      if (dutty_cycle_variable > 255) while(1);
      analogWrite(PWM_PIN, dutty_cycle_variable);
    }
  }
}
