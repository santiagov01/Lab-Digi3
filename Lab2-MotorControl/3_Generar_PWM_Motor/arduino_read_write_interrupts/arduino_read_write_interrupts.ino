// === Configuración PWM usando defines ===
#define PWM_PIN         15      // Pin GPIO para salida PWM
#define PWM_FREQ        500    // Frecuencia del PWM en Hz
#define PWM_RESOLUTION  8       // Resolución del PWM (bits)
#define DUTY_CYCLE      255     // Ciclo de trabajo (0-255 para 8 bits). Cambiar a variable en punto 3 y 4

const int pulsesPerRevolution = 20;
const int encoderPin = 16;

struct DataPoint {
  unsigned long timestamp;
  int rpm;
};

const int bufferSize = 5;  // Guardamos datos por 5 segundos
DataPoint dataBuffer[bufferSize];
int bufferIndex = 0;

volatile int pulseCount = 0;
unsigned long lastMillis = 0;

void handleEncoder() {
  pulseCount++;
}

void setup() {
  analogWriteFreq(PWM_FREQ);                  // Establecer frecuencia
  analogWriteResolution(PWM_RESOLUTION);      // Establecer resolución
  analogWrite(PWM_PIN, DUTY_CYCLE);           // Salida PWM con ciclo de trabajo

  Serial.begin(115200);
  pinMode(encoderPin, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(encoderPin), handleEncoder, RISING);
}

void loop() {
  unsigned long currentMillis = millis();

  // Cada 1 segundo
  if (currentMillis - lastMillis >= 1000) {
    noInterrupts();
    int count = pulseCount;
    pulseCount = 0;
    interrupts();

    float rpm = (count / (float)pulsesPerRevolution) * 60.0;

    // Guardar en buffer
    if (bufferIndex < bufferSize) {
      dataBuffer[bufferIndex++] = {currentMillis, count}; //cambiar count a rpm, ojo
    }

    lastMillis = currentMillis;
  }

  // Después de 5 datos (5 segundos), enviamos por serial
  if (bufferIndex >= bufferSize) {
    Serial.println("----- Datos guardados -----");
    for (int i = 0; i < bufferSize; i++) {
      Serial.print("Tiempo (ms): ");
      Serial.print(dataBuffer[i].timestamp);
      Serial.print(" | RPM: ");
      Serial.println(dataBuffer[i].rpm); 
    }
    Serial.println("---------------------------");

    bufferIndex = 0; // Reiniciar buffer
  }
}
