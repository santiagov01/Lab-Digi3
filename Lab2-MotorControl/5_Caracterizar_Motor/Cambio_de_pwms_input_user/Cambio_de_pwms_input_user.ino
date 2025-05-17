/**
 * @file rpm_pwm_capture.ino
 * @brief Controla un motor mediante PWM y mide las RPM usando un encoder.
 * 
 * Este programa está diseñado para la Raspberry Pi Pico con Arduino. Permite
 * el control del motor en dos modos: manual y captura escalonada.
 * En el modo de captura, se genera una secuencia de PWM ascendente y descendente, y
 * se registran los datos (timestamp, RPM, PWM) en un buffer. En modo manual,
 * el usuario puede fijar un valor de PWM específico y monitorear en tiempo real.
 * 
 * Comandos soportados por el monitor serial:
 * - `START <valor>`: Inicia la captura con un incremento/decremento de PWM de <valor>% por paso.
 * - `PWM <valor>`: Fija un valor de PWM en modo manual (0–100).
 */
// === Configuración PWM y Encoder ===
#define PWM_PIN         18     ///< Pin de salida PWM al motor.
#define PWM_FREQ        500    ///< Frecuencia del PWM.
#define PWM_RESOLUTION  8      ///< Resolución del PWM en bits.
#define ENCODER_PIN     16     ///< Pin de entrada del encoder.
#define PULSOS_POR_REV  20     ///< Pulsos por revolución del encoder.

#define MIN_PWM         0      ///< Valor mínimo de PWM.
#define MAX_PWM         130    ///< Valor máximo de PWM.

// === Estructura para guardar datos ===

/**
 * @struct DataPoint
 * @brief Almacena un punto de datos con tiempo, RPM y PWM.
 */
struct DataPoint {
  unsigned long timestamp; ///< Marca de tiempo en milisegundos.
  float rpm;               ///< Revoluciones por minuto.
  int pwm;                 ///< Valor de PWM aplicado.
};

const int bufferSize = 5000;          ///< Tamaño del búfer de datos.
DataPoint dataBuffer[bufferSize];     ///< Arreglo para almacenar datos.
DataPoint* bufferIndex = dataBuffer;  ///< Índice actual del búfer.

volatile int pulseCount = 0;          ///< Conteo de pulsos del encoder (variable compartida con ISR).
unsigned long lastSampleMillis = 0;   ///< Último tiempo de muestreo.
unsigned long lastPrintMillis = 0;    ///< Último tiempo de impresión por serial.
unsigned long lastStepMillis = 0;     ///< Último tiempo de paso PWM.

bool capturando = false;      ///< Indica si se está capturando datos.
bool modo_manual = true;      ///< Indica si está en modo manual.
int pwm_manual = 0;           ///< Valor de PWM fijado manualmente.
int pwm_actual = 0;           ///< PWM actual aplicado.
int delta_pwm = 0;            ///< Paso entre niveles de PWM.
int pasoPWM = 0;              ///< Valor de paso PWM (map del delta).
int numPasos = 0;             ///< Número de pasos PWM calculado.

float last_rpm = 0;           ///< Último valor de RPM leído.

bool subiendo = true;         ///< Dirección actual del escalón PWM.

/**
 * @brief Rutina de interrupción para contar los pulsos del encoder.
 */
void handleEncoder() {
  pulseCount++;
}

/**
 * @brief Configura pines, PWM, interrupciones y comunicación serial.
 */
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

/**
 * @brief Bucle principal del programa.
 * Realiza lectura de comandos, medición de RPM y control del motor.
 */
void loop() {
  leerComandosSerial();

  unsigned long ahora = millis();

  // Medición de RPM cada 4 ms
  if (ahora - lastSampleMillis >= 4) {
    lastSampleMillis = millis();
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
    if (modo_manual && (millis() - lastPrintMillis) >= 500) {
      lastPrintMillis = millis();
      Serial.print("PWM: ");
      Serial.print(pwm_actual);
      Serial.print(" | RPM: ");
      Serial.println(avg_rpm);
    }
    
  }

  // Manejo de escalones en modo captura
  if (capturando && (millis() - lastStepMillis) >= 2000) {
    lastStepMillis = millis();

    if(subiendo){
        pwm_actual += pasoPWM;
        
        if (pwm_actual >= MAX_PWM) {
          pwm_actual = MAX_PWM;
          subiendo = false;
          //analogWrite(PWM_PIN, pwm_actual);
        }
      analogWrite(PWM_PIN, pwm_actual);
    } else{
        pwm_actual -= pasoPWM;
        if (pwm_actual <= 0){
          pwm_actual = 0;
          analogWrite(PWM_PIN, pwm_actual);
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
      analogWrite(PWM_PIN, pwm_actual);
      }
    }
  }


/**
 * @brief Procesa los comandos recibidos por el puerto serial.
 * 
 * Comandos disponibles:
 * - `START <valor>`: Inicia captura con pasos PWM de tamaño <valor>.
 * - `PWM <valor>`: Aplica PWM manual.
 */
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
        Serial.println("ERROR: Valor de escalones PWM inválido (1-99)");
        return;
      }

      // Calcular número de pasos y reiniciar buffer
      numPasos = 100 / delta_pwm;
      pasoPWM = map(delta_pwm, 0, 100, 0, MAX_PWM);
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
      int pwm_map = map(pwm_actual, 0, 100, 0, MAX_PWM);
      analogWrite(PWM_PIN, pwm_map);
      modo_manual = true;
      Serial.print("PWM manual fijado a ");
      Serial.println(pwm_actual);
    }
  }
}
