/**
 * @file pwm_encoder_logger.ino
 * @brief Control de PWM y registro de RPM usando un encoder con Raspberry Pi Pico en Arduino IDE.
 * 
 * Este programa configura una salida PWM y mide las RPM mediante un encoder conectado a un pin GPIO.
 * Registra los datos de RPM cada segundo y los transmite por el puerto serial cada 5 segundos.
 */

// === Configuración PWM usando defines ===
#define PWM_PIN         18      /**< GPIO para la salida PWM */
#define PWM_FREQ        500     /**< Frecuencia de la señal PWM (Hz) */
#define PWM_RESOLUTION  8       /**< Resolución del PWM en bits */
#define DUTY_CYCLE_MAX  130     /**< Ciclo de trabajo máximo para PWM (0-255 para 8 bits) */

const int percentage = 30;                    /**< Porcentaje deseado del ciclo de trabajo */
const int pulsesPerRevolution = 20;          /**< Pulsos por revolución del encoder */
const int encoderPin = 16;                   /**< GPIO conectado al encoder */

/**
 * @struct DataPoint
 * @brief Estructura para almacenar mediciones de tiempo y RPM.
 */
struct DataPoint {
  unsigned long timestamp;  /**< Tiempo en milisegundos */
  float rpm;                /**< Revoluciones por minuto */
};

const int bufferSize = 5;                   /**< Número de datos a almacenar antes de enviar */
DataPoint dataBuffer[bufferSize];           /**< Buffer circular para almacenamiento de datos */
int bufferIndex = 0;                        /**< Índice actual en el buffer */

volatile int pulseCount = 0;                /**< Contador de pulsos del encoder (actualizado en ISR) */
unsigned long lastMillis = 0;               /**< Marca de tiempo del último registro */

/**
 * @brief Manejador de interrupciones para el encoder.
 * 
 * Incrementa el contador de pulsos cada vez que se detecta un flanco ascendente.
 */
void handleEncoder() {
  pulseCount++;
}

/**
 * @brief Función de configuración.
 * 
 * Inicializa la salida PWM, el encoder y la comunicación serial.
 */
void setup() {
  analogWriteFreq(PWM_FREQ);                         // Establecer frecuencia del PWM
  analogWriteResolution(PWM_RESOLUTION);             // Establecer resolución del PWM
  int duty_mapped = map(percentage, 0, 100, 0, DUTY_CYCLE_MAX);
  analogWrite(PWM_PIN, duty_mapped);                 // Enviar señal PWM con ciclo de trabajo mapeado

  Serial.begin(115200);                              // Inicializar comunicación serial
  pinMode(encoderPin, INPUT_PULLUP);                 // Configurar pin del encoder con pull-up
  attachInterrupt(digitalPinToInterrupt(encoderPin), handleEncoder, RISING); // Asignar ISR
}

/**
 * @brief Bucle principal del programa.
 * 
 * Mide la cantidad de pulsos del encoder cada segundo, calcula las RPM,
 * almacena los valores, y cada 5 segundos los transmite por serial.
 */
void loop() {
  unsigned long currentMillis = millis();

  // Verifica si ha pasado 1 segundo
  if (currentMillis - lastMillis >= 1000) {
    noInterrupts();                  // Proteger sección crítica
    int count = pulseCount;
    pulseCount = 0;
    interrupts();

    float rpm = (count / (float)pulsesPerRevolution) * 60.0;

    // Guardar en buffer
    if (bufferIndex < bufferSize) {
      dataBuffer[bufferIndex++] = {currentMillis, rpm};
    }

    lastMillis = currentMillis;
  }

  // Si se ha llenado el buffer, enviar por serial
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
