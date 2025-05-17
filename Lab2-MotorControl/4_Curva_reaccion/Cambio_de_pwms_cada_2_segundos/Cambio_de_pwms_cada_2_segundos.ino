/**
 * @file pwm_encoder_logger.ino
 * @brief Registro de RPM con encoder y control PWM escalonado en Raspberry Pi Pico.
 *
 * Este programa genera una señal PWM escalonada y mide la velocidad de rotación (RPM)
 * utilizando un encoder. Los datos se almacenan en un buffer y luego se transmiten por
 * puerto serial una vez finalizado el ciclo de PWM.
 *
 */

// === Configuración PWM y Encoder ===

/** @brief Pin GPIO utilizado para la señal PWM de salida. */
#define PWM_PIN         18

/** @brief Frecuencia de la señal PWM en Hertz. */
#define PWM_FREQ        500

/** @brief Resolución del PWM en bits. (8 bits = 0-255) */
#define PWM_RESOLUTION  8

/** @brief Pin GPIO conectado al encoder (interrupción por flanco de subida). */
#define ENCODER_PIN     16

/** @brief Pulsos por revolución del encoder. */
const int pulsesPerRevolution = 20;

/**
 * @struct DataPoint
 * @brief Estructura para guardar el tiempo y la velocidad (RPM).
 */
struct DataPoint {
  unsigned long timestamp; ///< Tiempo en milisegundos.
  float rpm;               ///< Velocidad en revoluciones por minuto.
};

// === Buffer de datos ===

/** @brief Tamaño del buffer para almacenar datos de RPM. */
const int bufferSize = 6000;

/** @brief Última posición válida del buffer. */
const int last_pos_buffer = bufferSize - 1;

/** @brief Buffer circular donde se almacenan los datos de tiempo y RPM. */
DataPoint dataBuffer[bufferSize];

/** @brief Puntero al índice actual de escritura en el buffer. */
DataPoint* bufferIndex = &dataBuffer[0];

/** @brief Contador de pulsos del encoder (modificado por interrupción). */
volatile int pulseCount = 0;

/** @brief Marca de tiempo de la última muestra de RPM. */
unsigned long lastSampleMillis = 0;

/** @brief Marca de tiempo para el próximo cambio de escalón PWM. */
unsigned long lastSampleMillis_to_2_Seg = 0;

/**
 * @brief Escalones de PWM (20% por paso).
 * Contiene valores ascendentes y luego descendentes.
 */
const int escalonesPWM[10] = {24,48,72,96,120,96,72,48,24,0};

/** @brief Puntero al escalón actual del arreglo de PWM. */
const int* indexEscalones = &escalonesPWM[0];

/** @brief Tiempo en milisegundos que se mantiene cada escalón. */
const unsigned int tiempoPorEscalon = 2000;

/** @brief Índice del paso actual en el patrón de PWM. */
int pasoActual = 0;

/** @brief Último valor calculado de RPM. */
float last_rpm = 0;

/** @brief Contador de datos almacenados en el buffer. */
int contador = 0;

/**
 * @brief Manejador de interrupción del encoder.
 *
 * Incrementa el contador de pulsos cuando se detecta un flanco de subida.
 */
void handleEncoder() {
  pulseCount++;
}

/**
 * @brief Función de configuración inicial de Arduino.
 *
 * Inicializa la comunicación serial, el PWM, el pin del encoder y la interrupción.
 */
void setup() {
  Serial.begin(115200);
  analogWriteFreq(PWM_FREQ);
  analogWriteResolution(PWM_RESOLUTION);

  pinMode(ENCODER_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENCODER_PIN), handleEncoder, RISING);

  delay(6000); // Espera para que el motor esté en reposo
  analogWrite(PWM_PIN, *indexEscalones); // Primer escalón de PWM
  Serial.println("Tiempo(ms),RPM");

  lastSampleMillis = millis();
  lastSampleMillis_to_2_Seg = millis();
}

/**
 * @brief Bucle principal.
 *
 * Captura la RPM cada 4 ms, cambia el PWM cada 2 segundos,
 * y guarda los datos en el buffer. Al final transmite los datos por serial.
 */
void loop() {
  if (millis() - lastSampleMillis >= 4) {
    noInterrupts();
    int count = pulseCount;
    pulseCount = 0;
    interrupts();

    lastSampleMillis = millis();

    bufferIndex->timestamp = millis();
    bufferIndex->rpm = count * (15000.0 / pulsesPerRevolution);
    bufferIndex++;
    contador++;

    if (millis() - lastSampleMillis_to_2_Seg >= 2000) {
      lastSampleMillis_to_2_Seg = millis();
      indexEscalones++;
      analogWrite(PWM_PIN, *indexEscalones);

      if (!(*indexEscalones)) {
        int i = 0;
        while(i < contador) {
          Serial.print(dataBuffer[i].timestamp);
          Serial.print(",");
          Serial.println(dataBuffer[i].rpm);
          i++;
        }
        while (true); // Fin del programa
      }
    }
  }
}
