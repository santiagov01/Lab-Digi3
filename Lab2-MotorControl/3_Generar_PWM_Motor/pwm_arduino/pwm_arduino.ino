/**
 * @file motor_min_duty_finder.ino
 * @brief Determina el mínimo porcentaje de duty cycle necesario para que el motor comience a moverse.
 * 
 * Este programa utiliza PWM para aumentar gradualmente el duty cycle del motor cada 500 ms,
 * mide las RPM usando un encoder, y detecta el momento en el que el motor empieza a girar.
 */

// === Configuración PWM usando defines ===
#define PWM_PIN         18      /**< GPIO utilizado para la salida PWM */
#define PWM_FREQ        500     /**< Frecuencia de la señal PWM (en Hz) */
#define PWM_RESOLUTION  8       /**< Resolución del PWM en bits */
#define DUTY_CYCLE_MAX  130     /**< Máximo valor de duty cycle para la resolución actual (0-255) */

int percentage = 0;                          /**< Duty cycle actual en porcentaje (0 a 100) */
const int pulsesPerRevolution = 20;         /**< Pulsos por revolución del encoder */
const int encoderPin = 16;                  /**< Pin GPIO conectado al encoder rotatorio */

volatile int pulseCount = 0;                /**< Contador de pulsos del encoder (modificado por interrupción) */
unsigned long lastMillis = 0;               /**< Última marca de tiempo para control de temporización */

/**
 * @brief Manejador de interrupción para el encoder.
 * 
 * Se llama cada vez que se detecta un flanco ascendente en el pin del encoder.
 */
void handleEncoder() {
  pulseCount++;
}

/**
 * @brief Función de configuración inicial.
 * 
 * Configura la salida PWM, el pin del encoder y la comunicación serial.
 */
void setup() {
  analogWriteFreq(PWM_FREQ);                         // Configurar la frecuencia del PWM
  analogWriteResolution(PWM_RESOLUTION);             // Configurar la resolución del PWM

  analogWrite(PWM_PIN, 0);                           // Inicializar con PWM en 0

  Serial.begin(115200);                              // Iniciar comunicación serial
  pinMode(encoderPin, INPUT_PULLUP);                 // Configurar pin del encoder como entrada con pull-up
  attachInterrupt(digitalPinToInterrupt(encoderPin), handleEncoder, RISING); // Asociar ISR
}

/**
 * @brief Bucle principal del programa.
 * 
 * Cada 500 ms, incrementa el duty cycle en 1%, aplica el nuevo PWM,
 * mide las RPM y verifica si el motor ha comenzado a moverse.
 */
void loop() {
  unsigned long currentMillis = millis();

  // Cada 500 ms
  if (currentMillis - lastMillis >= 500) {
    noInterrupts();
    int count = pulseCount;
    pulseCount = 0;
    interrupts();

    float rpm = (count / (float)pulsesPerRevolution) * 120.0;

    Serial.print("Tiempo (ms): ");
    Serial.print(currentMillis);
    Serial.print(", RPM: ");
    Serial.println(rpm);

    // Verifica si el motor ha comenzado a moverse
    if (rpm && rpm < 2000) {
      Serial.print("El motor empieza a moverse con %DUTY: ");
      Serial.println(percentage);
    }

    lastMillis = currentMillis;

    // Calcular y aplicar nuevo duty cycle
    int duty_mapped = map(percentage, 0, 100, 0, DUTY_CYCLE_MAX);
    analogWrite(PWM_PIN, duty_mapped);

    // Incrementar porcentaje hasta 60%, luego reiniciar
    percentage++;
    if (percentage > 60) percentage = 0;
  }
}
