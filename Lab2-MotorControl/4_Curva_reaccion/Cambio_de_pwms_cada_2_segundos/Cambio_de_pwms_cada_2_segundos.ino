// === Configuración PWM y Encoder ===
#define PWM_PIN         18      // Pin GPIO para salida PWM
#define PWM_FREQ        500     // Frecuencia PWM (Hz)
#define PWM_RESOLUTION  8       // Bits resolución PWM (0-255)
#define ENCODER_PIN     16      // Pin del encoder
const int pulsesPerRevolution = 20;
// --- Estructura para guardar tiempo y rpm ---
struct DataPoint {
  unsigned long timestamp;
  float rpm;
};

const int bufferSize = 6000;
const int last_pos_buffer = bufferSize-1;
DataPoint dataBuffer[bufferSize]; // Se permite puntero hasta posición bufferSize
DataPoint* bufferIndex = &dataBuffer[0];

volatile int pulseCount = 0;
unsigned long lastSampleMillis = 0;
unsigned long lastSampleMillis_to_2_Seg = 0;

const int escalonesPWM[10] = {24,48,72,96,120,96,72,48,24,0}; // 20% por paso, ascendente y descendente
const int * indexEscalones = &escalonesPWM[0];

const unsigned int tiempoPorEscalon = 2000;  // 2 segundos por escalón
int pasoActual = 0;

float last_rpm = 0;
int contador = 0;

void handleEncoder() {
  pulseCount++;
}

void setup() {
  Serial.begin(115200);
  analogWriteFreq(PWM_FREQ);
  analogWriteResolution(PWM_RESOLUTION);

  pinMode(ENCODER_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENCODER_PIN), handleEncoder, RISING);

  delay(6000); // Tiempo para que el motor esté en reposo
  analogWrite(PWM_PIN, *indexEscalones);//*indexEscalones = Escalon Actual
  Serial.println("Tiempo(ms),RPM");
  lastSampleMillis = millis();
  lastSampleMillis_to_2_Seg = millis();
}

void loop() {

  // Capturar RPM cada 4 ms
  if (millis() - lastSampleMillis >= 4) { //Tiempo actual- el anterior tomado
    noInterrupts();
    int count = pulseCount;
    pulseCount = 0;
    interrupts();

    lastSampleMillis = millis();

    // Guardar dato en buffer
    bufferIndex->timestamp = millis();
    // (pulsos en el intervalo/pulsos por revolucion)*(60segundos/tiempo en segundos)15000 = 60/0.004
    
    // float rpm = count * (15000.0 / pulsesPerRevolution);
    // float avg_rpm = (last_rpm + rpm)/2 // para tener un valor más real
    // float last_rpm = rpm; // actualiza rpms anteriores

    bufferIndex->rpm = count * (15000.0 / pulsesPerRevolution);
    bufferIndex++;
    contador++; //revisa la cantidad de escrituras para evitar transmitir los ceros al final de la transmisión
    
    // Cambiar PWM cada 2 segundos
    if (millis() - lastSampleMillis_to_2_Seg >= 2000) {
      lastSampleMillis_to_2_Seg = millis();
      indexEscalones++; //a la siguiente posición
      analogWrite(PWM_PIN, *indexEscalones);//*indexEscalones = Escalon Actual
      if (!(*indexEscalones)){ // *indexEscalones = 0?
        int i = 0;
        while(i < contador) {
          Serial.print(dataBuffer[i].timestamp);
          Serial.print(",");
          Serial.println(dataBuffer[i].rpm);
          i++;
        }
      while (true); // Finaliza programa
      } 
    }
  }
  
} 
