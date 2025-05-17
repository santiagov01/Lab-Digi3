from machine import Pin, PWM, Timer
import time

# PWM config
pwm_pin = Pin(18) # Pin de PWM
pwm = PWM(pwm_pin) # Pin de PWM
pwm.freq(500)  # 500 Hz PWM
ANCHO_DE_PULSO = 50 # Ancho de pulso de 50%
MAX_STARTED = 47 # Ancho de pulso máximo al iniciar el motor

# Encoder config
encoder_pin = Pin(16, Pin.IN, Pin.PULL_UP) # Pin del encoder
pulse_count = 0 # Contador de pulsos del encoder
pulses_per_revolution = 20.0 # Pulsos por revolución del encoder

# PWM buffer
buffer_size = 100 # Tamaño del buffer de RPM 
data_buffer = [] # Buffer para almacenar datos de RPM
buffer_index = 0 # Índice del buffer

def set_pwm_percent(duty_percent):
    global min_started
    """Configura el PWM con precisión de 1%"""
    if 0 <= duty_percent <= 100:
        adjusted_duty = min_started + (duty_percent * (MAX_STARTED - min_started) / 100)
        duty_u16  = int(adjusted_duty * 65535  / 100)
        pwm.duty_u16(duty_u16)

# Toma de muestras cada 4 ms
def sample_rpm():
    global pulse_count
    count = pulse_count
    pulse_count = 0
    rpm = (count / pulses_per_revolution) * 15000.0  # Conversión a RPM
    return rpm

# Interrupción de conteo
def handle_encoder(pin):
    global pulse_count
    pulse_count += 1

min_started = 0 # Inicializa el valor mínimo de PWM

encoder_pin.irq(trigger=Pin.IRQ_RISING, handler=handle_encoder) # Configura la interrupción para el encoder

for duty in range(0, 101):  # 0% to 100%
    # Timer para muestreo de RPM
    time.sleep(0.5)  # Espera 0.5 segundos
    set_pwm_percent(duty)
    print(f"Probando PWM: {duty}%")

    # Verifica si RPM ha sido distinta de cero
    if sample_rpm(): # RPM > 0
        min_started = duty
        print(f"El motor empezó a girar con PWM = {min_started}%")
        break

set_pwm_percent(ANCHO_DE_PULSO)

