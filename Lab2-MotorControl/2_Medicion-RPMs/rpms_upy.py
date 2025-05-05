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
pulses_per_revolution = 20.0 

def set_pwm_percent(duty_percent, min_started=0):
    """Configura el PWM con precisión de 1%"""
    if 0 <= duty_percent <= 100:
        adjusted_duty = min_started + (duty_percent * (MAX_STARTED - min_started) / 100)
        duty_u16  = int(adjusted_duty * 65535  / 100)
        pwm.duty_u16(duty_u16)

# Interrupción de conteo
def handle_encoder(pin):
    global pulse_count
    pulse_count += 1

encoder_pin.irq(trigger=Pin.IRQ_RISING, handler=handle_encoder)

# Toma de muestras cada 4 ms
def sample_rpm(timer):
    global pulse_count, data_buffer, buffer_index

    count = pulse_count
    pulse_count = 0
    timestamp = time.ticks_ms()
    rpm = (count / pulses_per_revolution) * 15000  # Conversión a RPM
    print(f"RPMs recientes: {rpm}")
set_pwm_percent(20)
# Timer para muestreo de RPM
timer = Timer()
timer.init(freq=250, mode=Timer.PERIODIC, callback=sample_rpm)