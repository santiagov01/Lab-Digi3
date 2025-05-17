from machine import Pin, PWM, Timer
import time

# PWM config
pwm_pin = Pin(18) # Pin de PWM
pwm = PWM(pwm_pin) # Pin de PWM
pwm.freq(500)  # 500 Hz PWM
duty_percent = 20 # Ancho de pulso de 20%
MAX_STARTED = 47 # Ancho de pulso máximo al iniciar el motor
adjusted_duty = (duty_percent * (MAX_STARTED) / 100)
duty_u16  = int(adjusted_duty * 65535  / 100)
pwm.duty_u16(duty_u16)

# Encoder config
encoder_pin = Pin(16, Pin.IN, Pin.PULL_UP) # Pin del encoder
pulse_count = 0 # Contador de pulsos del encoder
pulses_per_revolution = 20.0 

# Interrupción de conteo
def handle_encoder(pin):
    global pulse_count
    pulse_count += 1

encoder_pin.irq(trigger=Pin.IRQ_RISING, handler=handle_encoder)

# Toma de muestras cada 4 ms
def sample_rpm(timer):
    count = pulse_count
    pulse_count = 0
    timestamp = time.ticks_ms()
    rpm = (count / pulses_per_revolution) * 15000.0  # Conversión a RPM
    print(f"RPMs recientes: {rpm}")

# Timer para muestreo de RPM
timer = Timer()
timer.init(freq=250, mode=Timer.PERIODIC, callback=sample_rpm)
