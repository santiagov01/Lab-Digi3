from machine import Pin, PWM, Timer
import time

# PWM config
pwm_pin = Pin(18) # Asignación Pin de PWM
pwm = PWM(pwm_pin) # Pin de PWM
pwm.freq(500)  # 500 Hz PWM
PWM_MAX = 30801 # Valor máximo del ciclo de trabajo (100%) 47% de 65535
PWM_MIN = 12452 # Valor mínimo del ciclo de trabajo (0%) 19% de 65535
pwm_step = 5 # Paso de PWM en %
adjusted_duty = PWM_MIN + (pwm_step * (PWM_MAX - PWM_MIN) / 100) # Ajuste del rango de PWM
PWM_STEP  = int(adjusted_duty * 65535  / 100) # Conversión a duty_u16
pwm_direction = "up" # Dirección del ciclo de trabajo (ascendente - descendente)

# Encoder config
encoder_pin = Pin(16, Pin.IN, Pin.PULL_UP) # Pin del encoder
pulse_count = 0 # Contador de pulsos del encoder
pulses_per_revolution = 20.0 # Pulsos por revolución del encoder

# RPM buffer
buffer_size = 3000 # Tamaño del buffer de datos
data_buffer = [] # Buffer para almacenar datos de RPM
buffer_index = 0 # Índice del buffer

# Interrupción de conteo
def handle_encoder(pin):
    global pulse_count
    pulse_count += 1

# Toma de muestras cada 4 ms
def sample_rpm(timer):
    global pulse_count, data_buffer, buffer_index, buffer_size

    count = pulse_count
    pulse_count = 0
    timestamp = time.ticks_ms()
    rpm = (count / pulses_per_revolution) * 15000.0  # Conversión a RPM

    # Agregar datos al buffer
    data_buffer.append((timestamp, rpm, pwm.duty_u16()))
    buffer_index += 1

    # Verificar si el buffer está lleno
    if buffer_index >= buffer_size:
        print('Borrando Buffer')
        # Reiniciar el buffer
        data_buffer.clear()
        buffer_index = 0

def PWM_values(timer):
    global pwm, pwm_direction, data_buffer, buffer_index
    # Incrementa o decrementa el ciclo de trabajo según la dirección
    duty_u16 = pwm.duty_u16()
    if pwm_direction == "up":
        duty_u16 += PWM_STEP
        if duty_u16 >= PWM_MAX:
            duty_u16 = PWM_MAX
            pwm_direction = "down"  # Cambia la dirección a descendente
    elif pwm_direction == "down":
        duty_u16 -= PWM_STEP
        if duty_u16 <= PWM_MIN:
            duty_u16 = 0

            # Imprimir todos los datos del buffer
            for i in range(buffer_index):
                print(f"{data_buffer[i][0]},{data_buffer[i][1]},{data_buffer[i][2]}")  # Imprime el tiempo, RPM y PWM
            data_buffer.clear()
            buffer_index = 0
            pwm_direction = "stop" # Cambia la dirección a ascendente
                

    pwm.duty_u16(duty_u16)  # Actualiza el ciclo de trabajo del PWM

print("time(ms),rpm,pwm") # Encabezado de la salida

#interrupcion para el encoder
encoder_pin.irq(trigger=Pin.IRQ_RISING, handler=handle_encoder)

# Timer para muestreo de RPM
timer_muestreo = Timer()
timer_muestreo.init(freq=250, mode=Timer.PERIODIC, callback=sample_rpm)

# Timer para cambio de PWM
timer_PWM = Timer()
timer_PWM.init(freq=0.5, mode=Timer.PERIODIC, callback=PWM_values)