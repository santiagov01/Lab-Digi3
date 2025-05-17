from machine import Pin, PWM, Timer, UART
import time

# Configuración de UART
uart = UART(0, baudrate=115200)  # UART0 con baudrate de 115200

# Configuración de PWM
pwm_pin = Pin(18)
pwm = PWM(pwm_pin)
pwm.freq(500)  # Frecuencia de 500 Hz
PWM_MAX = 65535  # Valor máximo del ciclo de trabajo
PWM_MIN = 0      # Valor mínimo del ciclo de trabajo
pwm_direction = "up" # Dirección del ciclo de trabajo (ascendente - descendente)

# Configuración del encoder
encoder_pin = Pin(16, Pin.IN, Pin.PULL_UP)
pulse_count = 0
pulses_per_revolution = 20.0

# Variables globales
capturing = False
data_buffer = []
buffer_index = 0
pwm_step = 0
current_pwm = 0

# Interrupción para el encoder
def handle_encoder(pin):
    global pulse_count
    pulse_count += 1

def mapear(duty_percent):
    global min_started
    """Mapeamos el pwm"""
    adjusted_duty = PWM_MIN + (duty_percent * (PWM_MAX - PWM_MIN) / 100)
    duty_u16  = int(adjusted_duty * 65535  / 100)
    return duty_u16


# Toma de muestras cada 4 ms
def sample_rpm(timer):
    global pulse_count, data_buffer, buffer_index, buffer_size

    count = pulse_count
    pulse_count = 0
    timestamp = time.ticks_ms()
    rpm = (count / pulses_per_revolution) * 15000.0  # Conversión a RPM

    # Agregar datos al buffer
    data_buffer[buffer_index] = [timestamp, rpm, pwm.duty_u16()]
    buffer_index += 1
      

def PWM_values(timer):
    global pwm, pwm_direction, data_buffer, buffer_index, pwm_step
    # Incrementa o decrementa el ciclo de trabajo según la dirección
    duty_u16 = pwm.duty_u16()
    if pwm_direction == "up":
        duty_u16 += pwm_step
        if duty_u16 >= PWM_MAX:
            duty_u16 = PWM_MAX
            pwm_direction = "down"  # Cambia la dirección a descendente
    elif pwm_direction == "down":
        duty_u16 -= pwm_step
        if duty_u16 <= PWM_MIN:
            duty_u16 = PWM_MIN
            pwm_direction = "up"  # Cambia la dirección a ascendente
            # Imprimir todos los datos del buffer
            pwm.duty_u16(duty_u16)
            for data in data_buffer:
                print(f"{data[0]},{data[1]},{data[2]}")  # Imprime el tiempo, RPM y PWM
                time.sleep(0.5)  # Espera 0.5 s entre cada impresión para evitar saturar el UART
            # Reiniciar el buffer
            data_buffer.clear()
            buffer_index = 0 

    pwm.duty_u16(duty_u16)  # Actualiza el ciclo de trabajo del PWM

timer_capture = Timer()
timer_pwm = Timer()
# Función para procesar comandos seriales
def process_serial():
    global capturing, pwm_step, current_pwm

    if uart.any():
        command = uart.readline().decode('utf-8').strip()
        if command.startswith("START"):
            try:
                _, step = command.split()
                pwm_step = int(step)
                if pwm_step <= 0 or pwm_step > 100:
                    uart.write("ERROR: El valor de PWM_STEP debe estar entre 1 y 100.\n")
                    return
                
                pwm_step = mapear(pwm_step)  # Configura el PWM con el valor de paso
                capturing = True
                current_pwm = 0
                pwm.duty_u16(0)
                data_buffer.clear()
                uart.write("Captura iniciada.\n")
                timer_capture.init(freq=250, mode=Timer.PERIODIC, callback=sample_rpm)
                timer_pwm.init(freq=0.5, mode=Timer.PERIODIC, callback=PWM_values)  # Detener el temporizador de PWM
            except ValueError:
                uart.write("ERROR: Comando START inválido. Uso: START <valor>\n")
        elif command.startswith("PWM"):
            try:
                _, value = command.split()
                pwm_value = int(value)
                if pwm_value < 0 or pwm_value > 100:
                    uart.write("ERROR: El valor de PWM debe estar entre 0 y 100.\n")
                    return
                if not capturing:
                    current_pwm = pwm_value
                    current_pwm = mapear(current_pwm)  # Configura el PWM con el valor de duty cycle
                    pwm.duty_u16(current_pwm)
                    uart.write(f"PWM ajustado a {current_pwm}%.\n")
            except ValueError:
                uart.write("ERROR: Comando PWM inválido. Uso: PWM <valor>\n")
        else:
            uart.write("ERROR: Comando no reconocido.\n")

encoder_pin.irq(trigger=Pin.IRQ_RISING, handler=handle_encoder)

# Bucle principal
while True:
    process_serial()
    if not capturing:
        # Enviar datos 
        count = pulse_count
        pulse_count = 0
        for data in data_buffer:
            uart.write(f"{data[0]},{data[1]},{data[2]}\n")