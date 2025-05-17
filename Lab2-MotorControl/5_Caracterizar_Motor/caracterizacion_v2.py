## @file motor_control.py
#  @brief Control y monitoreo de RPM usando PWM y sensor Hall en MicroPython (Raspberry Pi Pico).
#
#  Este script permite operar un motor en modo automático o manual.
#  En el modo automático, realiza un barrido de duty cycle y mide las RPM.
#  En el modo manual, fija un PWM y mide continuamente las RPM.
#  Si las RPM superan un umbral, se activa una señal de alerta.
#
#  @author TuNombre
#  @date 2025-05-07

from machine import Pin, PWM
import utime
import sys
import select

# ----------------------------
# Configuración general
# ----------------------------

## Frecuencia del PWM en Hz
PWM_FREQ_HZ = 8000

## Pulsos por revolución del encoder
PULSES_PER_REV = 20

## Tiempo de estabilización antes de medir (ms)
T_ESTABILIZACION_MS = 3000

## Tiempo de medición (ms)
T_MEDICION_MS = 1000

## Umbral de RPM para activar la alerta
RPM_UMBRAL = 5000

## Pin de salida PWM al motor
pin_pwm = PWM(Pin(18))
pin_pwm.freq(PWM_FREQ_HZ)

## Pin del sensor de efecto Hall (entrada digital)
sensor_pin = Pin(16, Pin.IN, Pin.PULL_UP)

## LED indicador de estado (modo medición)
led_pin = Pin(3, Pin.OUT)

## Pin de salida para señal de alerta
alerta_pin = Pin(2, Pin.OUT)

## Contador de pulsos del sensor Hall
pulse_count = 0

# ----------------------------
# Definición de estados
# ----------------------------

## Estado de espera sin actividad
ESTADO_ESPERA = 0

## Estado de estabilización del motor
ESTADO_ESTABILIZANDO = 1

## Estado de medición de RPM (automático)
ESTADO_MIDIENDO = 2

## Estado de control manual con medición de RPM
ESTADO_MANUAL = 3

## Estado actual del sistema
estado = ESTADO_ESPERA

## Duty cycle actual en porcentaje
duty_pct = 0

## Paso de incremento/decremento para ciclo automático
step_value = 0

## Buffer para almacenar datos (timestamp, PWM, RPM)
buffer = []

## Marca de tiempo para control de tiempos
t0 = utime.ticks_ms()

## Indica si el ciclo automático está activo
ciclo_automatico = False

## Dirección de cambio del duty (True: subiendo)
subiendo = True

## Indica si ya se superó el umbral de RPM
rpm_excedido = False

# ----------------------------
# Funciones auxiliares
# ----------------------------

## @brief Convierte un porcentaje de duty cycle a valor de 16 bits.
#  @param pct Porcentaje de duty cycle (0–100).
#  @return Valor entero para función `duty_u16()`.
def percent_to_duty(pct):
    return int((pct / 100) * 65535)

## @brief Manejador de interrupción por flanco de bajada del sensor.
#  Incrementa el contador de pulsos por cada interrupción.
#  @param pin Pin que generó la interrupción.
def rpm_callback(pin):
    global pulse_count
    pulse_count += 1

sensor_pin.irq(trigger=Pin.IRQ_FALLING, handler=rpm_callback)

## @brief Lee un comando del puerto serial si está disponible.
#  @return Cadena con el comando o `None` si no hay datos.
def leer_comando_serial():
    if sys.stdin in select.select([sys.stdin], [], [], 0)[0]:
        return sys.stdin.readline().strip()
    return None

print("Esperando comando serial (START <valor> o PWM <valor>)...")

# ----------------------------
# Bucle principal
# ----------------------------
while True:
    # Lectura no bloqueante de comandos
    comando = leer_comando_serial()
    if comando:
        if comando.upper().startswith("START "):
            try:
                step_value = int(comando.split()[1])
                if step_value <= 0 or step_value > 100:
                    raise ValueError
                ciclo_automatico = True
                duty_pct = 0
                pin_pwm.duty_u16(percent_to_duty(duty_pct))
                t0 = utime.ticks_ms()
                estado = ESTADO_ESTABILIZANDO
                buffer = []
                subiendo = True
                rpm_excedido = False
                print(">> Ciclo automático iniciado con paso =", step_value)
            except:
                print(">> Error: comando START debe ser seguido por un valor entre 1 y 100")

        elif comando.upper().startswith("PWM "):
            try:
                valor = int(comando[4:])
                if 0 <= valor <= 100:
                    duty_pct = valor
                    pin_pwm.duty_u16(percent_to_duty(duty_pct))
                    estado = ESTADO_ESTABILIZANDO
                    t0 = utime.ticks_ms()
                    print(">> PWM fijado manualmente en", valor, "%")
                else:
                    print(">> Error: Valor fuera de rango (0–100)")
            except:
                print(">> Error de formato. Usa: PWM xx")

    # Modo manual con medición continua de RPM
    if estado == ESTADO_MANUAL:
        led_pin.value(1)
        if utime.ticks_diff(utime.ticks_ms(), t0) > T_MEDICION_MS:
            rpm = (pulse_count * 60) // PULSES_PER_REV
            print("PWM:", duty_pct, "% RPM:", rpm)
            alerta_pin.value(1 if rpm > RPM_UMBRAL else 0)
            pulse_count = 0
            t0 = utime.ticks_ms()

    # Modo de estabilización (espera antes de medir)
    elif estado == ESTADO_ESTABILIZANDO:
        led_pin.value(0)
        if utime.ticks_diff(utime.ticks_ms(), t0) > T_ESTABILIZACION_MS:
            pulse_count = 0
            t0 = utime.ticks_ms()
            if ciclo_automatico:
                estado = ESTADO_MIDIENDO
            else:
                estado = ESTADO_MANUAL

    # Modo automático: medición y ajuste de PWM
    elif estado == ESTADO_MIDIENDO:
        led_pin.value(1)
        if utime.ticks_diff(utime.ticks_ms(), t0) > T_MEDICION_MS:
            rpm = (pulse_count * 60) // PULSES_PER_REV
            timestamp = utime.ticks_ms()
            buffer.append((timestamp, duty_pct, rpm))

            if rpm > RPM_UMBRAL:
                alerta_pin.value(1)
                if not rpm_excedido:
                    print(f">> RPMs superados a un duty de {duty_pct}%")
                    rpm_excedido = True
            else:
                alerta_pin.value(0)

            if subiendo:
                duty_pct += step_value
                if duty_pct > 100:
                    duty_pct -= step_value
                    subiendo = False
            else:
                duty_pct -= step_value
                if duty_pct < 0:
                    # Finaliza ciclo automático
                    ciclo_automatico = False
                    estado = ESTADO_ESPERA
                    pin_pwm.duty_u16(0)
                    led_pin.value(0)
                    alerta_pin.value(0)
                    rpm_excedido = False
                    print(">> Fin de la captura automática. Enviando datos...")
                    print("timestamp_ms,PWM,RPM")
                    for dato in buffer:
                        print(f"{dato[0]},{dato[1]},{dato[2]}")
                    continue

            pin_pwm.duty_u16(percent_to_duty(duty_pct))
            t0 = utime.ticks_ms()
            pulse_count = 0
            estado = ESTADO_ESTABILIZANDO
