# motor_control_pico.py
# Control de velocidad de motor y medición de RPM usando un sensor Hall
# Modo automático y medición en Raspberry Pi Pico

from machine import Pin, PWM
import utime
import sys
import select

# ----------------------------
# Parámetros configurables
# ----------------------------
PULSES_PER_REV = 20          # Pulsos por revolución del sensor Hall
PWM_FREQ_HZ = 500           # Frecuencia del PWM
T_MEDICION_MS = 1000         # Tiempo de medición de RPM
RPM_UMBRAL = 5000            # Umbral de RPM para activar alerta

# ----------------------------
# Pines
# ----------------------------
motor_pwm = PWM(Pin(18))
motor_pwm.freq(PWM_FREQ_HZ)

encoder = Pin(16, Pin.IN, Pin.PULL_UP)
PWM_MAX = 47 # Valor máximo del ciclo de trabajo (100%) 47% de 65535
PWM_MIN = 0 # Valor mínimo del ciclo de trabajo (0%) 19% de 65535
# ----------------------------
# Variables globales
# ----------------------------
pulse_count = 0
duty = 0
step = 0
subiendo = True
ciclo_activo = False
rpm_excedido = False
datos = []
estado = "ESPERA"
t0 = utime.ticks_ms()

# ----------------------------
# Funciones auxiliares
# ----------------------------

def percent_to_duty(pwm_step):
    adjusted_duty = PWM_MIN + (pwm_step * (PWM_MAX - PWM_MIN) / 100) # Ajuste del rango de PWM
    return int(adjusted_duty * 65535  / 100) # Conversión a duty_u16

def contar_pulsos(pin):
    global pulse_count
    pulse_count += 1

encoder.irq(trigger=Pin.IRQ_FALLING, handler=contar_pulsos)

def leer_comando_serial():
    if sys.stdin in select.select([sys.stdin], [], [], 0)[0]:
        return sys.stdin.readline().strip()
    return None

def calcular_rpm(pulsos, t_ms):
    revs = pulsos / PULSES_PER_REV
    rpm = (revs * 60000) / t_ms
    return int(rpm)

# ----------------------------
# Bucle principal
# ----------------------------
print(">> Esperando comando serial: START <paso> o PWM <valor>")

while True:
    comando = leer_comando_serial()
    if comando:
        if comando.upper().startswith("START "):
            try:
                step = int(comando.split()[1])
                if not (1 <= step <= 100):
                    raise ValueError
                ciclo_activo = True
                duty = 0
                subiendo = True
                datos.clear()
                rpm_excedido = False
                motor_pwm.duty_u16(percent_to_duty(duty))
                t0 = utime.ticks_ms()
                estado = "ESTABILIZANDO"
                print(">> Ciclo automático iniciado con paso =", step)
            except:
                print(">> Error: Usa START <valor entre 1 y 100>")
        elif comando.upper().startswith("PWM "):
            try:
                duty = int(comando.split()[1])
                if not (0 <= duty <= 100):
                    raise ValueError
                ciclo_activo = False
                motor_pwm.duty_u16(percent_to_duty(duty))
                t0 = utime.ticks_ms()
                estado = "MANUAL"
                print(">> PWM manual fijado en", duty, "%")
            except:
                print(">> Error: Usa PWM <valor entre 0 y 100>")

    # Estado: Estabilización
    if estado == "ESTABILIZANDO":
        if utime.ticks_diff(utime.ticks_ms(), t0) >= 3000:
            pulse_count = 0
            t0 = utime.ticks_ms()
            estado = "MIDIENDO"

    # Estado: Medición
    elif estado == "MIDIENDO":
        if utime.ticks_diff(utime.ticks_ms(), t0) >= T_MEDICION_MS:
            rpm = calcular_rpm(pulse_count, T_MEDICION_MS)
            if ciclo_activo:
                datos.append((utime.ticks_ms(), duty, rpm))
                if rpm > RPM_UMBRAL and not rpm_excedido:
                    rpm_excedido = True

                # Ajuste de duty
                if subiendo:
                    duty += step
                    if duty > 100:
                        duty -= step
                        subiendo = False
                else:
                    duty -= step
                    if duty < 0:
                        print(">> Fin del ciclo automático. Datos:")
                        print("timestamp_ms,PWM,RPM")
                        for d in datos:
                            print(f"{d[0]},{d[1]},{d[2]}")
                        motor_pwm.duty_u16(0)
                        estado = "ESPERA"
                        continue

                motor_pwm.duty_u16(percent_to_duty(duty))
                t0 = utime.ticks_ms()
                pulse_count = 0
                estado = "ESTABILIZANDO"
            else:
                print(f">> PWM: {duty}% RPM: {rpm}")
                t0 = utime.ticks_ms()
                pulse_count = 0

    # Estado manual: muestra RPM continuamente
    elif estado == "MANUAL":
        if utime.ticks_diff(utime.ticks_ms(), t0) >= T_MEDICION_MS:
            rpm = calcular_rpm(pulse_count, T_MEDICION_MS)
            print(f">> PWM: {duty}% RPM: {rpm}")
            pulse_count = 0
            t0 = utime.ticks_ms()
