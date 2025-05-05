import serial
import time

# Ajusta el puerto según tu sistema
PORT = 'COM10'  # En Linux/Mac podría ser /dev/ttyACM0 o /dev/ttyUSB0
BAUDRATE = 115200

ser = serial.Serial(PORT, BAUDRATE, timeout=1)

def enviar_comando(cmd):
    ser.write((cmd + '\n').encode())

def leer_linea():
    try:
        line = ser.readline().decode('utf-8').strip()
        return line
    except:
        return ''

print("Conectado a la Raspberry Pi Pico.")

while True:
    cmd = input("Comando (START <val>, PWM <val>): ")
    if cmd.upper() == "SALIR":
        break
    enviar_comando(cmd)
    time.sleep(0.1)
    while True:
        line = leer_linea()
        if not line:
            break
        print(">>", line)
