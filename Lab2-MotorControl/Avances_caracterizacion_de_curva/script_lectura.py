import serial

puerto = 'COM11'        # Cambia esto por tu puerto correcto
baudrate = 115200
nombre_archivo = 'caracterizacion.csv'

with serial.Serial(puerto, baudrate, timeout=1) as ser, open(nombre_archivo, 'w') as f:
    print("Esperando datos de Arduino...")
    while True:
        try:
            line = ser.readline().decode('utf-8').strip()
            if line:
                print(line)
                f.write(line + '\n')
        except KeyboardInterrupt:
            print("\nLectura detenida.")
            break
