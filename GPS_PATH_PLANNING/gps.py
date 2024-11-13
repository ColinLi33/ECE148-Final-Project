import serial
import pynmea2

port = "/dev/ttyUSB1"

ser = serial.Serial(port, baudrate=460800, timeout=0.5)

while True:
    try:
        data = ser.readline().decode('ascii', errors='replace')
        print(data)
        if data.startswith('$GPGGA'):
            msg = pynmea2.parse(data)
            print(f"Latitude: {msg.latitude}, Longitude: {msg.longitude}")
    except serial.SerialException as e:
        print('Device error: {}'.format(e))
        break
    except pynmea2.ParseError as e:
        print('Parse error: {}'.format(e))
        continue