import serial
import pynmea2
import math


class GPS:
    def __init__(self):
        # self.current_location = {"lat": None, "long": None, "heading": None}
        self.current_location = {"lat": 32.8808, "long": -117.2374, "heading": 0}

    def update(self):
        port = "/dev/ttyUSB1"
        ser = serial.Serial(port, baudrate=460800, timeout=0.5)
        print("Started GPS Thread")

        while True:
            try:
                data = ser.readline().decode('ascii', errors='replace')
                if data.startswith('$GNRMC'):  # For heading data
                    msg = pynmea2.parse(data)
                    self.current_location["lat"] = msg.latitude
                    self.current_location["long"] = msg.longitude
                    if msg.true_course is not None:  # Check if heading is available
                        self.current_location["heading"] = msg.true_course
            except serial.SerialException as e:
                print('Device error: {}'.format(e))
                self.current_location = {"lat": 32.8808, "long": -117.2374, "heading": 0}
            except pynmea2.ParseError as e:
                print('Parse error: {}'.format(e))
                continue
