from flask import Flask, render_template, jsonify, request
from createMap import parseMap
import math
import serial
import pynmea2
import threading
import numpy as np

app = Flask(__name__)
graph = {}
currentLocation = {"lat": None, "long": None}
# currentLocation = {"lat": 32.8750165, "long": -117.2413341}
def update_gps():
    port = "/dev/ttyUSB1"
    ser = serial.Serial(port, baudrate=460800, timeout=0.5)
    print("Started GPS Thread")
    
    while True:
        try:
            data = ser.readline().decode('ascii', errors='replace')
            if data.startswith('$GNGLL'):
                msg = pynmea2.parse(data)
                currentLocation["lat"] = msg.latitude
                currentLocation["long"] = msg.longitude
        except serial.SerialException as e:
            print('Device error: {}'.format(e))
            break
        except pynmea2.ParseError as e:
            print('Parse error: {}'.format(e))
            continue

def lat_long_difference(origin, node):
    lat_to_meters = 111320
    long_to_meters = 40075000 * math.cos(math.radians(origin[0])) / 360

    latDiff = (node[0] - origin[0]) * lat_to_meters
    longDiff = (node[1] - origin[1]) * long_to_meters
    return latDiff, longDiff

def interpolatePath(path, num_points_between=5):
    points = np.array(path)
    t = np.linspace(0, 1, len(points))
    fx = np.interp(np.linspace(0, 1, (len(points)-1) * num_points_between + 1), t, points[:, 0])
    fy = np.interp(np.linspace(0, 1, (len(points)-1) * num_points_between + 1), t, points[:, 1])
    interpolated_points = [[x, y] for x, y in zip(fx, fy)]
    return interpolated_points

def savePath(path):
    with open('./static/path.csv', 'w') as f:
        origin = path[0]
        for node in path:
            latDiff, longDiff = lat_long_difference(origin, node)
            f.write(f'{latDiff},{longDiff},{0.5}\n')
    f.close()
    print("Path saved to path.csv")

@app.route('/')
def index():
    return render_template('map.html')

@app.route('/generate_path', methods=['POST'])
def generate_path():
    data = request.json
    start_point = tuple(data['start'])
    end_point = tuple(data['end'])

    if not ucsdMap.addTemporaryNode(start_point, max_distance=50):
        return jsonify({"error": "Current location too far from known paths"}), 400

    path = ucsdMap.shortestPath(start_point, end_point)
    
    ucsdMap.removeNode(start_point)
    if path is None:
        return jsonify({"error": "No path found"}), 404
    
    formattedPath = [[node[0], node[1]] for node in path]
    interpolatedPath = interpolatePath(formattedPath)
    savePath(interpolatedPath)
    return jsonify({"path": interpolatedPath})

@app.route('/get_location')
def get_location():
    print(currentLocation)
    if currentLocation["lat"] is not None and currentLocation["long"] is not None:
        return jsonify(currentLocation)
    return jsonify({"error": "No GPS fix"}), 404

if __name__ == '__main__':
    print("Loading Graph...")
    ucsdMap = parseMap("./static/ucsd.geojson")
    print("Generating Map...")
    ucsdMap.generateMap()
    print("Server Starting...")
    gps_thread = threading.Thread(target=update_gps, daemon=True)
    gps_thread.start()
    app.run('0.0.0.0', threaded=True)
