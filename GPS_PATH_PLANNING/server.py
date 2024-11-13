from flask import Flask, render_template, jsonify, request
from createMap import parseMap
import math

app = Flask(__name__)
graph = {}


def lat_long_difference(origin, node):
    lat_to_meters = 111320
    long_to_meters = 40075000 * math.cos(math.radians(origin[0])) / 360

    latDiff = (node[0] - origin[0]) * lat_to_meters
    longDiff = (node[1] - origin[1]) * long_to_meters
    return latDiff, longDiff

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
    start = tuple(data['start'])
    end = tuple(data['end'])
    path = ucsdMap.shortestPath(start, end)
    if path is None:
        return jsonify({"error": "No path found"}), 404
    
    formattedPath = [[node[0], node[1]] for node in path]
    savePath(formattedPath)

    return jsonify({"path": formattedPath})

if __name__ == '__main__':
    print("Loading Graph...")
    ucsdMap = parseMap("./static/ucsd.geojson")
    print("Generating Map...")
    ucsdMap.generateMap()
    print("Server Starting...")
    app.run('0.0.0.0')
