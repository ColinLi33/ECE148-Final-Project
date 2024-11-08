from flask import Flask, render_template, jsonify, request
from createMap import parseMap
import json

app = Flask(__name__)
graph = {}

@app.route('/')
def index():
    return render_template('map.html')

@app.route('/generate_path', methods=['POST'])
def generate_path():
    data = request.json
    start = tuple(data['start'])
    end = tuple(data['end'])
    print("SERVER: ", start,end)
    path = ucsdMap.shortestPath(start, end)
    if path is None:
        return jsonify({"error": "No path found"}), 404
    
    formatted_path = [[node[0], node[1]] for node in path]
    
    return jsonify({"path": formatted_path})

if __name__ == '__main__':
    print("Loading Graph...")
    ucsdMap = parseMap("./static/ucsd.geojson")
    print("Generating Map...")
    ucsdMap.generateMap()
    print("Server Starting...")
    app.run()
