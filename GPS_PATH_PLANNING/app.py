from flask import Flask, render_template, jsonify, request
import threading
from createMap import parseMap
from path_follower import PathFollower
from gps import GPS

app = Flask(__name__)
gps = GPS()
path_follower = PathFollower(gps)

print("Loading Graph...")
ucsdMap = parseMap("./static/ucsd.geojson")
print("Generating Map...")
ucsdMap.generateMap()


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
    interpolatedPath = path_follower.interpolate_path(formattedPath, 5)
    path_follower.save_path(interpolatedPath)
    threading.Thread(target=path_follower.follow_path, args=(interpolatedPath,)).start()

    return jsonify({"path": interpolatedPath})


@app.route('/get_location')
def get_location():
    # print(gps.current_location)
    if gps.current_location["lat"] is not None and gps.current_location["long"] is not None:
        return jsonify(gps.current_location)
    return jsonify({"error": "No GPS fix"}), 404


if __name__ == '__main__':
    print("Starting GPS thread...")
    gps_thread = threading.Thread(target=gps.update, daemon=True)
    gps_thread.start()

    print("Starting Flask server...")
    app.run('0.0.0.0', threaded=True)