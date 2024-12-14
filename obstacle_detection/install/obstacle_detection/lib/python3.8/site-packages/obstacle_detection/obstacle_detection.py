import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from depthai_sdk import OakCamera
from depthai_sdk.visualize.configs import StereoColor
from depthai_sdk.classes.packets import DisparityDepthPacket
from depthai_sdk.visualize.visualizers.opencv_visualizer import OpenCvVisualizer
import math
import depthai as dai
import cv2

# User-defined constants for distance thresholds
WARNING = 10.0  # Warning distance threshold: 10 meters
CRITICAL = 3.0  # Critical distance threshold: 3 meters

# Parameters for region filtering
left_bound = 3   # Exclude the left 2 columns
right_bound = 3  # Exclude the right 2 columns
top_bound = 2    # Exclude the top 2 rows
bottom_bound = 2 # Exclude the bottom 1 row

# Video display control: 0-off, 1-on
video = 1

# Calculate the number of valid rows and columns
total_cols = 15 - left_bound - right_bound  # Total valid columns
total_rows = 9 - top_bound - bottom_bound   # Total valid rows
queue_size = total_rows * total_cols        # Set queue size to match the number of regions

# Number of data points to display
num_to_display = 3

# Define a ROS 2 Node class
class ObstacleDetectionPublisher(Node):
    def __init__(self):
        super().__init__('obstacle_detection')  # Node name
        self.publisher_ = self.create_publisher(Float32MultiArray, 'object_detection', queue_size)
        self.get_logger().info("Obstacle Detection Node Started")

    def publish_frame_data(self, frame_data):
        # Create a Float32MultiArray message
        msg = Float32MultiArray()
        msg.data = frame_data  # Add all detected objects for the frame
        self.publisher_.publish(msg)  # Publish the message

        # Calculate the number of detected objects
        num_objects = len(frame_data) // 3

        # Log the count of objects
        log_message = f"Published {num_objects} objects in current frame"

        # Add the first 'num_to_display' objects to the log
        if num_objects > 0:
            num_to_show = min(num_to_display, num_objects)  # Show up to 'num_to_display' objects
            displayed_data = []
            for i in range(num_to_show):
                x, y, distance = frame_data[i * 3:(i + 1) * 3]
                displayed_data.append([x, y, distance])
            log_message += f": {displayed_data}"

        # Log the message
        self.get_logger().info(log_message)


# Main code logic using OakCamera
def main():
    rclpy.init()  # Initialize ROS 2
    node = ObstacleDetectionPublisher()  # Create an instance of the ROS 2 node

    with OakCamera() as oak:
        stereo = oak.create_stereo('480p')  # Use a valid resolution like '720p'

        # Configure depth postprocessing settings for accuracy
        config = stereo.node.initialConfig.get()
        config.postProcessing.brightnessFilter.minBrightness = 0
        config.postProcessing.brightnessFilter.maxBrightness = 255
        stereo.node.initialConfig.set(config)

        # Set depth visualization with RGBD colorization and a bone colormap
        stereo.config_postprocessing(colorize=StereoColor.RGBD, colormap=cv2.COLORMAP_BONE)
        stereo.config_stereo(confidence=50, lr_check=True, extended=True)

        # Create a SpatialLocationCalculator node for region-based depth calculations
        slc = oak.pipeline.create(dai.node.SpatialLocationCalculator)

        for x in range(left_bound, 15 - right_bound):  # Skip the left and right excluded columns
            for y in range(top_bound, 9 - bottom_bound):  # Skip the top and bottom excluded rows
                config = dai.SpatialLocationCalculatorConfigData()
                config.depthThresholds.lowerThreshold = 200  # Minimum depth threshold in mm
                config.depthThresholds.upperThreshold = 10000  # Maximum depth threshold in mm
                config.roi = dai.Rect(dai.Point2f((x + 0.5) * 0.0625, (y + 0.5) * 0.1),
                                      dai.Point2f((x + 1.5) * 0.0625, (y + 1.5) * 0.1))
                config.calculationAlgorithm = dai.SpatialLocationCalculatorAlgorithm.MEDIAN
                slc.initialConfig.addROI(config)

        stereo.depth.link(slc.inputDepth)

        slc_out = oak.pipeline.create(dai.node.XLinkOut)
        slc_out.setStreamName('slc')
        slc.out.link(slc_out.input)

        stereoQ = oak.queue(stereo.out.depth).get_queue()
        oak.start()

        slcQ = oak.device.getOutputQueue('slc')
        vis = OpenCvVisualizer()

        # Main loop to process frames and calculate distances
        while oak.running():
            oak.poll()  # Poll the OakCamera for updates
            packet: DisparityDepthPacket = stereoQ.get()  # Retrieve the latest depth packet
            slc_data = slcQ.get().getSpatialLocations()  # Retrieve spatial location data
            depthFrameColor = packet.get_colorized_frame(vis)  # Get the colorized depth frame
            frame_data = []

            for depthData in slc_data:
                roi = depthData.config.roi
                coords = depthData.spatialCoordinates
                distance = math.sqrt(coords.x**2 + coords.y**2 + coords.z**2) / 1000.0  # Convert to meters

                if distance == 0:  # Skip invalid distance values
                    continue

                # Add to the list if within WARNING or CRITICAL range
                x_m = coords.x / 1000.0
                y_m = coords.y / 1000.0
                if y_m < -0.18 and distance > 0.95:
                    continue
                if y_m < -0.15 and distance > 1.45:
                    continue
                if distance < CRITICAL or distance < WARNING:                   
                    frame_data.extend([x_m, y_m, distance])  # Add x, y, distance as a flat list

                # Highlight regions based on distance thresholds (for visualization)
                if video:
                    xmin = int(roi.topLeft().x * depthFrameColor.shape[1])
                    ymin = int(roi.topLeft().y * depthFrameColor.shape[0])
                    xmax = int(roi.bottomRight().x * depthFrameColor.shape[1])
                    ymax = int(roi.bottomRight().y * depthFrameColor.shape[0])

                    if distance < CRITICAL:
                        color = (0, 0, 255)
                        cv2.rectangle(depthFrameColor, (xmin, ymin), (xmax, ymax), color, thickness=4)
                        cv2.putText(depthFrameColor, "{:.1f}m".format(distance),
                                    (xmin + 10, ymin + 20), cv2.FONT_HERSHEY_TRIPLEX, 0.5, color)
                    elif distance < WARNING:
                        color = (0, 140, 255)
                        cv2.rectangle(depthFrameColor, (xmin, ymin), (xmax, ymax), color, thickness=2)
                        cv2.putText(depthFrameColor, "{:.1f}m".format(distance),
                                    (xmin + 10, ymin + 20), cv2.FONT_HERSHEY_TRIPLEX, 0.5, color)

            # Publish all detected objects for the current frame
            if frame_data:
                node.publish_frame_data(frame_data)

            # Display the processed frame
            if video:
                cv2.imshow('Frame', depthFrameColor)

        rclpy.shutdown()  # Shut down ROS 2 on exit


if __name__ == '__main__':
    main()


