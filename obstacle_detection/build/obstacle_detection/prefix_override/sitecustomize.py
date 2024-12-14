import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/projects/ros2_ws/obstacle_detection/install/obstacle_detection'
