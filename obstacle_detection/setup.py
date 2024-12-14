from setuptools import setup
import os

package_name = 'obstacle_detection'

# get path to config file
config_path = os.path.join('obstacle_detection', 'config')

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Add configuration file
        (os.path.join('share', package_name, 'config'),
            [os.path.join(config_path, 'obstacle_avoidance.yaml')]),
    ],
    install_requires=['setuptools','PyYAML'],
    zip_safe=True,
    maintainer='root',
    maintainer_email='djnighti@ucsd.edu',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'obstacle_detector = obstacle_detection.obstacle_detection:main',  # Entry point
            'obstacle_avoider = obstacle_detection.obstacle_avoidance:main', # Entry point
        ],
    },
)

