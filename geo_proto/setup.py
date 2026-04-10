from setuptools import setup

package_name = 'geo_proto'  # ROS2 package name

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],  # inner Python module folder
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='parallels',
    maintainer_email='parallels@todo.todo',
    description='Drone control ROS2 package for Jazzy',
    license='MIT',
    entry_points={
        'console_scripts': [
            'offboard_control = geo_proto.offboardControl:main',
            'offboard_control_mock = geo_proto.offboardControlMock:main',
            'run_server = geo_proto.runServer:main',
            'web_server = geo_proto.webServer:main',
            'indicator_led = geo_proto.indicator_led:main',
        ],
    },
)

