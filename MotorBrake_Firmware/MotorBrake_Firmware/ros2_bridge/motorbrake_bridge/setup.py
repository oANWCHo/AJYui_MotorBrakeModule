from setuptools import setup

package_name = 'motorbrake_bridge'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools', 'python-can'],
    zip_safe=True,
    maintainer='anawach',
    maintainer_email='anawach.claude@gmail.com',
    description='Bridges STM32 MotorBrake CAN frames to/from ROS 2 topics.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'brake_bridge = motorbrake_bridge.brake_bridge:main',
        ],
    },
)
