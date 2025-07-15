from setuptools import setup

package_name = 'color_detector'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
    ('share/ament_index/resource_index/packages',
        ['resource/' + package_name]),
    ('share/' + package_name, ['package.xml']),
    ('share/' + package_name + '/launch', ['launch/multi_detector.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Shir',
    maintainer_email='shir@example.com',
    description='Color detection and azimuth publisher for drones',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'color_detector_node = color_detector.color_detector_node:main',
            'azimuth_computer_node = color_detector.azimuth_computer_node:main',
        ],
    },
)
