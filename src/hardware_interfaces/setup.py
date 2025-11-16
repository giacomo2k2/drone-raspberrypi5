from setuptools import find_packages, setup

package_name = 'hardware_interfaces'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', ['config/sitl_camera_configuration.yaml']),
        ('share/' + package_name + '/config', ['config/cc_camera_configuration.yaml'])
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='giacomo',
    maintainer_email='pezzolati.g@gmail.com',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'detection_image_node = hardware_interfaces.detection_image_node:main',
            'camera_calibration_node = hardware_interfaces.camera_calibration_node:main',
            'video_logger_node = hardware_interfaces.video_logger_node:main',
            'camera_print_node = hardware_interfaces.camera_print_node:main',
        ],
    },
)
