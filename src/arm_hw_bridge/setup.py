from glob import glob
from setuptools import find_packages, setup

package_name = 'arm_hw_bridge'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/rviz', glob('rviz/*.rviz')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='instillmotion',
    maintainer_email='monish.saxena92@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'bridge = arm_hw_bridge.bridge:main',
            'live_bridge = arm_hw_bridge.live_bridge:main',
            'direct_drag = arm_hw_bridge.direct_drag:main',
            'mouse_arm_direct = arm_hw_bridge.mouse_arm_direct:main',
            'mouse_base_control = arm_hw_bridge.mouse_base_control:main',
            'mouse_base_direct = arm_hw_bridge.mouse_base_direct:main',
        ],
    },
)
