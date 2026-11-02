from glob import glob
import os
from setuptools import find_packages, setup

package_name = 'turtlebot3_nav2_web'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name] if os.path.exists('resource/' + package_name) else []),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'maps'), glob('maps/*')),
        (os.path.join('share', package_name, 'worlds'), glob('worlds/*')),
        (os.path.join('share', package_name, 'web'), ['web/index.html'] if os.path.exists('web/index.html') else []),
        (os.path.join('share', package_name, 'web', 'css'), glob('web/css/*')),
        (os.path.join('share', package_name, 'web', 'js'), glob('web/js/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ROS 2 Developer',
    maintainer_email='user@todo.todo',
    description='ROS 2 Humble Nav2 TurtleBot3 with local server and interactive UI',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'web_server_node = turtlebot3_nav2_web.web_server_node:main',
        ],
    },
)
