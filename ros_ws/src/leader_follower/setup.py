import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'leader_follower'


def model_files():
    """Gazebo models keep their textures in subfolders, so they must be
    installed preserving the directory structure."""
    out = []
    for path, _, files in os.walk('models'):
        if files:
            out.append((os.path.join('share', package_name, path),
                        [os.path.join(path, f) for f in files]))
    return out


setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*')),
        (os.path.join('share', package_name, 'config'), glob('config/*')),
        (os.path.join('share', package_name, 'params'), glob('params/*')),
        (os.path.join('share', package_name, 'rviz'), glob('rviz/*')),
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*')),
        (os.path.join('share', package_name, 'worlds'), glob('worlds/*')),
        (os.path.join('share', package_name, 'maps'), glob('maps/*')),
    ] + model_files(),
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Luigi Pio Ficco',
    maintainer_email='luigipioficco@gmail.com',
    description='Autonomous Leader-Follower Navigation with Visual Target Recovery',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'target_detector_node = leader_follower.target_detector_node:main',
            'follower_controller_node = leader_follower.follower_controller_node:main',
            'target_driver_nav2_node = leader_follower.target_driver_nav2_node:main',
            'metrics_logger_node = leader_follower.metrics_logger_node:main',
        ],
    },
)
