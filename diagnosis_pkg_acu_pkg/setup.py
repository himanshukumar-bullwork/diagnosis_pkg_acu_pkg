from setuptools import find_packages, setup
from glob import glob
import os

package_name = 'diagnosis_pkg_acu_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        # Ament index + package.xml
        ('share/ament_index/resource_index/packages', [os.path.join('resource', package_name)]),
        (os.path.join('share', package_name), ['package.xml']),
        # Launch files
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        # Config files
        (os.path.join('share', package_name, 'config_file'), glob('config_file/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='root',
    maintainer_email='vachel@bullworkmobility.com',
    description='Diagnostics for CAN, GPS, LiDAR, Nav2, Network, RL, ZED, and OS.',
    license='Apache-2.0',
    extras_require={'test':['pytest']},
    entry_points={
        'console_scripts': [
            'can = diagnosis_pkg_acu_pkg.can:main',
            'gps = diagnosis_pkg_acu_pkg.gps:main',
            'lidar = diagnosis_pkg_acu_pkg.lidar:main',
            'nav2_diagnosis = diagnosis_pkg_acu_pkg.nav2_diagnosis:main',
            'network_diag = diagnosis_pkg_acu_pkg.network_diag:main',
            'robot_localization_ekf_core = diagnosis_pkg_acu_pkg.robot_localization_ekf:main',
            'zed_x_diag = diagnosis_pkg_acu_pkg.zed_x_diag:main',
            'sensor_setup_automation = diagnosis_pkg_acu_pkg.sensor_setup_automation:main',
        ],
    },
)
