from setuptools import find_packages, setup
from glob import glob
import os

package_name = 'urdf_odom_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
        (os.path.join("share", package_name, "config"), glob("config/*.yaml")),

        # maps folder
        (os.path.join("share", package_name, "maps"), ["maps/.gitkeep"]),
        (os.path.join("share", package_name, "maps"), glob("maps/*.yaml")),
        (os.path.join("share", package_name, "maps"), glob("maps/*.pgm")),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='user',
    maintainer_email='alianlbj23@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            "wheel_odom_node = urdf_odom_pkg.wheel_odom_node:main",
            "wheel_twist_node = urdf_odom_pkg.wheel_twist_node:main",
        ],
    },
)
