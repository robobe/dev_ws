from setuptools import setup
import os
from glob import glob

package_name = 'basic'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join("share", package_name), glob("launch/*.launch.py")),
        (os.path.join("share", package_name, "config"), glob("config/*")),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='user',
    maintainer_email='robo2020@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "simple_pub = basic.simple_pub:main",
            "simple_sub = basic.simple_sub:main",
            "simple_param = basic.simple_param:main",
            "simple_param2 = basic.simple_param2:main",
            "simple_param3 = basic.simple_param3:main",
            "simple_srv = basic.simple_service:main",
            "simple_client = basic.simple_client:main"

        ],
    },
)
