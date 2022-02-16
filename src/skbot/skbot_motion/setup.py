from setuptools import setup

package_name = 'skbot_motion'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name, ['launch/gz.launch.py'])
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
            'hello = skbot_motion.hello:main',
            'hello_scan = skbot_motion.hello_laser:main',
            'wall_follow = skbot_motion.wall_follow:main',
            'goto = skbot_motion.goto_point:main',
            'bug0 = skbot_motion.bug0:main'
        ],
    },
)
