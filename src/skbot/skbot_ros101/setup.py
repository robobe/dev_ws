from setuptools import setup

package_name = 'skbot_ros101'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
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
            "activate_srv=skbot_ros101.simple_server:main",
            "simple_async_client=skbot_ros101.simple_async_client:main",
            "simple_sync_client=skbot_ros101.simple_sync_client:main",
            "pub=skbot_ros101.simple_pub:main",
            "sub=skbot_ros101.simple_sub:main",
            "all=skbot_ros101.simple_comp:main"
        ],
    },
)
