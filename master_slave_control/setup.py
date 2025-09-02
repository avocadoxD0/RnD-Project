from setuptools import setup, find_packages

package_name = 'master_slave_control'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/mirror_launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Farhaan Nasir',  # 👈 put your name here
    maintainer_email='farhaannasir2005@gmail.com',  # 👈 your email here
    description='Master-Slave control for two robotic arms',
    license='MIT',
    entry_points={
        'console_scripts': [
            'mirror_nodes = master_slave_control.mirror_nodes:main',
        ],
    },
)
