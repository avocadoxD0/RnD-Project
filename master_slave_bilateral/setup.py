from setuptools import find_packages, setup

package_name = 'master_slave_bilateral'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='rika',
    maintainer_email='rika@todo.todo',
    description='Master–Slave Bilateral Control System',
    license='TODO: License declaration',
    entry_points={
        'console_scripts': [
            'testmotion = master_slave_bilateral.testmotion:main',
            'bilateral_control = master_slave_bilateral.bilateral_control:main',
            'hybrid_bilateral = master_slave_bilateral.hybrid_bilateral_node:main',
            'bilateral_control_v2 = master_slave_bilateral.bilateral_control_v2:main'
        ],
    },
)
