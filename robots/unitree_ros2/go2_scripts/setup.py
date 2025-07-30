from setuptools import find_packages, setup

package_name = 'go2_scripts'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(where='src', exclude=['test']),
    package_dir={'': 'src'},
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=[
        'setuptools',
        'pyzmq',
        'opencv-python',
        'numpy'
    ],
    zip_safe=True,
    maintainer='Unitree',
    maintainer_email='unitree@unitree.com',
    description='Unitree ROS2 scripts package containing Python nodes for robot control',
    license='BSD 3-Clause License',
    entry_points={
        'console_scripts': [
            'go2_node = go2_scripts.nodes.go2_node:main',
            'simple_move_forward = go2_scripts.examples.simple_move_forward:main',
        ],
    },
)