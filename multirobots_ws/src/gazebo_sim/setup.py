from setuptools import find_packages, setup
from glob import glob

package_name = 'gazebo_sim'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/urdf', glob('urdf/*.urdf')),
        ('share/' + package_name + '/urdf/meshes', glob('urdf/meshes/*')),
        ('share/' + package_name + '/worlds', glob('worlds/*')),
        ('share/' + package_name + '/models/pine_tree', glob('models/pine_tree/*.*')),
        ('share/' + package_name + '/models/pine_tree/meshes', glob('models/pine_tree/meshes/*')),
        ('share/' + package_name + '/models/pine_tree/materials/scripts', glob('models/pine_tree/materials/scripts/*')),
        ('share/' + package_name + '/models/pine_tree/materials/textures', glob('models/pine_tree/materials/textures/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='root',
    maintainer_email='root@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'px4_teleop = gazebo_sim.px4_teleop:main',
            'summit_teleop = gazebo_sim.summit_teleop:main'
        ],
    },
)
