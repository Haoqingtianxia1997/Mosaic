import os
from setuptools import find_packages, setup
from glob import glob

package_name = 'action'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='mosaic',
    maintainer_email='mosaic@todo.todo',
    description='IK + Move',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'move_service_default = action.move_service_default:main',
            'image_saver = action.image_saver:main',
            'move = action.move:main',
            'open = action.open:main',
            'reset = action.reset:main',
            'move_cartesian = action.move_cartesian:main',
            'stir = action.stir:main',
            'add = action.add:main',
            'close = action.close:main',
            'grasp = action.grasp:main',
            'return_back = action.return_back:main',
        ],
    },
)
