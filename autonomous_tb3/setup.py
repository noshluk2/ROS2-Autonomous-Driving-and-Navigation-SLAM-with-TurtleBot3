from setuptools import setup
import os
from glob import glob
package_name = 'autonomous_tb3'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share',package_name,'launch') , glob('launch/*')),
        (os.path.join('share',package_name,'config') , glob('config/*')),
        (os.path.join('share',package_name,'world/maze') , glob('world/maze/*')),
        (os.path.join('share',package_name,'models/actor') , glob('models/actor/*')),
        (os.path.join('share',package_name,'models/table') , glob('models/table/*')),
        (os.path.join('share',package_name,'models/beer') , glob('models/beer/*.sdf')),
        (os.path.join('share',package_name,'models/beer') , glob('models/beer/*.config')),
        (os.path.join('share',package_name,'models/beer/scripts') , glob('models/beer/scripts/*')),
        (os.path.join('share',package_name,'models/beer/texutres') , glob('models/beer/texutres/*')),


    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='luqman',
    maintainer_email='noshluk2@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'occupancy_grid_pub = autonomous_tb3.occupancy_grid_pub:main',
            'sdf_spawner = autonomous_tb3.spawn_entity:main',
            'maze_solver = autonomous_tb3.maze_solver:main',
            'autonomous_waiter_lite = autonomous_tb3.hotel_waiter_single_goal:start_app',
            'autonomous_waiter = autonomous_tb3.hotel_waiter_multi_button:start_app'
        ],
    },
)
