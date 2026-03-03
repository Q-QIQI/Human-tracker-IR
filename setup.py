from setuptools import setup

import os

from glob import glob
 
package_name = 'human_tracker'
 
setup(

    name=package_name,

    version='0.0.1',

    packages=[package_name],

    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),

        ('share/' + package_name, ['package.xml']),

        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.launch.py')),

        # 🔹 Install model files
        (os.path.join('share', package_name, 'models'),
            glob('human_tracker/models/*.pt')),
    ],

    install_requires=['setuptools'],

    zip_safe=True,

    maintainer='user',

    maintainer_email='user@todo.todo',

    description='IR-Only Human Tracker for Unitree',

    license='MIT',

    tests_require=['pytest'],

    entry_points={

            'console_scripts': [

                'depth_tracker = human_tracker.depth_tracker:main',       

                'go2_follower = human_tracker.go2_follower:main',         

            ],

        },
    

)
 
