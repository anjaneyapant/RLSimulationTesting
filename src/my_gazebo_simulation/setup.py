from setuptools import setup

package_name = 'my_gazebo_simulation'

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
    maintainer='your_name',
    maintainer_email='your_email@example.com',
    description='Description of my_gazebo_simulation package',
    license='License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'train_trpo_turtlebot = my_gazebo_simulation.train_trpo_turtlebot:main',
        ],
    },
)
