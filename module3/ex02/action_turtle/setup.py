from setuptools import find_packages, setup

package_name = 'action_turtle'

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
    maintainer='nastya',
    maintainer_email='nastasapervaa@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        'action_turtle_server = action_turtle.action_server:main',
        'action_turtle_client = action_turtle.action_client:main',
        
        ],
    },
)
