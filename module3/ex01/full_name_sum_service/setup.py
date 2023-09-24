from setuptools import find_packages, setup

package_name = 'full_name_sum_service'

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
    description='PYTHON CLIENT SERVER TUTORIAL',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        'service_name = full_name_sum_service.service_member_function:main',
        'client_name = full_name_sum_service.client_member_function:main',
        ],
    },
)
