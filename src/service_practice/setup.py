from setuptools import find_packages, setup

package_name = 'service_practice'

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
    maintainer='ilyas',
    maintainer_email='ilyassilyass@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
    'console_scripts': [
        'add_server = service_practice.add_server:main',
        'add_client = service_practice.add_client:main',
        'turtle_validator = service_practice.turtle_validator_server:main',
        'turtle_control = service_practice.turtle_control_client:main',
        'calculator_server = service_practice.calculator_server:main',
        'calculator_client = service_practice.calculator_client:main',
    ],
},
)
