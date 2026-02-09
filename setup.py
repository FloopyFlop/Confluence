from setuptools import setup, find_packages

package_name = 'confluence'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(include=[package_name, f"{package_name}.*"]),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=[
        'setuptools',
        'pymavlink>=2.4.40',
        'numpy',
        'pyserial',
    ],
    extras_require={
        'torch': ['torch'],
    },
    package_data={
        package_name: ['models/*.ckpt'],
    },
    zip_safe=True,
    maintainer='abm',
    maintainer_email='abm@todo.todo',
    description='ROS2 microservices: firehose, uniform pump, inject, fault detector.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'firehose = confluence.firehose:main',
            'uniform_pump = confluence.uniform_pump:main',
            'inject = confluence.inject:main',
            'fault_detector = confluence.fault_detector:main',
            'orchestrator = confluence.orchestrator:main',
            'monolithic_fault = confluence.monolithic_fault:main',
            'inject_param = confluence.hooks.inject_param:main',
            'induce_fault = confluence.hooks.induce_fault:main',
        ],
    },
)
