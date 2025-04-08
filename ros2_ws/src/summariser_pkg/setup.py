from setuptools import find_packages, setup

package_name = 'summariser_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools', 'transformers'],
    zip_safe=True,
    maintainer='user',
    maintainer_email='user@todo.todo',
    description='Summarises voice-command text using HuggingFace models.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'summariser_node = summariser_pkg.summariser_node:main',
        ],
    },
)