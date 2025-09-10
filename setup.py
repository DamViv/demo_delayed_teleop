from setuptools import setup

package_name = 'demo_delayed_teleop'

setup(
    name=package_name,
    version='0.0.1',
    packages=['demo_delayed_teleop'],  # nom du dossier Python
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='TonNom',
    maintainer_email='ton.email@example.com',
    description='Teleop with delay and safety override',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'safety_node = demo_delayed_teleop.safety_node:main',
            'color_detector = demo_delayed_teleop.color_detector:main',
        ],
    },
)
