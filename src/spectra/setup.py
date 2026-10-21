from setuptools import setup

setup(
    name='spectra',
    version='0.1.0',
    packages=['spectra'],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/spectra']),
        ('share/spectra', ['package.xml']),
    ],
    install_requires=['setuptools'],
    entry_points={'console_scripts': ['spectra_node = spectra.node:main']},
)
