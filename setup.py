
from setuptools import setup, find_packages
import platform

setup(
    name='dynamixel_sdk',
    version='3.7.31',
    packages=['dynamixel_sdk'],
    package_dir={'': 'src'},
    license='Apache 2.0',
    description='Dynamixel SDK 3. python package',
    long_description=open('README.txt').read(),
    url='https://github.com/ROBOTIS-GIT/DynamixelSDK',
    author='Leon Jung',
    author_email='rwjung@robotis.com',
    install_requires=['pyserial']
)

setup(
    name='jeus_armcontrol',
    version='0.0.0',
    packages=['jeus_armcontrol'],
    package_dir={'': 'src'},
    license='None',
    description='Jeus arm control python package',
    long_description=open('README.txt').read(),
    url='https://github.com/Code-YMJ/Maunpulator_Unit_Test',
    author='Code.MJ',
    author_email='minjae8970@gmail.com',
    install_requires=['numpy', 'pyyaml']
)

