
from setuptools import setup, find_packages
import platform

setup(
    name='jeus_armcontrol',
    version='0.0.0',
    packages=find_packages(include=['jeus_armcontrol','dynamixel_sdk']),
    # package_dir={'jeus_armcontrol': 'src','dynamixel_sdk': 'src','jeus_vision': 'src','modules': 'src.eus_vision','utils': 'src.jeus_vision'},
    package_dir={'': 'src'},
    license='None',
    description='Jeus arm control python package',
    long_description=open('README.txt').read(),
    url='https://github.com/Code-YMJ/Maunpulator_Unit_Test',
    author='Code.MJ',
    author_email='minjae8970@gmail.com',
    install_requires=['pyserial','numpy', 'pyyaml', 'PySide6']
)

# setup(
#     name='jeus_armcontrol',
#     version='0.0.0',
#     packages=['jeus_armcontrol'],
#     package_dir={'': 'src'},
#     license='None',
#     description='Jeus arm control python package',
#     long_description=open('README.txt').read(),
#     url='https://github.com/Code-YMJ/Maunpulator_Unit_Test',
#     author='Code.MJ',
#     author_email='minjae8970@gmail.com',
#     install_requires=['pyserial','numpy', 'pyyaml', 'PySide6']
# )
setup(
    name='jeus_vision',
    version='0.0.0',
    packages=find_packages(include=['jeus_vision','jeus_vision.modules','jeus_vision.utils']),

    package_dir={'': 'src'},
    license='None',
    description='Jeus arm control python package',
    long_description=open('README.txt').read(),
    url='https://github.com/Code-YMJ/Maunpulator_Unit_Test',
    author='Code.MJ',
    author_email='minjae8970@gmail.com',
    install_requires=[ ]
)

# setup(
#     name='jeus_yolo',
#     version='0.0.0',
#     packages=['jeus_yolo'],
#     package_dir={'': 'src'},
#     license='None',
#     description='Jeus arm control python package',
#     long_description=open('README.txt').read(),
#     url='https://github.com/Code-YMJ/Maunpulator_Unit_Test',
#     author='Code.MJ',
#     author_email='minjae8970@gmail.com',
#     install_requires=[ ]
# )