import os
import threading
# from jeus_log import *
# log = jeus_log(os.getcwd(), 'test')
# log.Warning('Warning')
# log.Error('Error')
# log.Debug('Debug')
# log.Info('Info')
# log.Critical('Critical')
# a=2
# log.Critical('Joint_%02d'%a)
# log.Critical('%02d'%a)

from jeus_armcontrol import *

manupulator = jeus_maunpulator()

if not manupulator.get_param_value(os.path.join(os.getcwd(),'Config'),'arm_config.yaml'):
    exit(1)
if not manupulator.generate_module():
    exit(1)
dt =0

manupulator.torque_on()


manupulator.move_joint(3,90)

manupulator.torque_off()
