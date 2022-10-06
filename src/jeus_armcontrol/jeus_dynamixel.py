from dataclasses import dataclass
from sqlite3 import connect
from dynamixel_sdk import *
import numpy as np
import os
import time
import sys
from jeus_log import *


ctl_table = {}
# -------------------------------------------
# EEPROM region 
# ctl_table['ADDR_MODEL_NUM']  = 0   # size=2 access=R  init_value=1020
ctl_table['ADDR_MODEL_NUM']  = {'addr':0,  'size':2, 'access':'R' } # init_value=1020
ctl_table['ADDR_MODEL_INFO'] = {'addr':2,  'size':4, 'access':'R' } # init_value=-
ctl_table['ADDR_FW_VER']     = {'addr':6,  'size':1, 'access':'R' } # init_value=-
ctl_table['ADDR_DXL_ID']     = {'addr':7,  'size':1, 'access':'RW'} # init_value=1. 0~252
ctl_table['ADDR_BAUD_RATE']  = {'addr':8,  'size':1, 'access':'RW'} # init_value=1. 0~7
ctl_table['ADDR_RT_DELAY']   = {'addr':9,  'size':1, 'access':'RW'} # init_value=250. status packet delay after instruction packet. 2us unit. possible value = 0~254.
ctl_table['ADDR_DRV_MODE']   = {'addr':10, 'size':1, 'access':'RW'} # init_value=0. 
ctl_table['ADDR_OPR_MODE']   = {'addr':11, 'size':1, 'access':'RW'} # init_value=3
ctl_table['ADDR_2ND_ID']     = {'addr':12, 'size':1, 'access':'RW'} # init_value=255. 0~252. 255 means disabled
ctl_table['ADDR_PROT_TY']    = {'addr':13, 'size':1, 'access':'RW'} # init_value=2
ctl_table['ADDR_HOME_OFS']   = {'addr':20, 'size':4, 'access':'RW'} # init_value=0. one pulse unit. -1024 ~ +1024
ctl_table['ADDR_MOV_THR']    = {'addr':24, 'size':4, 'access':'RW'} # init_value=10. 0.229rpm unit. 0~1023
ctl_table['ADDR_TEMP_LIM']   = {'addr':31, 'size':1, 'access':'RW'} # init_value=72. 1 deg unit. 0~100
ctl_table['ADDR_MAX_VLIM']   = {'addr':32, 'size':2, 'access':'RW'} # init_value=140. 0.1V unit. 95~160 
ctl_table['ADDR_MIN_VLIM']   = {'addr':34, 'size':2, 'access':'RW'} # init_value=60. 0.1V unit. 95~160 
ctl_table['ADDR_PWM_LIM']    = {'addr':36, 'size':2, 'access':'RW'} # init_value=885. 0.113% unit. 0~885
ctl_table['ADDR_CURR_LIM']   = {'addr':38, 'size':2, 'access':'RW'} # init_value=1193. 2.69mA unit. 0~1193 
ctl_table['ADDR_VEL_LIM']    = {'addr':44, 'size':4, 'access':'RW'} # init_value=200. 0~1023. 0.229rpm
ctl_table['ADDR_MAX_PLIM']   = {'addr':48, 'size':4, 'access':'RW'} # init_value=4095. one pulse unit. 0~4095 
ctl_table['ADDR_MAX_PLIM']   = {'addr':52, 'size':4, 'access':'RW'} # init_value=4095. one pulse unit. 0~4095 
ctl_table['ADDR_START_CONF'] = {'addr':60, 'size':1, 'access':'RW'} # init_value=3
ctl_table['ADDR_SHUTDOWN']   = {'addr':63, 'size':1, 'access':'RW'} # init_value=52. Define shutdown condition.
# -------------------------------------------
# RAM region 
ctl_table['ADDR_TQ_ENB']     = {'addr':64,  'size':1, 'access':'RW'} # init_value=0. 0~1
ctl_table['ADDR_LED_STS']    = {'addr':65,  'size':1, 'access':'RW'} # init_value=0. 0~1
ctl_table['ADDR_RTN_TY']     = {'addr':68,  'size':1, 'access':'RW'} # init_value=2. 0~2. Ruturn for all instruction
ctl_table['ADDR_REG_INST']   = {'addr':69,  'size':1, 'access':'R' } # init_value=0. 0~1
ctl_table['ADDR_HW_ERR']     = {'addr':70,  'size':1, 'access':'R' } # init_value=0. -
ctl_table['ADDR_VEL_IGAIN']  = {'addr':76,  'size':2, 'access':'RW'} # init_value=1000. 0~16383
ctl_table['ADDR_VEL_PGAIN']  = {'addr':78,  'size':2, 'access':'RW'} # init_value=100. 0~16383
ctl_table['ADDR_POSI_DGAIN'] = {'addr':80,  'size':2, 'access':'RW'} # init_value=0. 0~16383
ctl_table['ADDR_POSI_IGAIN'] = {'addr':82,  'size':2, 'access':'RW'} # init_value=0. 0~16383
ctl_table['ADDR_POSI_PGAIN'] = {'addr':84,  'size':2, 'access':'RW'} # init_value=800. 0~16383
ctl_table['ADDR_FWD_GAIN2']  = {'addr':88,  'size':2, 'access':'RW'} # init_value=0. 0~16383
ctl_table['ADDR_FWD_GAIN1']  = {'addr':90,  'size':2, 'access':'RW'} # init_value=0. 0~16383
ctl_table['ADDR_BUS_WD']     = {'addr':98,  'size':1, 'access':'RW'} # init_value=0. 0~127. 20ms unit. 
ctl_table['ADDR_GOAL_PWM']   = {'addr':100, 'size':2, 'access':'RW'} # init_value=-. -36~+36. 0.113% unit.
ctl_table['ADDR_GOAL_CURR']  = {'addr':102, 'size':2, 'access':'RW'} # init_value=-. -38~+38. 2.69mA unit
ctl_table['ADDR_GOAL_VEL']   = {'addr':104, 'size':4, 'access':'RW'} # init_value=-. -44~+44. 0.229rpm unit
ctl_table['ADDR_PROF_ACCEL'] = {'addr':108, 'size':4, 'access':'RW'} # init_value=0. 0~32767. 214.557rev/min^2 or 1ms
ctl_table['ADDR_PROF_VEL']   = {'addr':112, 'size':4, 'access':'RW'} # init_value=0. 0~32767. 0.229rpm
ctl_table['ADDR_GOAL_POSI']  = {'addr':116, 'size':4, 'access':'RW'} # init_value=-. min posi ~ max posi. 1 pulse unit.
ctl_table['ADDR_REAL_TICK']  = {'addr':120, 'size':2, 'access':'R' } # init_value=-. 0~32767. real time tick
ctl_table['ADDR_MOV_STS']    = {'addr':122, 'size':1, 'access':'R' } # init_value=0. 
ctl_table['ADDR_MOV_STS_I']  = {'addr':123, 'size':1, 'access':'R' } # init_value=0. 
ctl_table['ADDR_PRES_PWM']   = {'addr':124, 'size':2, 'access':'R' } # init_value=-. -36~+36. 0.113% unit.
ctl_table['ADDR_PRES_LOAD']  = {'addr':126, 'size':2, 'access':'R' } # init_value=-. -38~+38. 2.69mA unit. current load on 
ctl_table['ADDR_PRES_VEL']   = {'addr':128, 'size':4, 'access':'R' } # init_value=-. -44~+44. 0.229rpm unit
ctl_table['ADDR_PRES_POSI']  = {'addr':132, 'size':4, 'access':'R' } # init_value=-. -2,147,483,648 ~ 2,147,483,647 
ctl_table['ADDR_VEL_TRAJ']   = {'addr':136, 'size':4, 'access':'R' } # init_value=-. 0.229rpm unit
ctl_table['ADDR_POSI_TRAJ']  = {'addr':140, 'size':4, 'access':'R' } # init_value=-. 1 pulse unit. 
ctl_table['ADDR_PRES_INVTG'] = {'addr':144, 'size':2, 'access':'R' } # init_value=-. 0.1V unit
ctl_table['ADDR_PRES_TEMP']  = {'addr':146, 'size':1, 'access':'R' } # init_value=-. 1deg unit
ctl_table['ADDR_BACKUP_RDY'] = {'addr':147, 'size':1, 'access':'R' } # init_value=-. 0~1

if os.name == 'nt':
    import msvcrt
    def getch():
        return msvcrt.getch().decode()
else:
    import sys, tty, termios
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    def getch():
        try:
            tty.setraw(sys.stdin.fileno())
            ch = sys.stdin.read(1)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        return ch

# Enable or Disable Torque 
TORQUE_ENABLE     = 1         # Value for enabling the torque
TORQUE_DISABLE    = 0         # Value for disabling the torque

# Operating mode
OPR_MODE_CURR_CTL       = 0   # control current (torque). no control for velocity and position
OPR_MODE_VEL_CTL        = 1   # velocity control 
OPR_MODE_POSI_CTL       = 3   # position control 
OPR_MODE_MT_POSI_CTL    = 4   # multi-turn position control 
OPR_MODE_CURR_POSI_CTL  = 5   # current based position control 
OPR_MODE_PWM_CTL        = 16  # PWM direct control 

DXL_POSI_TOL = 10        # Dynamixel moving status threshold
POSI_360 = 4095
UINT_MAX = 4294967296    # 2^32
@dataclass
class joint_move_param:
    p_gain : int = 0
    i_gain : int = 0
    d_gain : int = 0
    inposition : int = 0


class mot_manipulator:
    def __init__(self, version, port, baudrate) -> None:
        self.log = jeus_log(os.getcwd(), 'mot_manipulator')
        self.protocol_version : int = version
        self.connect_port : str = port
        self.baudrate : int = baudrate
        

    def connect(self) -> bool:
        self.port_handler  = PortHandler(self.connect_port)
        self.packet_handler = PacketHandler(self.protocol_version)
        # Initialize GroupSyncWrite instance
        self.groupSyncWrite = GroupSyncWrite(self.port_handler, self.packet_handler, ctl_table['ADDR_GOAL_POSI']['addr'], ctl_table['ADDR_GOAL_POSI']['size'])

        # Initialize GroupSyncRead instace for Present Position
        self.groupSyncRead = GroupSyncRead(self.port_handler, self.packet_handler, ctl_table['ADDR_PRES_POSI']['addr'],  ctl_table['ADDR_PRES_POSI']['size'])
        # Open port
        if not self.port_handler.openPort():
            self.log.Error("Failed to open the port")
            return False
        else:
            self.log.Info("Succeeded to open the port")
    
        # Set port baudrate
        if not self.port_handler.setBaudRate(self.baudrate):
            self.log.Error("Failed to change the baudrate")
            return False
        else:
            self.log.Info("Succeeded to change the baudrate")
        return True

    # Function: set joint_parameter
    def set_module_param(self, module_param :dict[int,joint_move_param]):
        self.module_param :dict[int,joint_move_param] = module_param


    # Function: Write control table address
    def writeCtlTable(self, idx, addr_str, val) -> bool:
        if not hasattr(self, 'port_handler') or not hasattr(self, 'packet_handler'):
            if not self.connect():
                self.log.Critical('writeCtlTable Error : cannot connect')
                return
        addr = ctl_table[addr_str]['addr']
        size = ctl_table[addr_str]['size']
        if(1==size):  
            dxl_comm_result, dxl_error = self.packet_handler.write1ByteTxRx(self.port_handler, idx, addr, val)
        elif(2==size):  
            dxl_comm_result, dxl_error = self.packet_handler.write2ByteTxRx(self.port_handler, idx, addr, val)
        elif(4==size):  
            dxl_comm_result, dxl_error = self.packet_handler.write4ByteTxRx(self.port_handler, idx, addr, val)
        else:
            self.log.Error("ERR: size {0} not supported".format(size))
            return False
        if dxl_comm_result != COMM_SUCCESS:
            self.log.Error("ERR: %s" % self.packet_handler.getTxRxResult(dxl_comm_result))
            return False
            
        elif dxl_error != 0:
            self.log.Error("ERR: %s" % self.packet_handler.getRxPacketError(dxl_error))
            return False
        return True

    # Function: Read control table address
    def readCtlTable(self, idx, addr_str):
        if not hasattr(self, 'port_handler') or not hasattr(self, 'packet_handler'):
            if not self.connect():
                self.log.Error('readCtlTable Error : cannot connect')
                return
        addr = ctl_table[addr_str]['addr']
        size = ctl_table[addr_str]['size']
        if(1==size):  
            rv, dxl_comm_result, dxl_error = self.packet_handler.read1ByteTxRx(self.port_handler, idx, addr)
        elif(2==size):  
            rv, dxl_comm_result, dxl_error = self.packet_handler.read2ByteTxRx(self.port_handler, idx, addr)
        elif(4==size):  
            rv, dxl_comm_result, dxl_error = self.packet_handler.read4ByteTxRx(self.port_handler, idx, addr)
        else:
            self.log.Error("ERR: size {0} not supported".format(size))
            sys.exit(1)
        if dxl_comm_result != COMM_SUCCESS:
            self.log.Error("ERR: %s" % self.packet_handler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            self.log.Error("ERR: %s" % self.packet_handler.getRxPacketError(dxl_error))
        return rv

    # Function: Handle reboot command
    def handleReboot(self, idx):
        dxl_comm_result, dxl_error = self.packet_handler.reboot(self.port_handler, idx)
        if dxl_comm_result != COMM_SUCCESS:
            self.log.Warning("FAIL: %s" % self.packet_handler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            self.log.Error("ERR: %s" % self.packet_handler.getRxPacketError(dxl_error))
        self.log.Info("[ID:%03d] reboot Succeeded\n" % idx)


    # Function: Read present position
    def get_current_position(self, idx):
        rv = self.readCtlTable(idx, 'ADDR_PRES_POSI')
        return rv

    # Function: Read present milli ampere load
    def get_current_loadmA(self, idx):
        rv = self.readCtlTable(idx, 'ADDR_PRES_LOAD')
        if(rv>32768):
            load = rv - 65536 # negative means power is ingressing due to counter EMF
        else:
            load = rv
        loadf = int(2.69*float(load))
        return loadf


    # Function: Read present temperature
    def get_current_temp(self, idx):
        rv = self.readCtlTable(idx, 'ADDR_PRES_TEMP')
        if(rv>128):
            temp = rv - 128
        else:
            temp = rv
        return temp

    # Function: Read operating mode
    def get_operation_mode(self, idx):
        rv = self.readCtlTable(idx, 'ADDR_OPR_MODE')
        return rv

    # Function: Read present velocity
    def get_current_velo(self, idx):
        rv = self.readCtlTable(idx, 'ADDR_PRES_VEL')
        return rv

    def convert_position_byte(self, val: int):
        return [DXL_LOBYTE(DXL_LOWORD(val)), DXL_HIBYTE(DXL_LOWORD(val)), DXL_LOBYTE(DXL_HIWORD(val)), DXL_HIBYTE(DXL_HIWORD(val))]
    # Function: Write parameter
    def writeParam(self,idx, p_gain, i_gain, d_gain, f1_gain, f2_gain, prof_vel, prof_accel, debug_on):
        if(debug_on==True):
            self.log.Info("write PID gain param idx{0}".format(idx))
        self.writeCtlTable(idx, 'ADDR_POSI_PGAIN', p_gain)
        self.writeCtlTable(idx, 'ADDR_POSI_IGAIN', i_gain)
        self.writeCtlTable(idx, 'ADDR_POSI_DGAIN', d_gain)
        self.writeCtlTable(idx, 'ADDR_FWD_GAIN1', f1_gain)
        self.writeCtlTable(idx, 'ADDR_FWD_GAIN2', f2_gain)
        if(debug_on==True):
            self.log.Info("write profile param idx{0}".format(idx))
        # writeCtlTable(idx, 'ADDR_PROF_VEL', 32767) # operation is same to 0
        self.writeCtlTable(idx, 'ADDR_PROF_VEL', prof_vel)
        self.writeCtlTable(idx, 'ADDR_PROF_ACCEL', prof_accel)

    # Function: Write goal position
    def set_goal_position(self,idx, gp, debug_on):
        if(debug_on==True):
            self.log.Info("write goal position")
        self.writeCtlTable(idx, 'ADDR_GOAL_POSI', gp)
        if(debug_on==True):
            self.log.Info("PASS: write goal position")

    def torque_onoff(self, idx, IsOn) -> bool:
        addr = ctl_table['ADDR_TQ_ENB']['addr']
        torque = str()
        if IsOn:
            torque = TORQUE_ENABLE
        else:
            torque = TORQUE_DISABLE
        dxl_comm_result, dxl_error = self.packet_handler.write1ByteTxRx(self.port_handler, idx, addr, torque)
        if dxl_comm_result != COMM_SUCCESS:
            self.log.Error("%s" % self.packet_handler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            self.log.Error("%s" % self.packet_handler.getRxPacketError(dxl_error))
        else:
            self.log.Error("id:{0}, torque {1}".format(idx, IsOn))

    def move_one_joint(self, idx : int, angle : int, vel=20, monitor_on = True) -> bool:
        addr_goal_posi = ctl_table['ADDR_GOAL_POSI']['addr']
        addr_cur_posi = ctl_table['ADDR_PRES_POSI']['addr']
        goalPosi = int(POSI_360*angle/360)
        if not self.writeParam(idx, self.module_param[idx].p_gain, self.module_param[idx].i_gain, self.module_param[idx].d_gain, 0, 0, vel, 0, False):
            return False
        self.set_goal_position(idx, goalPosi, False)
        if monitor_on:
            while True:
                cur_pos = self.get_current_position(idx)
                cur_load = self.get_current_loadmA(idx)
                cur_temp = self.get_current_temp(idx)
                posi_diff = abs(goalPosi-cur_pos)
                if self.module_param[idx].inposition >= posi_diff:
                    break
        return True


    def move_multi_joint(self, idx : list[int], angle :  list[int], vel= 20, monitor_on = True) -> bool:
        addr_cur_posi = ctl_table['ADDR_PRES_POSI']['addr']
        len_cur_posi = ctl_table['ADDR_PRES_POSI']['size']
        goalPosi = np.array(angle)*POSI_360/360 
        n = len(idx)
        arrive_inposition = [False for i in range(n)]

        self.groupSyncRead.clearParam()
        self.groupSyncWrite.clearParam()
        if not type(vel) is list:
            vel = [vel for i in range(n)]
        if n != len(angle):
            self.log.Error('move_all_joint Fail : idx length error')
        for i in range(len(idx)):       
            dxl_addparam_result = self.groupSyncRead.addParam(idx[i])
            if dxl_addparam_result != True:
                self.log.Error("[ID:%03d] groupSyncRead addparam failed" % idx[i])
            if not self.writeParam(i, self.module_param[idx].p_gain, self.module_param[idx].i_gain, self.module_param[idx].d_gain, 0, 0, vel[idx[i]], 0, False):
                return False
        # Add Dynamixel#1 goal position value to the Syncwrite parameter storage
        for i in range(n):
            send_data = self.convert_position_byte(angle[i])
            dxl_addparam_result = self.groupSyncWrite.addParam(idx[i], send_data)
            if dxl_addparam_result != True:
                self.log.Error("[ID:%03d] groupSyncWrite addparam failed" % idx[i])
                return False

        # Syncwrite goal position
        dxl_comm_result = self.groupSyncWrite.txPacket()
        if dxl_comm_result != COMM_SUCCESS:
            self.log.Error("%s" % self.packet_handler.getTxRxResult(dxl_comm_result))


        if monitor_on:
            while True:
                for i in range(n):
                    dxl_getdata_result = self.groupSyncRead.isAvailable(idx[i], addr_cur_posi, len_cur_posi)
                    if dxl_getdata_result != True:
                        self.log.Error("[ID:%03d] groupSyncRead getdata failed" % idx[i])
                for i in range(n):
                    cur_pos = self.groupSyncRead.getData(idx[i], addr_cur_posi, len_cur_posi)
                    if self.module_param[idx].inposition >= abs(cur_pos-goalPosi[i]):
                        arrive_inposition[i] = True
                if not False in arrive_inposition:
                    break

        return True

    def get_multi_position(self, idx : list[int]):
        self.groupSyncRead.clearParam()
        addr_cur_posi = ctl_table['ADDR_PRES_POSI']['addr']
        len_cur_posi = ctl_table['ADDR_PRES_POSI']['size']
        cur_pos = dict()
        n = len(idx)
        for i in range(n):       
            dxl_addparam_result = self.groupSyncRead.addParam(idx[i])
            if dxl_addparam_result != True:
                self.log.Error("[ID:%03d] groupSyncRead addparam failed" % idx[i])
                return cur_pos
        for i in range(n):
            dxl_getdata_result = self.groupSyncRead.isAvailable(idx[i], addr_cur_posi, len_cur_posi)
            if dxl_getdata_result != True:
                self.log.Error("[ID:%03d] groupSyncRead getdata failed" % idx[i])
                return cur_pos
            for i in range(n):
                cur_pos[idx[i]] = self.groupSyncRead.getData(idx[i], addr_cur_posi, len_cur_posi)
        return cur_pos

        
    def disconnect(self):
        self.port_handler.closePort()
        return True
    