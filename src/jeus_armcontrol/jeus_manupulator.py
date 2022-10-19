import enum
import math
from operator import index
from threading import Timer
from .jeus_kinematictool import *
from .jeus_dynamixel import *
from .jeus_log import *
import yaml
class MoveMode(enum.Enum):
    J_Move = 0
    L_Move = 0

class jeus_maunpulator():
    
    def __init__(self,
                 grav = DEFAULT_GRAV_ACC_VEC):
        self.module : mot_manipulator
        self.device_config = device_param()
        self.index_list = []
        self.module_config = dict()
        self.log = jeus_log(os.getcwd(), 'jeus_maunpulator')
        
        # ROBOT 생성
        super().__init__()

# Create module function

    def generate_module(self):
        self.module : mot_manipulator = mot_manipulator(self.device_config)
        self.module.set_module_param(self.module_config)
        if not self.module.connect():
            self.log.Critical('can not connect module')
            return False
        
        return True

    def get_param_value(self, config_path, config_filename):
        if not os.path.exists(os.path.join(config_path, config_filename)):
            self.log.Error("get_param_value : does not exist config file")
            return False
        try:
            with open(os.path.join(config_path,config_filename)) as open_file:
                config_yaml = yaml.load(open_file, Loader= yaml.FullLoader)

                self.device_config.port = config_yaml['Device']['Port']

                self.device_config.baudrate = config_yaml['Device']['Baudrate']

                self.device_config.protocol_version = config_yaml['Device']['ProtocolVersion']

                number_of_joint = config_yaml['Joint']['Number']
                self.log.Info(number_of_joint)
                link_chain_buf=[]
                for i in range(number_of_joint):
                    s = 'Joint_%02d'%i
                    self.log.Info(s)
                    joint_yaml = config_yaml['Joint'][s]
                    theta = joint_yaml['theta']*DEG2RAD
                    d = joint_yaml['d']
                    alpha = joint_yaml['alpha']*DEG2RAD
                    a = joint_yaml['a']
                    beta = joint_yaml['beta']*DEG2RAD
                    b = joint_yaml['b']
                    joint_param = joint_move_param()
                    index : int = joint_yaml['index']

                    joint_param.p_gain = joint_yaml['p_gain']
                    joint_param.i_gain = joint_yaml['i_gain']
                    joint_param.d_gain = joint_yaml['d_gain']
                    joint_param.max_rpm = joint_yaml['max_rpm']
 
                    joint_param.max_acc = joint_yaml['max_acc']

                    joint_param.max_LoadmA = joint_yaml['max_LoadmA']

                    joint_param.min_ang = joint_yaml['min_ang']

                    joint_param.max_ang = joint_yaml['max_ang']

                    joint_param.inposition = joint_yaml['inposition']
                    joint_param.joint_offset = joint_yaml['offset_angle'] 

                    self.module_config[index] = joint_param

                    self.index_list.append(index)
                    link_chain_buf.append([theta,d,a,alpha,beta,b])

                self.LinkChain = np.array(link_chain_buf)
            return True
                
        except:
            self.log.Error("get_param_value : does not exist config fil")
            return False


# dynamixel control function
  
    def torque_on(self,  joint_num : int = -1) -> bool:
        if joint_num == -1:
            for k in self.index_list:
                rtn = self.module.torque_onoff(k,True)
                self.log.Info(f"{k} is Torque On [result] : {rtn}")
        else:
            rtn = self.module.torque_onoff(self.index_list[joint_num],True)
            self.log.Info(f"{self.index_list[joint_num]} is Torque On [result] : {rtn}")
        return rtn

    def torque_off(self,  joint_num : int = -1) -> bool:
        if joint_num == -1:
            for k in self.index_list:
                rtn = self.module.torque_onoff(k,False)
                self.log.Info(f"{k} is Torque Off [result] : {rtn}")
        else:
            rtn = self.module.torque_onoff(self.index_list[joint_num],False)
            self.log.Info(f"{self.index_list[joint_num]} is Torque Off [result] : {rtn}")
        return rtn

    def disconnect(self):
        self.module.disconnect()
        return True

    def point2Angle(self, move_mode : MoveMode, x : float, y : float, z : float, rx=-1, ry=-1, rz=-1):
        # todo - 
        joint_angle_current =np.array(self.get_current_joint_pos().values())
        joint_angle_target = self.get_ik_modf([x,y,z],joint_angle_current)
        buf = [((int(joint_angle_target[i]*RAD2DEG)-self.module_config[self.index_list[i]].joint_offset)* -1) for i in range(4)]
        self.log.Error(f'joint_angle_target {buf}')
        buf = self.adjust_offset_angle(joint_angle_target)
        self.log.Error(f'joint_angle_target {buf}')
        return joint_angle_target

    def move_point(self, move_mode:MoveMode, x : float, y : float, z : float, rx=-1, ry=-1, rz=-1):
        # todo - 
        joint_angle_current = np.array([self.get_current_joint_pos()[self.index_list[i]] for i in range(4)])*DEG2RAD
        self.log.Info(f"joint_angle_current {joint_angle_current}")
        joint_angle_target = self.get_ik_modf([x,y,z],joint_angle_current)
        self.log.Info(f"joint_angle_target {joint_angle_target}")
        ac = np.array(self.adjust_offset_angle(joint_angle_current-joint_angle_target))
        # at = np.array(self.adjust_offset_angle(joint_angle_target))
        dif = np.array([self.module.angle_to_safetyangle(ac[i]) for i in range(4)])
        diff = abs(dif)
        self.log.Info(f"diff {diff}")
        ratio = diff/max(diff)
        self.log.Info(f"ratio {ratio}")
        velo = [ratio[i]* 100 for i in range(4)]
        paths : list
        # if move_mode == MoveMode.J_Move:
        #     self.get_J_path(joint_angle_current,joint_angle_target,0,1, 0.05)

        # return
        if move_mode == MoveMode.J_Move:            
            self.move_joint_all(joint_angle_target,velo,True)
        return
    def move_point_test(self, move_mode : MoveMode, x : float, y : float, z : float):
        # TBD 
        joint_angle_current =np.array(list(self.get_current_joint_pos().items()))*DEG2RAD
        joint_angle_target = self.get_ik_modf([x,y,z],joint_angle_current)
        paths : list
        if move_mode == MoveMode.J_Move:
            self.get_J_path(joint_angle_current,joint_angle_target,0,1, 0.05)

        return

    def move_joint(self, joint_num : int , angle : float , vel = 100) -> bool:
        if not self.module.move_one_joint(self.index_list[joint_num], angle,vel):
            self.log.Error(f'Move Fail Joint {self.index_list[joint_num]} go angle {angle}')
            return False
        return True

    def move_joint_all(self,  angles : list[int] , vel = 20, useoffset = False) -> bool:        
        if useoffset :
            angle_adjust = self.adjust_offset_angle(angles)
        else:
            angle_adjust= angles
        self.log.Info(f"angle_adjust {angle_adjust}")
        if not self.module.move_multi_sync_joint(self.index_list, angle_adjust,vel):
            self.log.Error('Move Fail Joints')
            return False
        return True
    
    def get_torque_status(self,joint_num : int) -> bool:
        rtn = self.module.get_torque_status(self.index_list[joint_num])
        return rtn
    
    def get_current_joints_pos(self):
        positions = self.module.get_multi_position(self.index_list)
        return positions


    def get_current_joint_pos(self):
        positions = dict()
        for i in self.index_list:
            positions[i]=self.module.get_current_position(i)
        return positions
    
    def adjust_offset_angle(self, val : list[int]):
        buf = [(((val[i]*RAD2DEG)-self.module_config[self.index_list[i]].joint_offset)* -1) for i in range(4)]
        return buf



# Kinematic function
# 
    def get_J_path(self, qs, qd, t0 = 0.0, t1 = 1.0, dt = 0.01, vel = 100, res = None ):
        """
        Test Code 
        """
        Ux = PathPlanner(qs, qd, t0, t1)
        t = 0
        
        Q = []
        q_pre = deepcopy(qs) 
        
        Q.append(qs)
        
        dof = 4

        while(1):
            t += dt
            
            # Get Cartesian Reference =================================================
            q_tar = np.zeros(dof); qd_tar = np.zeros(dof); qdd_tar = np.zeros(dof);
            for i in range(dof):
                val, val_d, val_dd = get_state_traj(t, t0, Ux[i])
                q_tar[i] = val[1]
                qd_tar[i] =int(abs(val_d[0]))
                # qd_tar[i] =round(abs(val_d[0]),1)
                if qd_tar[i] == 0:
                    qd_tar[i] = 1 
        
            self.log.Info(f"goal posi : {q_tar}")
            self.log.Info(f"vel : {qd_tar}")
            q_cur = q_tar
            self.move_joint_all(q_cur,10,True)
            # Append Data ------------------------------------------------------------
            Q.append(q_cur)

            # recursive --------------------------------
            q_pre = deepcopy(q_cur)

            if t > t1:
                break
        return q_tar
 
 

    def get_current_xyz(self):
        joint_angle_current =np.array(list(self.get_current_joint_pos().values()))*DEG2RAD
        buf_vector = self.get_fk(joint_angle_current)
         # translation vector
        translation = buf_vector[0:3,3] 
        return translation
        
        
    def get_fk(self, q):
        """ 
        조인트들의 angle을 end effector's homogeneous transformation matrix로 바꿔주는 함수
        q = angles [4x1]
        return homogeneous transformation matrix [4x4]
        """        
        T = np.eye(4)
        for i in range(len(q)):
            T = T @ get_link_tform_six_para(self.LinkChain[i,:], q[i])
            
        return T
    
    def get_ik(self, T, curJnt = [None for i in range(4)], shoulder = FRONT, elbow = DOWN):
        """ 
        armform 은 shoulder, elbow down 으로 가정.
        이를 벗어나는 자세에 대해서는 해를 구하지 못함
        """

        CMPV = 1e-9
        
        _,   d1, a1, _, _, _ = self.LinkChain[0]
        offs, _, a2, _, _, b2 = self.LinkChain[1]
        _,    _, a3, _, _, b3 = self.LinkChain[2]
        _,   d4, a4,  _, _, _ = self.LinkChain[3]   
        
        # rotation vector
        R = T[0:3,0:3]
         # translation vector
        o = T[0:3,3]

        xc, yc, zc = o
        
        xc = 0.0 if abs(xc) <= CMPV else xc
        yc = 0.0 if abs(yc) <= CMPV else yc
        zc = 0.0 if abs(zc) <= CMPV else zc
        
        # ====================================================
        # J2 기준의 r-s 축  
        # ====================================================
        
        s = zc-d1
        s = 0.0 if np.abs(s) <= CMPV else s
        
        rc = np.sqrt(xc**2 + yc**2)
        rc = 0.0 if np.abs(rc) <= CMPV else rc
        
        if shoulder == FRONT:
            r =  rc - a1 #! dir-x 
        else:
            r = -rc - a1 #! dir-x
        r = 0.0 if np.abs(r) <= CMPV else r
        
        # ====================================================
        # Link Length
        # ====================================================
        L2 = np.sqrt(a2**2 + b2**2)
        L3 = np.sqrt(a3**2 + b3**2)
        L4 = a4

        # ====================================================
        # th1
        # ====================================================
        if (np.abs(xc) < CMPV and np.abs(yc) < CMPV):
            th1 = curJnt[0]
        else:
            if shoulder == FRONT:
                th1 = np.arctan2(yc, xc) # range : -180 < th1 < 180
            else:
                th1 = np.arctan2(-yc, -xc)

            th1 = 0.0 if np.abs(th1) <= CMPV else th1
        
        # ====================================================
        # phi
        # ====================================================
        # 수정 필요
        R1 = get_link_tform_six_para(self.LinkChain[0], th1)[0:3,0:3]
        R_temp = R1.T @ R
        
        R_temp[2,0] = 0
        R_temp[2,1] = 0
        R_temp[0,2] = 0
        R_temp[2,2] = 0
        R_temp[2,2] = 1
        
        _, _, rz = tr2eulXYZ(R_temp)
        
        phi = -rz

        # ====================================================
        # th3
        # ====================================================
        x3 = r - L4*cos(phi)
        y3 = s - L4*sin(phi)
        
        Dc = (x3**2 + y3**2 - L2**2 - L3**2 ) / (2 * L2 * L3)
        
        if np.abs(Dc + TINY) > 1.0:
            self.log.Error("CANNOT REACH THAT POSITION")
        
        th3 = np.arccos(Dc)
        
        # if elbow down 
        th3 = -th3
        
        # ====================================================
        # th2
        # ====================================================    
        s3 = sin(th3)
        c3 = cos(th3)

        k1 = L2 + L3 * c3
        k2 = L3 * s3

        th2 = np.arctan2(y3, x3) - np.arctan2(k2, k1) # 왜 - 붙어야 답이지?

        # ====================================================
        # th4
        # ====================================================  
        th4 = phi - th2 - th3

        # ====================================================
        # offset 
        # ====================================================  
        
        sigma2 = np.arctan2(np.abs(b2), np.abs(a2))
        th2_prime = pi/2 - sigma2
        q2 = -(th2 - th2_prime)

        sigma3 = np.arctan2(np.abs(b3), np.abs(a3))
        th3_prime = th2_prime - sigma3
        
        q3 = -(-th3 - th3_prime)
        q3 = (-th3 - th3_prime)
        
        q1 = th1

        th4_prime = sigma3
        q4 = (-th4-th4_prime)
        
        return np.array([q1, q2, q3, q4])

    def get_ik_modf(self, p, curJnt = [None for i in range(4)], shoulder = FRONT, elbow = DOWN):
        """ 
        armform 의 elbow 는 down 으로 가정
        이를 벗어나는 자세에 대해서는 해를 구하지 못함
        
        armform 조합에 대한 검증 필요
        
        """

        CMPV = 1e-9
        
        _,   d1, a1, _, _, _ = self.LinkChain[0]
        offs, _, a2, _, _, b2 = self.LinkChain[1]
        _,    _, a3, _, _, b3 = self.LinkChain[2]
        _,   d4, a4,  _, _, _ = self.LinkChain[3]   
        
        
        xc, yc, zc = p
        xc = 0.0 if abs(xc) <= CMPV else xc
        yc = 0.0 if abs(yc) <= CMPV else yc
        zc = 0.0 if abs(zc) <= CMPV else zc
        
        # ====================================================
        # J2 기준의 r-s 축  
        # ====================================================
        
        s = zc-d1
        s = 0.0 if np.abs(s) <= CMPV else s
        
        rc = np.sqrt(xc**2 + yc**2)
        rc = 0.0 if np.abs(rc) <= CMPV else rc
        
        if shoulder == FRONT:
            r =  rc - a1 #! dir-x 
        else:
            r = -rc - a1 #! dir-x
        
        r = 0.0 if np.abs(r) <= CMPV else r
        
        # ====================================================
        # Link Length
        # ====================================================
        L2 = np.sqrt(a2**2 + b2**2)
        L3 = np.sqrt(a3**2 + b3**2)
        L4 = a4

        # ====================================================
        # th1
        # ====================================================
        if (np.abs(xc) < CMPV and np.abs(yc) < CMPV):
            th1 = curJnt[0]
        else:
            if shoulder == FRONT:
                th1 = np.arctan2(yc, xc) # range : -180 < th1 < 180
            else:
                th1 = np.arctan2(-yc, -xc)

            th1 = 0.0 if np.abs(th1) <= CMPV else th1
        
        # ====================================================
        # phi
        # ====================================================
        phi = 0.0 # for constraint
        
        # ====================================================
        # th3
        # ====================================================
        x3 = r - L4*cos(phi)
        y3 = s - L4*sin(phi)
        
        Dc = (x3**2 + y3**2 - L2**2 - L3**2 ) / (2 * L2 * L3)
        
        if np.abs(Dc + TINY) > 1.0:
            print("CANNOT REACH THAT POSITION")
        
        th3 = np.arccos(Dc)
        
        # if elbow down 
        th3 = -th3
        
        # ====================================================
        # th2
        # ====================================================    
        s3 = sin(th3)
        c3 = cos(th3)

        k1 = L2 + L3 * c3
        k2 = L3 * s3

        th2 = np.arctan2(y3, x3) - np.arctan2(k2, k1) # 왜 - 붙어야 답이지?

        # ====================================================
        # th4
        # ====================================================  
        th4 = phi - th2 - th3

        # ====================================================
        # offset 
        # ====================================================  
        
        sigma2 = np.arctan2(np.abs(b2), np.abs(a2))
        th2_prime = pi/2 - sigma2
        q2 = -(th2 - th2_prime)

        sigma3 = np.arctan2(np.abs(b3), np.abs(a3))
        th3_prime = th2_prime - sigma3
        
        q3 = -(-th3 - th3_prime)
        q3 = (-th3 - th3_prime)
        
        q1 = th1

        th4_prime = sigma3
        q4 = (-th4-th4_prime)
        
        return np.array([q1, q2, q3, q4])

