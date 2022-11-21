from collections import deque
import enum
import math
from operator import index
from threading import Timer, Thread, Lock
from .jeus_kinematictool import *
from .jeus_dynamixel import *
from .jeus_log import *
import yaml
class MoveMode(enum.Enum):
    J_Move = 0
    L_Move = 1

class cmd_from_main(enum.Enum):
    Connect_MC = 0
    Move_Point = 1
    Move_Joint = 2
    Move_Joints = 3
    Torque_OnOff = 4



class jeus_maunpulator_test():
    
    def __init__(self,
                 grav = DEFAULT_GRAV_ACC_VEC):
        self.module : mot_manipulator
        self.device_config = device_param()
        self.index_list = []
        self.module_config = dict()
        self.log = jeus_log(os.getcwd(), 'jeus_maunpulator')
        self.is_finish = False
        self.que = deque()
        self.main_thread = Thread(target = self.main_func)
        self.pos_lock = Lock()
        self.torque_lock =Lock()
        self.torque = dict()
        # ROBOT 생성
        super().__init__()

    

    def main_func(self):
        while self.is_finish == False:
            if len(self.que) > 0 :
                buf = self.que.popleft()
                if buf[0] == cmd_from_main.Move_Point:
                    """
                    buf ={cmd_from_main.Move_Point, MoveModes, x,y,z}
                    """
                    mode : MoveMode = buf[1]
                    x : float = buf[2]
                    y : float = buf[3]
                    z : float = buf[4]
                    self.move_point(mode, x,y,z)
                elif buf[0] == cmd_from_main.Move_Joint:
                    """
                    buf ={cmd_from_main.Move_Joint, joint_index, angles}
                    """
                    joint_index = buf[1]
                    angle = buf[2]
                    self.move_joint(joint_index,angle)
                
                elif buf[0] == cmd_from_main.Move_Joints:
                    """
                    buf ={cmd_from_main.Move_Joints, joint_index, angles, use_offset}
                    """

                    joint_index = buf[1]
                    angles = buf[2]
                    use_offset = buf[3]
                    self.move_joint_all(joint_index,angles,use_offset)
                elif buf[0] == cmd_from_main.Torque_OnOff:
                    """
                    
                    """
                    joint_index = buf[1]
                    turn_on = buf[2]
                    if turn_on :
                        self.torque_on(joint_index)
                    else:
                        self.torque_off(joint_index)
            if hasattr(self, 'module'):
                self.update_pos()
                self.update_torque()
            time.sleep(0.01)

    def get_pos(self):
        if self.pos_lock.acquire(timeout=0.05):                    
            self.return_pos = self.current_pos
            self.pos_lock.release()
            return self.return_pos
        else:
            return self.current_pos
    
    def get_torque_status(self,joint_num : int) -> dict:
        if self.torque_lock.acquire(timeout=0.05):
            rtn = self.torque[self.index_list[joint_num]]
            self.torque_lock.release()
            return rtn

    def update_pos(self):
        if self.pos_lock.acquire():                    
            self.current_pos = self.get_current_joint_pos()
            self.pos_lock.release()
    def update_torque(self):
        if self.torque_lock.acquire():                    
            for i in self.index_list:
                self.torque[i] =  self.module.get_torque_status(i)
            self.torque_lock.release()

# Create module function

    def generate_module(self):
        self.module : mot_manipulator = mot_manipulator(self.device_config)
        self.module.set_module_param(self.module_config)

        if not self.module.connect():
            self.log.Critical('can not connect module')
            return False
        self.update_pos()
        self.get_pos()
        self.main_thread.start()
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
                    theta = joint_yaml['theta']*DEG2RAD # Radian
                    d = joint_yaml['d'] # mm 
                    alpha = joint_yaml['alpha']*DEG2RAD # Radian
                    a = joint_yaml['a'] # mm
                    beta = joint_yaml['beta']*DEG2RAD # Radian
                    b = joint_yaml['b'] # mm
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
# public Function

    def Torque_ON(self, joint_num : int = -1):
        self.que.append([cmd_from_main.Torque_OnOff, joint_num, True])

    def Torque_OFF(self, joint_num : int = -1):
        self.que.append([cmd_from_main.Torque_OnOff, joint_num, False])

    def MoveJoint(self, joint_num , angle):
        self.que.append([cmd_from_main.Move_Joint, joint_num, angle])

    def MoveJoints(self, joint_nums, angles ,use_offset):
        self.que.append([cmd_from_main.Move_Joints, joint_nums, angles,use_offset])

    def MovePoint(self, move_mode : MoveMode, x,y,z):
        self.que.append([cmd_from_main.Move_Point, move_mode, x,y,z])
    
        
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

    def move_point(self, move_mode:MoveMode, x : float, y : float, z : float, rx=-1, ry=-1, rz=-1):
        # todo - 
        joint_angle_current = np.array([self.get_current_joint_pos()[self.index_list[i]] for i in range(4)])*DEG2RAD
        joint_angle_target = self.get_ik_modf([x,y,z],joint_angle_current)
        self.log.Info(f"joint_angle_current {joint_angle_current} -> joint_angle_target {joint_angle_target}")

        paths : list
        # if move_mode == MoveMode.J_Move:
        #     self.get_J_path(joint_angle_current,joint_angle_target,0,1, 0.05)
        
        # return
        if move_mode == MoveMode.J_Move:            
            self.move_joint_all(angles=joint_angle_target,vel= 50,useoffset = True)
        elif move_mode == MoveMode.L_Move:
            self.get_L_Path(joint_angle_current,joint_angle_target, t1 = 5)
        return
    

    def move_joint(self, joint_num : int , angle : float , vel = 100) -> bool:
        idx = self.index_list[joint_num]
        goal_angle = angle
        goal_vel = vel
        isdone = False
        start_time = datetime.now()
        while isdone==False:
            self.module.move_one_joint(idx, angle,vel)
            time.sleep(0.01)
            self.update_pos()
            current_angle = self.current_pos[idx]
            if abs(current_angle - goal_angle) <= 0.3:
                isdone = True
            duringtime = datetime.now() - start_time
            if duringtime.seconds > 5:
                isdone =True
            time.sleep(0.01)
   
    # def move_joint(self, joint_num : int , angle : float , vel = 100) -> bool:

    #     if not self.module.move_one_joint(self.index_list[joint_num], angle,vel):
    #         self.log.Error(f'Move Fail Joint {self.index_list[joint_num]} go angle {angle}')
    #         return False
    #     return True

    def move_joint_all(self,  angles : list[int] , vel = 100, useoffset = False, timeout = 6) -> bool:        
        if useoffset :
            angle_adjust = self.adjust_offset_angle(angles)
        else:
            angle_adjust= angles
        self.log.Info(f"angle_adjust {angle_adjust}")
        start_time = datetime.now()
        idxes = self.index_list
        goal_angle = angles
        isdone = False
        while isdone==False:
            self.module.move_multi_sync_joint(idxes, angle_adjust,vel)
            time.sleep(0.01)
            self.update_pos()
            isdone = True
            current_angle = self.current_pos
            i = 0
            for idx in idxes:
                if abs(current_angle[idx] -  goal_angle[i]) <= 0.3:
                    isdone &= True
                else:
                    isdone &= False
                i +=1

            duringtime = datetime.now() - start_time
            if duringtime.seconds > timeout:
                isdone =True
            time.sleep(0.01)
    
    # def move_joint_all(self,  angles : list[int] , vel = 20, useoffset = False) -> bool:        
    #     if useoffset :
    #         angle_adjust = self.adjust_offset_angle(angles)
    #     else:
    #         angle_adjust= angles
    #     self.log.Info(f"angle_adjust {angle_adjust}")
    #     if not self.module.move_multi_sync_joint(self.index_list, angle_adjust,vel):
    #         self.log.Error('Move Fail Joints')
    #         return False
    #     return True
    

    
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
    def get_J_path(self, qs, qd, t0 = 0.0, t1 = 1.0, dt = 0.01, vel = 20, res = None ):
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
                q_tar[i] = val
                qd_tar[i] = abs(val_d)
                # qd_tar[i] =round(abs(val_d[0]),1)
        
            self.log.Info(f"goal posi : {q_tar}")
            self.log.Info(f"vel : {qd_tar}")
            q_cur = q_tar
            self.move_joint_all(q_cur, qd_tar)
            # Append Data ------------------------------------------------------------
            Q.append(q_cur)

            # recursive --------------------------------
            q_pre = deepcopy(q_cur)

            if t > t1:
                break
        return q_tar

    def get_L_Path(self, qs, xd, t0 = 0.0, t1 = 1.0, dt=0.001, vel = 100):
        """
        테스트 안해봄
        """
        xs = tr2pose(self.get_fk(qs))
        
        Td = pose2tr(xd)
        qd = self.get_ik(Td, curJnt = qs, shoulder = FRONT, elbow = DOWN)

        Ux = PathPlanner(xs, xd, t0, t1)
        
        x_pre = deepcopy(xs)
        q_pre = deepcopy(qs) 

        qd_pre = np.zeros(4)
        qdd_pre = np.zeros(4)
        
        qd_cur = np.zeros(4)
        qdd_cur = np.zeros(4)

        t = 0.0
        
        Q = []
        
        while(1):
            t += dt
            
            # Get Cartesian Reference =================================================
            x_tar = np.zeros(4); xd_tar = np.zeros(4); xdd_tar = np.zeros(4);
            for i in range(4):
                val, val_d, val_dd = get_state_traj(t, t0, Ux[i])
                x_tar[i] = val; xd_tar[i] = val_d; xdd_tar[i] = val_dd
                
            # q_cur = get_dq(q_pre, x_tar)
            q_cur = self.get_ik(pose2tr(x_tar), curJnt = qs, shoulder = FRONT, elbow = DOWN)
            x_cur = tr2pose(self.get_fk(q_cur))

            # get torque ------------------------------------------------------------------
            # calculate qd, qdd
            qdd_cur = (qd_cur - qd_pre) / dt
            qd_cur = (q_cur - q_pre) / dt

            R = eulXYZ2r(x_cur[3:])

            # Append Data ------------------------------------------------------------
            Q.append(q_cur)
            
            # recursive --------------------------------
            q_pre = deepcopy(q_cur)
            x_pre = deepcopy(x_cur)

            qd_pre = deepcopy(qd_cur)
            qdd_pre = deepcopy(qdd_cur)


            # termination -------------------------
            err = np.linalg.norm(xd - x_cur)
            if t > t1:
                print('over t : ', t)
                break
            
        print("Done !")

        return np.array(Q)
 
    # def get_L_Path(self, qs, xd, t0 = 0.0, t1 = 1.0, dt=0.001, vel = 100):
    #     """
    #     테스트 안해봄
    #     """
    #     xs = tr2pose(self.get_fk(qs))
        
    #     Td = pose2tr(xd)
    #     qd = self.get_ik(Td, curJnt = qs, shoulder = FRONT, elbow = DOWN)

    #     Ux = PathPlanner(xs, xd, t0, t1)
        
    #     x_pre = deepcopy(xs)
    #     q_pre = deepcopy(qs) 

    #     qd_pre = np.zeros(4)
    #     qdd_pre = np.zeros(4)
        
    #     qd_cur = np.zeros(4)
    #     qdd_cur = np.zeros(4)

    #     t = 0.0
        
    #     Q = []
        
    #     while(1):
    #         t += dt
            
    #         # Get Cartesian Reference =================================================
    #         x_tar = np.zeros(6); xd_tar = np.zeros(6); xdd_tar = np.zeros(6);
    #         for i in range(6):
    #             val, val_d, val_dd = get_state_traj(t, t0, Ux[i])
    #             x_tar[i] = val; xd_tar[i] = val_d; xdd_tar[i] = val_dd
                
    #         # q_cur = get_dq(q_pre, x_tar)
    #         q_cur = self.get_ik(pose2tr(x_tar), curJnt = qs, shoulder = FRONT, elbow = DOWN)
    #         x_cur = tr2pose(self.get_fk(q_cur))

    #         # get torque ------------------------------------------------------------------
    #         # calculate qd, qdd
    #         qdd_cur = (qd_cur - qd_pre) / dt
    #         qd_cur = (q_cur - q_pre) / dt

    #         R = eulXYZ2r(x_cur[3:])

    #         # Append Data ------------------------------------------------------------
    #         Q.append(q_cur)
            
    #         # recursive --------------------------------
    #         q_pre = deepcopy(q_cur)
    #         x_pre = deepcopy(x_cur)

    #         qd_pre = deepcopy(qd_cur)
    #         qdd_pre = deepcopy(qdd_cur)


    #         # termination -------------------------
    #         err = np.linalg.norm(xd - x_cur)
    #         if t > t1:
    #             print('over t : ', t)
    #             break
            
    #     print("Done !")

    #     return np.array(Q)
 

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
