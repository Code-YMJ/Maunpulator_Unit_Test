from .jeus_kinetictool import *
from .jeus_dynamixel import *
from .jeus_log import *
import yaml
class jeus_maunpulator():
    
    def __init__(self,
                 grav = DEFAULT_GRAV_ACC_VEC):
        self.module : mot_manipulator
        self.device_config = device_param()
        self.index_list = list()
        self.module_config = dict()  #ket :joint index //  value : joint parameter
        # DEFAULT LINK CHAIN =========================================== theta, d, a, alpha, beta, b 
        # self.LinkChain = np.array([[       0,    76.5,     0,  -np.pi/2, 0,     0],
        #                            [-np.pi/2,       0,   530,         0, 0,  24.0],
        #                            [ np.pi/2,       0,   474,         0, 0, -24.0],
        #                            [       0,       0,    96,         0, 0,     0]])
        self.log = jeus_log(os.getcwd(), 'jeus_maunpulator')
        # ROBOT 생성
        super().__init__()

    def generate_module(self):
        self.module : mot_manipulator = mot_manipulator(self.device_config)
        self.module.set_module_param(self.module_config)
        if not self.module.connect():
            self.log.Critical('can not connect module')
            return False
        return True
    
    def move_point(self,x,y,z,rx,ry,rz):
        return


    def move_joint(self, joint_num : int , angle : int) -> bool:
        if not self.module.move_one_joint(self.index_list[joint_num], angle):
            self.log.Error('Move Fail Joint %d %d', self.index_list[joint_num], angle)
            return False
        return True
        
    def move_joint_all(self, joint_num : list[int] , angles : list[int]) -> bool:
        idx_list = list()
        for i in joint_num:
            idx_list.append(i)
        if not self.module.move_multi_joint(idx_list, angles):
            self.log.Error('Move Fail Joints')
            return False
        return True

    def get_param_value(self, config_path, config_filename):
        if not os.path.exists(config_path+config_filename):
            self.log.Error("get_param_value : does not exist config file")
            return False
        try:
            with open(config_path+config_filename) as open_file:
                config_yaml = yaml.load(open_file, Loader= yaml.FullLoader)
                self.device_config.port = config_yaml['Device']['Port']
                self.device_config.baudrate = config_yaml['Device']['Baudrate']
                self.device_config.protocol_version = config_yaml['Device']['ProtocolVersion']
                number_of_joint = config_yaml['Joint']['Number']
                link_chain_buf=[]
                for i in range(number_of_joint):
                    joint_yaml = config_yaml['Joint']['Joint_%02d'%i]
                    theta = joint_yaml['theta']
                    d = joint_yaml['d']
                    alpha = joint_yaml['alpha']
                    a = joint_yaml['a']
                    beta = joint_yaml['beta']
                    b = joint_yaml['b']
                    joint_param = joint_move_param()
                    index = joint_yaml['index']
                    joint_param.max_rpm = joint_yaml['max_rpm']
                    joint_param.max_acc = joint_yaml['max_acc']
                    joint_param.max_LoadmA = joint_yaml['max_LoadmA']
                    joint_param.min_ang = joint_yaml['min_ang']
                    joint_param.max_ang = joint_yaml['max_ang']
                    joint_param.inposition = joint_yaml['inposition']
                    self.module_config[index] = joint_param
                    self.index_list[index]
                    link_chain_buf.append([theta,d,a,alpha,beta,b])

                self.LinkChain = np.array(link_chain_buf)
            return True
                
        except:
            self.log.Error("get_param_value : does not exist config fil")
            return False
        
    def get_fk(self, q):
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
        
        
        R = T[0:3,0:3] # rotation 
        o = T[0:3,3] # translation

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



a = jeus_maunpulator()
if not a.get_param_value():
    exit(1)
