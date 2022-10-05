from jeus_armcontrol.jeus_armcontrol import *

class ROBOT():
    
    def __init__(self,
                 grav = DEFAULT_GRAV_ACC_VEC):

        # DEFAULT LINK CHAIN ===========================================
        self.LinkChain = np.array([[       0,    76.5,     0,  -np.pi/2, 0,     0],
                                  [-np.pi/2,       0,   530,         0, 0,  24.0],
                                  [ np.pi/2,       0,   474,         0, 0, -24.0],
                                  [       0,       0,    96,         0, 0,     0]])
  
        # ROBOT 생성
        super().__init__()
        
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