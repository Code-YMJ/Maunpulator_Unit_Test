import sys
import warnings
# import pywinauto

from PySide6.QtGui import (QBrush, QColor, QConicalGradient, QCursor,
    QFont, QFontDatabase, QGradient, QIcon,
    QImage, QKeySequence, QLinearGradient, QPainter,
    QPalette, QPixmap, QRadialGradient, QTransform)
from PySide6.QtWidgets import (QApplication, QComboBox, QGroupBox, QLabel,
    QMainWindow, QPlainTextEdit, QPushButton, QSizePolicy,
    QWidget)
from PySide6.QtCore import *
import os
from dataclasses import dataclass
import threading
from jeus_armcontrol import  jeus_manupulator_refactory,jeus_kinematictool
from jeus_vision.jeus_vision import *

import yaml
from jeus_ui.ui_jr_main_form import *



config_init_posi = 'init_position'
config_safety_posi = 'safety_position'
config_wait_posi = 'wait_position'
config_target_posi = 'target_position'
config_transform = 'transform'
x = 'x'
y = 'y'
z = 'z'
ry = 'ry'
joint_0 = 'joint_0'
joint_1 = 'joint_1'
joint_2 = 'joint_2'
joint_3 = 'joint_3'

class worker(QThread):
    def __init__(self,parent,  func, *args):
        super().__init__(parent)
        self.func = func
        self.val = args
    def run(self):
        self.func(self.val[0],self.val[1])


"""
x,y,z 이동후 rx 회전, rz 회전 !!
기준 좌표계 기준으로한 회전
"""
@dataclass
class transform_param():
    x : int = 0
    y : int = 0
    z : int = 0
    rx : int = 0
    ry : int = 0
    rz : int = 0

targe_list = [
    "person",
    "btn_1",
    "btn_2",
    "btn_3",
    "btn_4",
    "btn_5",
    "btn_6",
    "btn_7",
    "btn_8",
    "btn_9",
    "btn_10",
    "btn_11",
    "btn_12",
    "btn_13",
    "btn_14",
    "btn_15",
    "btn_16",
    "btn_17",
    "btn_18",
    "btn_19"
    ]

class MainWindow(QMainWindow, Ui_MainWindow):
    def __init__(self):
        super().__init__()
        self.setupUi(self)
        self.show()
        self.setconfig()
        self.connect_vision_module()
        # self.timer =  QTimer(self)
        self.timer = QTimer(self)
        # self.timer.singleShot(100,self.get_current_positions)
        self.timer.start(100)
        # self.
        self.timer.timeout.connect(self.get_current_positions)
        self.is_stream = False

    def connect_vision_module(self):
        # self.weight = 'btn_221203/best.pt'
        self.vision = jeus_vision()
        self.vision.init_camera()
        self.vision.init_yolo(self.weight)


    def connect_device(self):
        if not hasattr(self, 'IsConnect') or not self.IsConnect  :
            self.btnConnect.setText('connectting')
            self.manupulator = jeus_manupulator_refactory.jeus_maunpulator_test()
            self.manupulator.get_param_value(os.path.join(os.getcwd(),'Config'),'arm_config.yaml')
            if self.manupulator.generate_module():
                self.IsConnect = True
                self.btnConnect.setText('connected')
                self.manupulator.Torque_ON()
        else:
            self.IsConnect = False
            self.manupulator.disconnect()
            self.btnConnect.setText('disconnected')


    def setconfig(self):
        path =os.path.join(os.getcwd(),'Config','model_config.yaml')
        with open(path) as fileopen:
            model_config = yaml.load(fileopen, Loader=yaml.FullLoader)
            self.weight = model_config['weight_path']
            init_pos = model_config[config_init_posi]
            self.tbInitPos_0.setPlainText(str(init_pos[joint_0]))
            self.tbInitPos_1.setPlainText(str(init_pos[joint_1]))
            self.tbInitPos_2.setPlainText(str(init_pos[joint_2]))
            self.tbInitPos_3.setPlainText(str(init_pos[joint_3]))

            safety_pos = model_config[config_safety_posi]
            self.tbSafetyPos_0.setPlainText(str(safety_pos[joint_0]))
            self.tbSafetyPos_1.setPlainText(str(safety_pos[joint_1]))
            self.tbSafetyPos_2.setPlainText(str(safety_pos[joint_2]))
            self.tbSafetyPos_3.setPlainText(str(safety_pos[joint_3]))

            wait_pos = model_config[config_wait_posi]
            self.tbWaitPos_X.setPlainText(str(wait_pos[x]))
            self.tbWaitPos_Y.setPlainText(str(wait_pos[y]))
            self.tbWaitPos_Z.setPlainText(str(wait_pos[z]))

            target_pos = model_config[config_target_posi]
            self.tbTargetPos_X.setPlainText(str(target_pos[x]))
            self.tbTargetPos_Y.setPlainText(str(target_pos[y]))
            self.tbTargetPos_Z.setPlainText(str(target_pos[z]))

            transforms_config = model_config[config_transform]
            self.transform_config = transform_param()
            self.transform_config.x = transforms_config[x]
            self.transform_config.y = transforms_config[y]
            self.transform_config.z = transforms_config[z]
            self.transform_config.rx = transforms_config['rx']
            self.transform_config.ry = transforms_config['ry']
            self.transform_config.rz = transforms_config['rz']
            self.t = np.array([self.transform_config.x,self.transform_config.y,self.transform_config.z])
            self.eul = np.array([self.transform_config.rx*jeus_kinematictool.DEG2RAD,self.transform_config.ry*jeus_kinematictool.DEG2RAD,self.transform_config.rz*jeus_kinematictool.DEG2RAD])

    def save(self):
        path =os.path.join(os.getcwd(),'Config','model_config.yaml')
        data=\
            { config_init_posi:{joint_0:float(self.tbInitPos_0.toPlainText()),joint_1: float(self.tbInitPos_1.toPlainText()), joint_2: float(self.tbInitPos_2.toPlainText()), joint_3: float(self.tbInitPos_3.toPlainText())}  \
            , config_safety_posi:{joint_0:float(self.tbSafetyPos_0.toPlainText()),joint_1: float(self.tbSafetyPos_1.toPlainText()), joint_2: float(self.tbSafetyPos_2.toPlainText()), joint_3: float(self.tbSafetyPos_3.toPlainText())}  \
            , config_wait_posi:{x:float(self.tbWaitPos_X.toPlainText()), y:float(self.tbWaitPos_Y.toPlainText()), 'z': float(self.tbWaitPos_Z.toPlainText())}   \
            , config_transform:{x:self.transform_config.x, y:self.transform_config.y, 'z': self.transform_config.z, 'rx': self.transform_config.rx, 'ry': self.transform_config.ry, 'rz': self.transform_config.rz}   \
             , config_target_posi:{'x':float(self.tbTargetPos_X.toPlainText()), 'y':float(self.tbTargetPos_Y.toPlainText()), 'z': float(self.tbTargetPos_Z.toPlainText()) }}
        with open(path,'w') as fileopen:
            yaml.dump(data,fileopen,default_flow_style=False)

        
    def get_current_positions(self):
        if hasattr(self, 'manupulator') and self.IsConnect:
            positions = self.manupulator.get_pos()
            if positions != None:
                sorted_keys = sorted(positions.keys())

                self.tbCurrentPosJoint1.setPlainText(str(round(positions[sorted_keys[0]], 3)))
                self.tbCurrentPosJoint2.setPlainText(str(round(positions[sorted_keys[1]], 3)))
                self.tbCurrentPosJoint3.setPlainText(str(round(positions[sorted_keys[2]], 3)))
                self.tbCurrentPosJoint4.setPlainText(str(round(positions[sorted_keys[3]], 3)))


    def torque_onoff(self):
        sender = self.sender()        
        sender_idx = int(sender.objectName().split('_')[1])
        if self.manupulator.get_torque_status(sender_idx):
            self.manupulator.Torque_OFF(sender_idx)
        else:
            self.manupulator.Torque_ON(sender_idx)

    def move_joint(self):
        sender = self.sender()        
        sender_idx = int(sender.objectName().split('_')[1])
        angle :float = 0.0
        if sender_idx == 0:
            angle = float(self.tbTargetPosJoint_0.toPlainText())
        elif sender_idx == 1:
            angle = float(self.tbTargetPosJoint_1.toPlainText())
        elif sender_idx == 2:
            angle = float(self.tbTargetPosJoint_2.toPlainText())
        elif sender_idx == 3:
            angle = float(self.tbTargetPosJoint_3.toPlainText())
        else:
            print(f'move_joint error : index {sender_idx}')
            return
        if not self.manupulator.get_torque_status(sender_idx):
            print(f'Error : Joint {int(sender_idx)+1} torque off')
            return
        self.manupulator.MoveJoint(sender_idx,angle)

    def calculate_xyz(self):
        try:
            pc = np.array([float(self.tbCurrent_X.toPlainText()),float(self.tbCurrent_Y.toPlainText()),float(self.tbCurrent_Z.toPlainText())])
            cal_pc =jeus_kinematictool.point_to_base(pc,self.t,self.eul)
            x = round(cal_pc[0],3)
            y = round(cal_pc[1],3)
            z = round(cal_pc[2],3)
            self.tbWaitPos_X.setPlainText(str(x-20))
            self.tbWaitPos_Y.setPlainText(str(y))
            self.tbWaitPos_Z.setPlainText(str(z))

            
            self.tbTargetPos_X.setPlainText(str(x))
            self.tbTargetPos_Y.setPlainText(str(y))
            self.tbTargetPos_Z.setPlainText(str(z))
        except:
            pass


    def move_J(self):
        sender = self.sender()          
        sender_name = sender.objectName()
        if sender_name == self.btnMoveInitPos.objectName():
            joint_0 = float(self.tbInitPos_0.toPlainText())
            joint_1 = float(self.tbInitPos_1.toPlainText())
            joint_2 = float(self.tbInitPos_2.toPlainText())
            joint_3 = float(self.tbInitPos_3.toPlainText())
            angles = [joint_0,joint_1,joint_2,joint_3]
            self.manupulator.MoveJoints(angles ,100, False)

        elif sender_name == self.btnMoveSafetyPos.objectName():
            joint_0 = float(self.tbSafetyPos_0.toPlainText())
            joint_1 = float(self.tbSafetyPos_1.toPlainText())
            joint_2 = float(self.tbSafetyPos_2.toPlainText())
            joint_3 = float(self.tbSafetyPos_3.toPlainText())
            angles = [joint_0,joint_1,joint_2,joint_3]
            self.manupulator.MoveJoints( angles ,100, False)
        
        elif sender_name == self.btnMoveWaitPos.objectName():
            x = float(self.tbWaitPos_X.toPlainText())
            y = float(self.tbWaitPos_Y.toPlainText())
            z = float(self.tbWaitPos_Z.toPlainText())
            self.manupulator.MovePoint(jeus_manupulator_refactory.MoveMode.J_Move, x, y, z)
            # joint_angles = self.manupulator.point2Angle(MoveMode.J_Move, x, y, z)

        elif sender_name == self.btnMoveTargetPos_J.objectName():
            x = float(self.tbTargetPos_X.toPlainText())
            y = float(self.tbTargetPos_Y.toPlainText())
            z = float(self.tbTargetPos_Z.toPlainText())
            self.manupulator.MovePoint(jeus_manupulator_refactory.MoveMode.J_Move, x, y, z)
        else:
            return

    def move_L(self):
        x = float(self.tbTargetPos_X.toPlainText())
        y = float(self.tbTargetPos_Y.toPlainText())
        z = float(self.tbTargetPos_Z.toPlainText())
        self.manupulator.MovePoint(jeus_manupulator_refactory.MoveMode.L_Move, x, y, z)

    def sequence_DetectBtn(self):
        target = self.tbFloor.toPlainText()
        if not target in targe_list:
            self.tbCurrent_X.setPlainText('Target None')
            self.tbCurrent_Y.setPlainText('Target None')
            self.tbCurrent_Z.setPlainText('Target None')
            return
        
        result = self.vision.get_Point(target=target)
        if type(result) is not str:
            camera_x, camera_y, camera_z = result
            self.tbCurrent_X.setPlainText(str(round(camera_x*1000,3)))
            self.tbCurrent_Y.setPlainText(str(round(camera_y*1000,3)))
            self.tbCurrent_Z.setPlainText(str(round(camera_z*1000,3)))
            self.calculate_xyz()
        else:
            self.tbCurrent_X.setPlainText(result)
            self.tbCurrent_Y.setPlainText(result)
            self.tbCurrent_Z.setPlainText(result)
        

    def sequence_PushBtn(self):
        safety_joint_0 = float(self.tbSafetyPos_0.toPlainText())
        safety_joint_1 = float(self.tbSafetyPos_1.toPlainText())
        safety_joint_2 = float(self.tbSafetyPos_2.toPlainText())
        safety_joint_3 = float(self.tbSafetyPos_3.toPlainText())
        safety_angles = [safety_joint_0,safety_joint_1,safety_joint_2,safety_joint_3]
        self.manupulator.MoveJoints(safety_angles ,100, False)
        
        wait_x = float(self.tbWaitPos_X.toPlainText())
        wait_y = float(self.tbWaitPos_Y.toPlainText())
        wait_z = float(self.tbWaitPos_Z.toPlainText())
        self.manupulator.MovePoint(jeus_manupulator_refactory.MoveMode.J_Move, wait_x, wait_y, wait_z)
        
        
        target_x = float(self.tbTargetPos_X.toPlainText())
        target_y = float(self.tbTargetPos_Y.toPlainText())
        target_z = float(self.tbTargetPos_Z.toPlainText())
        self.manupulator.MovePoint(jeus_manupulator_refactory.MoveMode.L_Move, target_x, target_y, target_z)


        self.manupulator.MovePoint(jeus_manupulator_refactory.MoveMode.L_Move, wait_x, wait_y, wait_z)

        self.manupulator.MoveJoints(safety_angles ,100, False)
        target_joint_0 = float(self.tbInitPos_0.toPlainText())
        target_joint_1 = float(self.tbInitPos_1.toPlainText())
        target_joint_2 = float(self.tbInitPos_2.toPlainText())
        target_joint_3 = float(self.tbInitPos_3.toPlainText())
        target_angles = [target_joint_0,target_joint_1,target_joint_2,target_joint_3]
        self.manupulator.MoveJoints(target_angles ,100, False)

    def sequence_all(self):
        self.sequence_DetectBtn()
        self.sequence_PushBtn()
    
    def Stream(self):
        if not self.is_stream:
            self.btnSeq_Stream.setText("Connecting Stream")
        else:
            self.btnSeq_Stream.setText("Disconnecting Stream \n WAIT!!!!")
        QApplication.processEvents()

        self.is_stream = self.vision.stream_on_off()
        if not self.is_stream:
            self.btnSeq_Stream.setText("Connect Stream")
        else:
            self.btnSeq_Stream.setText("Disconnect Stream")  

        
def main():
    app = QApplication([])
    ex = MainWindow()
    QApplication.processEvents()
    app.exec()
    if hasattr(ex, 'manupulator'):
        ex.manupulator.is_finish = True
    sys.exit()


if __name__ == '__main__':
    main()
