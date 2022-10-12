from PySide6.QtGui import (QBrush, QColor, QConicalGradient, QCursor,
    QFont, QFontDatabase, QGradient, QIcon,
    QImage, QKeySequence, QLinearGradient, QPainter,
    QPalette, QPixmap, QRadialGradient, QTransform)
from PySide6.QtWidgets import (QApplication, QComboBox, QGroupBox, QLabel,
    QMainWindow, QPlainTextEdit, QPushButton, QSizePolicy,
    QWidget)
from PySide6.QtCore import *
import os
import sys
import threading
from jeus_armcontrol import *
import yaml
from ui_jr_main_form import *

config_init_posi = 'init_position'
config_wait_posi = 'wait_position'
config_target_posi = 'target_position'
x = 'x'
y = 'y'
z = 'z'
ry = 'ry'

class worker(QThread):
    def __init__(self,parent,  func, *args):
        super().__init__(parent)
        self.func = func
        self.val = args
    def run(self):
        self.func(self.val[0],self.val[1])

class MainWindow(QMainWindow, Ui_MainWindow):
    def __init__(self):
        super().__init__()
        self.setupUi(self)
        self.show()
        self.setconfig()
        # self.connect_device()
        # self.monitor_onoff(True)
        self.timer = QTimer(self)
        self.timer.start(10)
        self.timer.timeout.connect(self.get_current)




    def setconfig(self):
        path =os.path.join(os.getcwd(),'Config','model_config.yaml')
        with open(path) as fileopen:
            model_config = yaml.load(fileopen, Loader=yaml.FullLoader)
            init_pos = model_config[config_init_posi]
            self.tbInitPos_X.setPlainText(str(init_pos[x]))
            self.tbInitPos_Y.setPlainText(str(init_pos[y]))
            self.tbInitPos_Z.setPlainText(str(init_pos[z]))
            self.tbInitPos_Ry.setPlainText(str(init_pos[ry]))

            wait_pos = model_config[config_wait_posi]
            self.tbWaitPos_X.setPlainText(str(wait_pos[x]))
            self.tbWaitPos_Y.setPlainText(str(wait_pos[y]))
            self.tbWaitPos_Z.setPlainText(str(wait_pos[z]))
            self.tbWaitPos_Ry.setPlainText(str(wait_pos[ry]))

            target_pos = model_config[config_target_posi]
            self.tbTargetPos_X.setPlainText(str(target_pos[x]))
            self.tbTargetPos_Y.setPlainText(str(target_pos[y]))
            self.tbTargetPos_Z.setPlainText(str(target_pos[z]))
            self.tbTargetPos_Ry.setPlainText(str(target_pos[ry]))


        
    def get_current(self):
        if hasattr(self, 'manupulator') and self.IsConnect:
            positions = self.manupulator.get_current_joint_pos()
            key_hard = sorted(positions.keys())

            self.tbCurrentPosJoint1.setPlainText(str(round(positions[key_hard[0]], 3)))
            self.tbCurrentPosJoint2.setPlainText(str(round(positions[key_hard[1]], 3)))
            self.tbCurrentPosJoint3.setPlainText(str(round(positions[key_hard[2]], 3)))
            self.tbCurrentPosJoint4.setPlainText(str(round(positions[key_hard[3]], 3)))

    def connect_device(self):
        if not hasattr(self, 'IsConnect') or not self.IsConnect:
            self.btnConnect.setText('connectting')
            self.manupulator = jeus_maunpulator()
            self.manupulator.get_param_value(os.path.join(os.getcwd(),'Config'),'arm_config.yaml')
            if self.manupulator.generate_module():
                self.IsConnect = True
                self.btnConnect.setText('connected')
                self.manupulator.torque_on()
        else:
            self.IsConnect = False
            self.manupulator.disconnect()
            self.btnConnect.setText('disconnected')

    def torque_onoff(self):
        sender = self.sender()        
        sender_idx = int(sender.objectName().split('_')[1])
        if self.manupulator.get_torque_status(sender_idx):
            self.manupulator.torque_off(sender_idx)
        else:
            self.manupulator.torque_on(sender_idx)

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
            print(f'Error : Joint {sender_idx} torque off')
            return
        # w = worker(self,self.manupulator.move_joint,sender_idx,angle)
        # w.start()
        self.manupulator.move_joint(sender_idx,angle)
    def calculate_xyz(self):
        x,y,z= self.manupulator.get_current_xyz()
        self.tbCurrent_X.setPlainText(str(x))
        self.tbCurrent_Y.setPlainText(str(y))
        self.tbCurrent_Z.setPlainText(str(z))


    def move_J(self):        
        sender = self.sender()        
        sender_name = sender.objectName()
        x = 0.0
        y = 0.0
        z = 0.0

        if sender_name == self.btnMoveInitPos.objectName():
            x = float(self.tbInitPos_X.toPlainText())
            y = float(self.tbInitPos_Y.toPlainText())
            Z = float(self.tbInitPos_Z.toPlainText())

        elif sender_name == self.btnMoveWaitPos.objectName():
            x = float(self.tbInitPos_X.toPlainText())
            y = float(self.tbInitPos_Y.toPlainText())
            Z = float(self.tbInitPos_Z.toPlainText())
        
        elif sender_name == self.btnMoveTargetPos_J.objectName():
            x = float(self.tbInitPos_X.toPlainText())
            y = float(self.tbInitPos_Y.toPlainText())
            Z = float(self.tbInitPos_Z.toPlainText())
        else:
            return
        self.manupulator.move_point(MoveMode.J_Move, x, y, z)

    def move_L(self):
        pass
    def sequence_push(self):
        pass
    def sequence_init(self):
        pass
    def sequence_all(self):
        pass
    def save(self):
        path =os.path.join(os.getcwd(),'Config','model_config.yaml')
        data=\
            { config_init_posi:{x:float(self.tbInitPos_X.toPlainText()), y:float(self.tbInitPos_Y.toPlainText()), z: float(self.tbInitPos_Z.toPlainText()), ry: float(self.tbInitPos_Ry.toPlainText())}  \
            , config_wait_posi:{x:float(self.tbWaitPos_X.toPlainText()), y:float(self.tbWaitPos_Y.toPlainText()), 'z': float(self.tbWaitPos_Z.toPlainText()),'ry': float(self.tbWaitPos_Ry.toPlainText())}   \
            , config_target_posi:{'x':float(self.tbTargetPos_X.toPlainText()), 'y':float(self.tbTargetPos_Y.toPlainText()), 'z': float(self.tbTargetPos_Z.toPlainText()),'ry': float(self.tbTargetPos_Ry.toPlainText())}}
        with open(path,'w') as fileopen:
            yaml.dump(data,fileopen,default_flow_style=False)
        

app = QApplication([])
ex = MainWindow()
QApplication.processEvents()
app.exec_()
