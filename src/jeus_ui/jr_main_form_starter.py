import sys
from jr_main_form import Ui_MainWindow
from PyQt5 import *
from PyQt5.QtCore import *
from PyQt5.QtWidgets import *

class jr_main_form_starter(QMainWindow, Ui_MainWindow):
    def __init__(self):
        super().__init__()
        self.setupUi(self)
        self.show()

app = QApplication([])
ex = jr_main_form_starter()
QApplication.processEvents()
sys.exit(app.exec_())