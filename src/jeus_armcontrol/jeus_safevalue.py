import threading
class JR_SafeValue():
    def __init__(self,initValue) -> None:
        self.lock = threading.Lock()
        self.value = initValue
    
    def get_value(self):
        return self.value
    
    def set_value(self, changeValue):
        with self.lock:
            self.value = changeValue