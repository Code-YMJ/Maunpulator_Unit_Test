from datetime import datetime
import logging 

class jeus_log:
    def __init__(self, path, log_name) -> None:
        self.log = logging.getLogger(log_name)
        self.log.setLevel(logging.DEBUG)
        fommat_s = logging.Formatter(f'{log_name} '+'[%(levelname)s] (%(asctime)s) > %(message)s')
        fommat_f = logging.Formatter('[%(levelname)s] (%(asctime)s) > %(message)s')
        file_handler = logging.FileHandler(f'{path}\\{log_name}.txt')
        stream_handler = logging.StreamHandler()
        file_handler.setFormatter(fommat_f)
        stream_handler.setFormatter(fommat_s)
        self.log.addHandler(file_handler)
        self.log.addHandler(stream_handler)

    def Debug(self, msg):
        self.log.debug(msg)
    
    def Info(self, msg):
        self.log.info(msg)

    def Warning(self, msg):
        self.log.warning(msg)

    def Error(self, msg):
        self.log.error(msg)
        
    def Critical(self, msg):
        self.log.critical(msg)