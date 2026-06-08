class CommunicationConfig:
    def __init__(self):
        # 根据操作系统自动选择串口
        import platform
        if platform.system() == 'Windows':
            self.port = "COM13"  # Windows系统默认串口
        else:
            self.port = "/dev/ttyS7"  # Linux系统默认串口
        
        self.baudrate = 115200
        self.timeout = 0.001

config = CommunicationConfig()