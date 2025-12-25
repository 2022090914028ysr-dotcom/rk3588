import serial
from dataclasses import dataclass
import struct
import platform
import time
import threading
from typing import List, Optional, Tuple
from motor_config import config

@dataclass
class RS485Frame:
    device_id: int = 0x01
    function_code: int = 0
    address:int = 0
    address_count:int = 0
    bytes_count:int = 0
    data: List[int] = None


class MotorController:
    def __init__(self):
        self.serial_port = None
        self.running = False
        self.device_id = 0x01
        self.read_thread = None
        self.reading_reg_address:List[int] = [0x20,0xAB] # 读线程正在读取的寄存器地址
        self.reading_reg_count = 2 # 读线程正在读取的寄存器数量
        self.error_count = 0
        self.max_error_count = 10
        self.checksum_type = 'modbus'    # 校验和类型：modbus, crc16, xor
        self.l_motor_rpm = 0 # 左电机转速，单位：rpm
        self.r_motor_rpm = 0 # 右电机转速，单位：rpm

    def initialize(self) -> bool:
        try:
            self.serial_port = serial.Serial(
                port=config.port,
                baudrate=config.baudrate,
                timeout=config.timeout
            )
            print(f"通讯初始化完成 - 端口: {config.port}, 波特率: {config.baudrate}")

            if self.set_velocity_mode():
                time.sleep(0.005)
                if self.set_sync_mode():
                    time.sleep(0.005)
                    if self.enable_motor():
                        time.sleep(0.005)
                        if self.set_sync_velocity(0,0):
                            self.running = True
                            time.sleep(0.005)
                            self.start_reading()
                            return self.running

            raise Exception("初始化失败")
                 
        except Exception as e:
            print(f"初始化出现错误: {e}")
            return False
        
    def close(self) -> bool:
        try:
            if self.set_sync_velocity(0,0):
                time.sleep(0.005)
                if self.disable_motor():
                    self.running = False
                    time.sleep(0.005)
                    self.stop_reading()
                    return not self.running
                    
            raise Exception("关闭电机失败")
        except Exception as e:
            print(f"关闭电机出现错误: {e}")
            return False
        
    def _verify_checksum(self, frame: bytes) -> bool:
        """验证帧的校验和"""
        # 提取校验和前的数据
        data_before_checksum = frame[:-2]  
        received_checksum = frame[-2:]
        # 根据配置的校验和类型计算
        if self.checksum_type == 'modbus':
            calculated_checksum = self._calculate_crc16(data_before_checksum)
        elif self.checksum_type == 'crc16':
            calculated_checksum = self._calculate_crc16(data_before_checksum)
        elif self.checksum_type == 'xor':
            calculated_checksum = self._calculate_xor_checksum(data_before_checksum)
        else:
            return False
        # 比较校验和
        return calculated_checksum == received_checksum
    
    
    def _calculate_crc16(self, data: bytes) -> list:
        """计算CRC16校验和"""
        crc = 0xFFFF
        for byte in data:
            crc ^= byte
            for _ in range(8):
                if crc & 0x0001:
                    crc = (crc >> 1) ^ 0xA001
                else:
                    crc >>= 1
        # 小端序输出
        return bytes([crc & 0xFF, (crc >> 8) & 0xFF])
    
    
    def _calculate_xor_checksum(self, data: bytes) -> bytes:
        """计算XOR校验和"""
        checksum = 0
        for byte in data:
            checksum ^= byte
        return bytes([checksum])
        
    def _parse_frame(self, frame: bytes) -> Optional[RS485Frame]:
        """解析RS4835原始数据帧"""
        try:
            # 检查校验和
            if not self._verify_checksum(frame):
                print("数据校验和不通过")
                return None

            device_id = frame[0]
            if device_id != self.device_id:
                print("设备id错误：{device_id}")
                return None
             
            function_code = frame[1]         

            if function_code == 0x06:    # 写单个寄存器
                address = (frame[2] << 8) | frame[3]
                address_count = 1
                bytes_count = 0
                data = list(frame[4:6])
            
            elif function_code == 0x10: # 写多个寄存器
                address = (frame[2] << 8) | frame[3]
                address_count = (frame[4] << 8) | frame[5]
                bytes_count = 0
                data = None

            elif function_code == 0x03: # 读寄存器
                address = 0
                address_count = 0
                bytes_count = frame[2]
                data = list(frame[3:3+bytes_count])
            else:
                return None
      
            return RS485Frame(
                device_id=device_id,
                function_code=function_code,
                address=address,
                address_count=address_count,
                bytes_count = bytes_count,
                data=data
            )           
        except Exception as e:
            print(f"RS4835数据解析出现错误: {e}")
            return None
        
    def _parse_motor_rpm(self,byte_array):
        # 解析读取的寄存器值为左右电机转速（单位：rpm）
        try:
            # 验证输入长度
            if len(byte_array) != 4:
                raise ValueError("字节数组长度必须为4")
            
            # 验证所有值都是有效的字节（0-255）
            for b in byte_array:
                if not 0 <= b <= 255:
                    raise ValueError(f"无效的字节值: {b}")
            
            # 提取左右电机的字节（大端序）
            left_motor_bytes = bytes(byte_array[0:2])  # 前两位是左电机
            right_motor_bytes = bytes(byte_array[2:4])  # 后两位是右电机
            
            # 使用struct解析大端序有符号整数
            left_raw_value = struct.unpack('>h', left_motor_bytes)[0]
            right_raw_value = struct.unpack('>h', right_motor_bytes)[0]
            
            
            # 转换单位：0.1rpm/min -> rpm
            left_rpm = left_raw_value / 10.0
            right_rpm = right_raw_value / 10.0

            self.l_motor_rpm = left_rpm
            self.r_motor_rpm = right_rpm

            print(f"左电机转速: {left_rpm:.2f} rpm, 右电机转速: {right_rpm:.2f} rpm")
            
        except Exception as e:
            print(f"解析错误: {e}")
            return None
        
    def start_reading(self)->bool:
        """开始读取数据"""
        if not self.serial_port or not self.running:
            print("未初始化，无法开始读取")
            return False
        
        self.read_thread = threading.Thread(target=self._read_loop, daemon=True)
        self.read_thread.start()
        print("RS4835数据读取线程已启动")
        return True
        
    def stop_reading(self):
        """停止读取数据"""
        if self.read_thread and self.read_thread.is_alive():
            self.read_thread.join(timeout=1.0)
        print("RS4835数据读取已停止")

    def set_reading_reg_param(self,reg_address:List[int],reg_count:int):
        """设置读线程正在读取的寄存器地址和数量"""
        self.reading_reg_address = reg_address
        self.reading_reg_count = reg_count
    
    def _read_loop(self):
        """数据读取循环"""
        while self.running:
            try:
                if self._read_data(self.reading_reg_address,self.reading_reg_count):
                    self.error_count = 0
                else:
                    self.error_count += 1
                    if self.error_count >= self.max_error_count:
                        print(f"连续读取失败 {self.max_error_count} 次，可能存在硬件问题")
                        self.error_count = 0  # 重置计数器，继续尝试
                
                time.sleep(1)  # 100Hz读取频率
                
            except Exception as e:
                print(f"数据读取循环异常: {e}")
                time.sleep(0.1)
    
    def _read_data(self,reg_address:List[int],reg_count:int) -> bool:
        """读取一次数据"""
        try:
            self.serial_port.flushInput()
            read_command = bytearray()
            read_command.append(self.device_id)
            read_command.append(0x03)
            read_command.extend(reg_address)

            # 寄存器数量转为字节
            high_byte = (reg_count >> 8) & 0xFF
            low_byte = reg_count & 0xFF 

            read_command.extend([high_byte,low_byte])

            # 添加校验和
            if self.checksum_type in ['modbus', 'crc16']:
                checksum = self._calculate_crc16(read_command)
                read_command.extend(checksum)
            elif self.checksum_type == 'xor':
                checksum = self._calculate_xor_checksum(read_command)
                read_command.extend(checksum)
            
            # 发送读取命令
            self.serial_port.write(read_command)
            
           # 等待数据
            start_time = time.time()
            while time.time() - start_time < 0.05:  # 最多等待50ms
                count = self.serial_port.inWaiting()
                if count >= 2*reg_count+5:
                    break
                time.sleep(0.005)
            
            # 读取数据
            com_input = self.serial_port.read(count)
            
            # 验证数据格式
            if len(com_input) != 5+2*reg_count or com_input[0] != self.device_id or com_input[1] != 0x03:
                print(f"数据格式错误,数据长度：{len(com_input)},数据：{com_input}")
                return False
            
            # 解析数据
            parsed_frame = self._parse_frame(com_input)
            if parsed_frame:
                print(parsed_frame)
                if reg_address==[0x20,0xAB]:
                    self._parse_motor_rpm(parsed_frame.data) # 如果正在读取的为左右轮子转速则解析并保存
                return True          
            return False
            
        except Exception as e:
            print(f"RS4835数据读取出现错误: {e}")
            return False
        
    def send_command(self,device_id:int,function_code:int,data:List[int]=None)->bool:
        try:
            if not self.serial_port or not self.serial_port.is_open:
                return False
            #构建命令帧

            command = bytearray()
            command.append(device_id)
            command.append(function_code)

            if data:
                command.extend(data)
            else:
                return False
            
            # 添加校验和
            if self.checksum_type in ['modbus', 'crc16']:
                checksum = self._calculate_crc16(command)
                command.extend(checksum)
            elif self.checksum_type == 'xor':
                checksum = self._calculate_xor_checksum(command)
                command.extend(checksum)
  
            #print(f"发送的指令：{[hex(b) for b in command]}")
            # 发送命令
            self.serial_port.write(command)
            return True
            
        except Exception as e:
            print(f"命令发送出现错误: {e}")
            return False  

    def set_sync_mode(self)->bool:
        # 设置电机为同步控制模式
        try:
            command = [0x20,0x0F,0x00,0x01]
            if self.send_command(self.device_id,0x06,command):                 
                print("电机设置同步控制模式命令发送成功") 
            else:
                print("电机设置同步控制模式命令发送失败")
                return False     
            return True 
        except Exception as e:
            print(f"电机设置同步控制模式出现错误: {e}")
            return False


    def enable_motor(self)->bool:
        try:
            command = [0x20,0x0E,0x00,0x08]
            if self.send_command(self.device_id,0x06,command):                 
                print("电机使能命令发送成功") 
            else:
                print("电机使能命令发送失败")
                return False     
            return True 
        except Exception as e:
            print(f"电机使能出现错误: {e}")
            return False

    def disable_motor(self)->bool:
        try:
            command = [0x20,0x0E,0x00,0x07]
            if self.send_command(self.device_id,0x06,command):
                print(f"电机停机命令发送成功")     
            else:
                print("电机停机命令发送失败")   
                return False  
            return True 
        except Exception as e:
            print(f"电机停机出现错误: {e}")
            return False
        
    def set_velocity_mode(self)->bool:
        try:
            command = [0x20,0x0D,0x00,0x03]
            if self.send_command(self.device_id,0x06,command):
                print(f"电机设置速度模式命令发送成功")     
            else:
                print("电机设置速度模式命令发送失败")   
                return False
            return True   
        except Exception as e:
            print(f"电机设置速度模式出现错误: {e}")
            return False
        
    def value_to_hex(self,value:int)->List[int]:

        # 将值转换为两个字节的十六进制表示

        byte_data = struct.pack('<h', value)
        
        # 提取低字节和高字节
        low_byte = byte_data[0]
        high_byte = byte_data[1]
        
        return [high_byte, low_byte]
    
    def set_sync_velocity(self,velocity_l:int,velocity_r:int)->bool:
        try:
            l_rpm_hex = self.value_to_hex(velocity_l)
            r_rpm_hex = self.value_to_hex(-velocity_r)
            command = [0x20,0x88,0x00,0x02,0x04]
            command.extend(l_rpm_hex)
            command.extend(r_rpm_hex)
            if self.send_command(self.device_id,0x10,command):
                print(f"电机设置同步速度命令发送成功")     
            else:
                print("电机设置同步速度命令发送失败")
                return False   
            return True   
        except Exception as e:
            print(f"电机设置同步速度出现错误: {e}")
            return False
    
    def set_l_motor_velocity(self,velocity:int)->bool:
        try:
            rpm_hex = self.value_to_hex(velocity)
            command = [0x20,0x88]
            command.extend(rpm_hex)
            if self.send_command(self.device_id,0x06,command):
                print(f"左电机设置速度命令发送成功")     
            else:
                print("左电机设置速度命令发送失败")   
                return False   
            return True
        except Exception as e:
            print(f"左电机设置速度出现错误: {e}")
            return False
     
    def set_r_motor_velocity(self,velocity:int)->bool:
        try:
            velocity = -velocity # 右侧电机速度取反
            rpm_hex = self.value_to_hex(velocity)
            command = [0x20,0x89]
            command.extend(rpm_hex)
            if self.send_command(self.device_id,0x06,command):
                print(f"右电机设置速度命令发送成功")     
            else:
                print("右电机设置速度命令发送失败")   
                return False   
            return True
        except Exception as e:
            print(f"右电机设置速度出现错误: {e}")
            return False
        
    def set_l_motor_accelerate_time(self,time:int)->bool:
        #设置左电机S型加速时间，单位：ms
        try:
            time_hex = self.value_to_hex(time)
            command = [0x20,0x80]
            command.extend(time_hex)
            if self.send_command(self.device_id,0x06,command):
                print(f"左电机设置S型加速时间命令发送成功")     
            else:
                print("左电机设置S型加速时间命令发送失败")   
                return False   
            return True
        except Exception as e:
            print(f"左电机设置S型加速时间出现错误: {e}")
            return False
    
    def set_r_motor_accelerate_time(self,time:int)->bool:
        try:
            time_hex = self.value_to_hex(time)
            command = [0x20,0x81]
            command.extend(time_hex)
            if self.send_command(self.device_id,0x06,command):
                print(f"右电机设置S型加速时间命令发送成功")     
            else:
                print("右电机设置S型加速时间命令发送失败")   
                return False   
            return True
        except Exception as e:
            print(f"右电机设置S型加速时间出现错误: {e}")
            return False
        
    def set_l_motor_decelerate_time(self,time:int)->bool:
        #设置左电机S型减速时间，单位：ms
        try:
            time_hex = self.value_to_hex(time)
            command = [0x20,0x82]
            command.extend(time_hex)
            if self.send_command(self.device_id,0x06,command):
                print(f"左电机设置S型减速时间命令发送成功")     
            else:
                print("左电机设置S型减速时间命令发送失败")   
                return False   
            return True
        except Exception as e:
            print(f"左电机设置S型减速时间出现错误: {e}")
            return False
        
    def get_motor_speeds(self) -> Tuple[float, float]:
        """获取左右电机转速"""
        return self.l_motor_rpm, self.r_motor_rpm    

    
