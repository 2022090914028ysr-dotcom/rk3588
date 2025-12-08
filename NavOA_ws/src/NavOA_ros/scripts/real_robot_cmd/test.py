#  def test_relative_pos_mode(self)->bool:
#         try:
#             command = [0x20,0x0F,0x00,0x00]
#             if self.send_command(self.device_id,0x06,command):
#                 print("设置异步模式命令发送成功") 
#             else:
#                 print("设置异步模式命令发送失败")
#                 return False
#             time.sleep(2)

#             command = [0x20,0x0D,0x00,0x01]
#             if self.send_command(self.device_id,0x06,command):
#                 print("电机设置相对位置模式命令发送成功") 
#             else:
#                 print("电机使设置相对位置模式命令发送失败")
#                 return False 
#             time.sleep(2)

#             command = [0x20,0x80,0x01,0xF4]
#             if self.send_command(self.device_id,0x06,command):
#                 print("设置左电机S型加速时间500ms命令发送成功") 
#             else:
#                 print("设置左电机S型加速时间500ms命令发送失败")
#                 return False
#             time.sleep(2)
            
#             command = [0x20,0x81,0x01,0xF4]
#             if self.send_command(self.device_id,0x06,command):
#                 print("设置右电机S型加速时间500ms命令发送成功") 
#             else:
#                 print("设置右电机S型加速时间500ms命令发送失败")
#                 return False
#             time.sleep(2)
            
#             command = [0x20,0x82,0x01,0xF4]
#             if self.send_command(self.device_id,0x06,command):
#                 print("设置左电机S型减速时间500ms命令发送成功") 
#             else:
#                 print("设置左电机S型减速时间500ms命令发送失败")
#                 return False
#             time.sleep(2)

#             command = [0x20,0x83,0x01,0xF4]
#             if self.send_command(self.device_id,0x06,command):
#                 print("设置右电机S型减速时间500ms命令发送成功") 
#             else:
#                 print("设置右电机S型减速时间500ms命令发送失败")
#                 return False
#             time.sleep(2)

#             command = [0x20,0x8E,0x00,0x32]
#             if self.send_command(self.device_id,0x06,command):
#                 print("设置左电机最大转速50RPM命令发送成功") 
#             else:
#                 print("设置左电机最大转速50RPM命令发送失败")
#                 return False
#             time.sleep(2)

#             command = [0x20,0x8F,0x00,0x32]
#             if self.send_command(self.device_id,0x06,command):
#                 print("设置右电机最大转速50RPM命令发送成功") 
#             else:
#                 print("设置右电机最大转速50RPM命令发送失败")
#                 return False
#             time.sleep(2)
            
#             command = [0x20,0x0E,0x00,0x08]
#             if self.send_command(self.device_id,0x06,command):      
#                 print("电机使能命令发送成功") 
#             else:
#                 print("电机使能命令发送失败")
#                 return False
#             time.sleep(5) 


#             command = [0x20,0x8A,0x00,0x02,0x04,0x00,0x00,0x50,0x00]
#             if self.send_command(self.device_id,0x10,command):
#                 print("设置左电机目标位置20480pulses命令发送成功") 
#             else:
#                 print("设置左电机目标位置20480pulses命令发送失败")
#                 return False
#             time.sleep(2)

#             command = [0x20,0x0E,0x00,0x11]
#             if self.send_command(self.device_id,0x06,command):
#                 print("左电机启动命令发送成功") 
#             else:
#                 print("左电机启动命令发送失败")
#                 return False 
#             time.sleep(2)

#             command = [0x20,0x8A,0x00,0x02,0x04,0xFF,0xFF,0xB0,0x00]
#             if self.send_command(self.device_id,0x10,command):
#                 print("设置左电机目标位置-20480pulses命令发送成功") 
#             else:
#                 print("设置左电机目标位置-20480pulses命令发送失败")
#                 return False  
#             time.sleep(2)

#             command = [0x20,0x0E,0x00,0x11]
#             if self.send_command(self.device_id,0x06,command):
#                 print("左电机启动命令发送成功") 
#             else:
#                 print("左电机启动命令发送失败")
#                 return False
#             time.sleep(2)

#             command = [0x20,0x8C,0x00,0x02,0x04,0x00,0x00,0x50,0x00]
#             if self.send_command(self.device_id,0x10,command):
#                 print("设置右电机目标位置20480pulses命令发送成功") 
#             else:
#                 print("设置右电机目标位置20480pulses命令发送失败")
#                 return False
#             time.sleep(2)

#             command = [0x20,0x0E,0x00,0x12]
#             if self.send_command(self.device_id,0x06,command):
#                 print("右电机启动命令发送成功") 
#             else:
#                 print("右电机启动命令发送失败")
#                 return False 
#             time.sleep(2)

#             command = [0x20,0x8C,0x00,0x02,0x04,0xFF,0xFF,0xB0,0x00]
#             if self.send_command(self.device_id,0x10,command):
#                 print("设置右电机目标位置-20480pulses命令发送成功") 
#             else:
#                 print("设置右电机目标位置-20480pulses命令发送失败")
#                 return False  
#             time.sleep(2)

#             command = [0x20,0x0E,0x00,0x12]
#             if self.send_command(self.device_id,0x06,command):
#                 print("右电机启动命令发送成功") 
#             else:
#                 print("右电机启动命令发送失败")
#                 return False
#             time.sleep(2)

#             command = [0x20,0x0E,0x00,0x07]
#             if self.send_command(self.device_id,0x06,command):
#                 print(f"电机失能命令发送成功")     
#             else:
#                 print("电机失能命令发送失败")   
#                 return False
                  
#             return True
#         except Exception as e:
#             print(f"电机设置相对位置模式出现错误: {e}")
#             return False
        
#         def test_velocity_mode(self)->bool:
#         try:
#             command = [0x20,0x0D,0x00,0x03]
#             if self.send_command(self.device_id,0x06,command):
#                 print("设置速度模式命令发送成功") 
#             else:
#                 print("设置速度模式命令发送失败")
#                 return False
#             time.sleep(2)

#             command = [0x20,0x80,0x01,0xF4]
#             if self.send_command(self.device_id,0x06,command):
#                 print("设置左电机S型加速时间500ms命令发送成功") 
#             else:
#                 print("设置左电机S型加速时间500ms命令发送失败")
#                 return False
#             time.sleep(2)
            
#             command = [0x20,0x81,0x01,0xF4]
#             if self.send_command(self.device_id,0x06,command):
#                 print("设置右电机S型加速时间500ms命令发送成功") 
#             else:
#                 print("设置右电机S型加速时间500ms命令发送失败")
#                 return False
#             time.sleep(2)
            
#             command = [0x20,0x82,0x01,0xF4]
#             if self.send_command(self.device_id,0x06,command):
#                 print("设置左电机S型减速时间500ms命令发送成功") 
#             else:
#                 print("设置左电机S型减速时间500ms命令发送失败")
#                 return False
#             time.sleep(2)

#             command = [0x20,0x83,0x01,0xF4]
#             if self.send_command(self.device_id,0x06,command):
#                 print("设置右电机S型减速时间500ms命令发送成功") 
#             else:
#                 print("设置右电机S型减速时间500ms命令发送失败")
#                 return False
#             time.sleep(2)
            
#             command = [0x20,0x0E,0x00,0x08]
#             if self.send_command(self.device_id,0x06,command):      
#                 print("电机使能命令发送成功") 
#             else:
#                 print("电机使能命令发送失败")
#                 return False
#             time.sleep(2) 


#             # command = [0x20,0x88,0x00,0x64]
#             # if self.send_command(self.device_id,0x06,command):
#             #     print("设置左电机目标转速100RPM命令发送成功") 
#             # else:
#             #     print("设置左电机目标转速100RPM命令发送失败")
#             #     return False
#             # time.sleep(2)

#             # command = [0x20,0x88,0xFF,0x9C]
#             # if self.send_command(self.device_id,0x06,command):
#             #     print("设置左电机目标转速-100RPM命令发送成功") 
#             # else:
#             #     print("设置左电机目标转速-100RPM命令发送失败")
#             #     return False 
#             # time.sleep(2)

#             # command = [0x20,0x0E,0x00,0x07]
#             # if self.send_command(self.device_id,0x06,command):
#             #     print(f"电机失能命令发送成功")     
#             # else:
#             #     print("电机失能命令发送失败")   
#             #     return False

#             command = [0x20,0x89,0x00,0x64]
#             if self.send_command(self.device_id,0x06,command):
#                 print("设置右电机目标转速100RPM命令发送成功") 
#             else:
#                 print("设置右电机目标转速100RPM命令发送失败")
#                 return False 
#             time.sleep(2)

#             command = [0x20,0x89,0xFF,0x9C]
#             if self.send_command(self.device_id,0x06,command):
#                 print("设置右电机目标转速-100RPM命令发送成功") 
#             else:
#                 print("设置右电机目标转速-100RPM命令发送失败")
#                 return False 
#             time.sleep(2)

#             command = [0x20,0x0E,0x00,0x07]
#             if self.send_command(self.device_id,0x06,command):
#                 print(f"电机失能命令发送成功")     
#             else:
#                 print("电机失能命令发送失败")   
#                 return False
                  
#             return True
#         except Exception as e:
#             print(f"电机设置速度模式出现错误: {e}")
#             return False
        
                
# def motor_test():
#     controller = MotorController()
#     try:
#         if not controller.initialize():
#             print("电机初始化失败")
#         else:    
#             controller.start_reading()
#             time.sleep(0.2)
#             controller.test_velocity_mode()
#             # controller.test_relative_pos_mode()
#         while(1):
#             time.sleep(10)
#     except KeyboardInterrupt:
#         controller.stop_reading()
#         controller.running = False
#         print("用户键盘中断")
