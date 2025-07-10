import rclpy
from rclpy.node import Node
from ai_msgs.msg import PerceptionTargets
from Uart import process_data



class DetectionParser(Node):
    def __init__(self):
        super().__init__('detection_parser')
        self.subscription = self.create_subscription(
            PerceptionTargets,
            '/hobot_dnn_detection',
            self.callback,
            10)
        self.coordinates = [0, 0, 0, 0]
    def callback(self, msg):
        for target in msg.targets:  # 遍历所有检测目标
            target_type = target.type  # 获取目标类别
            for roi in target.rois:  # 遍历每个目标的ROI信息
                rect = roi.rect  # 获取位置框
                x = rect.x_offset
                y = rect.y_offset
                w = rect.width
                h = rect.height
                confidence = roi.confidence  # 置信度
                # if target_type=="bottle":
                self.get_logger().info(
                    f"[{target_type}] x:{x}, y:{y}, w:{w}, h:{h}, confidence:{confidence:.2f}"
                    )
                self.coordinates[0]=x
                self.coordinates[1]=y 
                self.coordinates[2]=w 
                self.coordinates[3]=h 
                # if target_type=="bottle":
                #     process_data(self.coordinates)
def main():
    
    rclpy.init()
    node = DetectionParser()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()



# import sys
# import signal
# import os
# import time

# # 导入python串口库
# import serial
# import serial.tools.list_ports

# def serialSend(name, dat):
#     uart_dev="/dev/ttyS1"
#     baudrate = "9600"
#     ser = serial.Serial(uart_dev, int(baudrate), timeout=1) 
#     print(ser)

#     Center = (dat[0] + dat[2])/2 #计算矩形框的中心位置

#     if dat[2]-dat[0]<=50 :
#         if Center >= 280:
#             Send_data = "@Right\r\n"
#         elif Center >= 240:
#             Send_data = "@Keep\r\n"
#         else :
#             Send_data = "@Left\r\n"
#     else :
#         Send_data="@Near\r\n"
#     write_num = ser.write(Send_data.encode('UTF-8'))
#     print("Send: ", Send_data)
#     # received_data = ser.read(write_num).decode('UTF-8')
#     # print("Recv: ", received_data)

#     # time.sleep(0.5)
#     ser.close()
#     return 0

import serial
from collections import deque
import time
# 初始化串口对象
# uart_dev="/dev/ttyS1"
# baudrate = "9600"
# ser = serial.Serial(uart_dev, int(baudrate), timeout=1)  # 根据实际情况修改端口号和波特率

# 创建一个队列来存储接收到的参数
queue = deque(maxlen=5)

def process_data(coordinates):
    uart_dev="/dev/ttyS1"
    baudrate = "115200"
    ser = serial.Serial(uart_dev, int(baudrate), timeout=1)
    # label, coordinates = data
    xL, yL, width, height = coordinates
    avg_x = (xL+width+xL) / 2
    if avg_x > 380:
        Send_data="@Right\r\n"
        # ser.write(b'Left\n')
    elif avg_x >= 310:
        Send_data="@Keep\r\n"
        # ser.write(b'Keep\n')
    else:
        Send_data="@Left\r\n"
    # if width>=70 :
    #     Send_data="@Near\r\n"
    write_num=ser.write(Send_data.encode('UTF-8'))
    print("Send: ", Send_data)
    received_data = ser.readline().decode('UTF-8')
    print("Recv: ", received_data)
    time.sleep(0.01)
    

def serialSend(label, coordinates):
    queue.append((label, coordinates))
    
    if len(queue) == 5:
        for data in queue:
            process_data(data)
        queue.clear()

