import serial
from collections import deque
import time

def process_data(coordinates):
    uart_dev="/dev/ttyS1"
    baudrate = "115200"
    ser = serial.Serial(uart_dev, int(baudrate), timeout=1)

    xL, yL, width, height = coordinates
    avg_x = (xL+width+xL) / 2
    if avg_x > 380:
        Send_data="@Right\r\n"

    elif avg_x >= 310:
        Send_data="@Keep\r\n"

    else:
        Send_data="@Left\r\n"

    write_num=ser.write(Send_data.encode('UTF-8'))
    print("Send: ", Send_data)
    received_data = ser.readline().decode('UTF-8')
    print("Recv: ", received_data)
    time.sleep(0.01)
    
