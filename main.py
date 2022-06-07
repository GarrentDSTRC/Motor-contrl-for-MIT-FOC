import serial
import motor
import time

motor_id_list = [9]
for i in  len(motor_id_list):
    motor[i]=motor(motor_id_list[i])

s_uart = serial.Serial('com4', 115200)

s_uart.bytesize = 8

s_uart.stopbits = 1

s_uart.parity = "N"

s_can=s_uart


motor[1].motor_enable(s_uart)

time.sleep(1)

n = s_uart.readline()

if n:
    data = [hex(x) for x in bytes(n)]

    print(data)

s_uart.flush()

# print('ok')

# 关闭 串口

if s_uart.isOpen():
    s_uart.close()


