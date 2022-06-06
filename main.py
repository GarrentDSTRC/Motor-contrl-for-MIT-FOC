import serial

import time

motor_enable_date = b'\OxFF\OxFF\OxFF\OxFF\OxFF\OxFF\OxFF\OXFC'

motor_speed_mode = b'\x02\x00\xC4\xC6'

motor_accdec_set = b'\x0A\x14\x14\x32'

motor_speed_set = b'\x06\x00\x88\x8E'

motor_status = b'\x80\x00\x80'

# 设置速度模式02 00 C4 C6

# 设置加减速时间0A 14 14 32

# 设置转速06 00 88 8E

# 使能电机00 00 01 01

# 停机电机 00 00 00 00


s = serial.Serial('com4', 57600)

s.bytesize = 8

s.stopbits = 1

s.parity = "N"

motor_enable(s)

time.sleep(1)

n = s.readline()

if n:
    data = [hex(x) for x in bytes(n)]

    print(data)

s.flush()

# print('ok')

# 关闭 串口

if s.isOpen():
    s.close()


def motor_enable(serial):
    try:

        serial.write(motor_speed_mode)

        serial.write(motor_accdec_set)

        time.sleep(0.1)

        serial.write(motor_speed_set)

        time.sleep(0.1)

        serial.write(motor_enable_date)

        time.sleep(0.1)

        serial.write(motor_status)

        time.sleep(1)

        serial.write(motor_status)

        time.sleep(1)

        serial.write(motor_status)

        time.sleep(1)

        serial.write(motor_status)

        time.sleep(1)

        print('send ok')

    except:

        print('cant write motor_enable')