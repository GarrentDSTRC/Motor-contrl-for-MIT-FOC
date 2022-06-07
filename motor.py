import numpy as np
import time
import struct
import serial
import threading

T_LIMIT=100


class motor():

    def __init__(self, motor_id, com1='com4',com2='com4', bps1=115200,bps2=115200):
        self.id=motor_id

        self.disable = -4
        self.enable = -3
        self.setzero= -2

        # 设置串口通讯
        self.serial_uart = serial.Serial(com1, bps1)
        self.serial_uart.timeout = 0
        self.serial_uart.bytesize = 8
        self.serial_uart.stopbits = 1
        self.serial_uart.parity = "N"

        self.serial_can = serial.Serial(com2, bps2)
        self.serial_can.timeout = 0
        self.serial_can.bytesize = 8
        self.serial_can.stopbits = 1
        self.serial_can.parity = "N"

    def motor_enable(self):
        try:
            self.serial_uart.write(struct.pack(">ii", -4,self.enable))
        except:
            print('cant write motor_enable')

    def motor_disable(self):
            self.serial_uart.write(struct.pack(">ii", -4,self.disable))

    def motor_setzero(self):
        try:
            self.serial_can.write(struct.pack(">ii", -4,self.setzero))
        except:
            print('cant write motor_setzero')

    def motor_sent(self,mode,MotorPara):
        #多线程
        threads = []
        t1 = threading.Thread(target=self.motor_read)
        threads.append(t1)

        if mode=='P' :
            t2 = threading.Thread(target=self.motor_Pctrl,args=(MotorPara))
        elif  mode=='V' :
            t2 = threading.Thread(target=self.motor_sent)
        elif mode=='T':
            t2 = threading.Thread(target=self.motor_sent)
        else:
            print("motor_sent wrong")

        threads.append(t2)
        for t in threads:
            t.start()

    def motor_Pctrl(self,MotorPara):
        # pdes=np.linspace(0,95.5,100)
        ID=MotorPara[0,:,0]
        Pdes=MotorPara[1,:,0]
        Vdes = MotorPara[2, :, 0]
        tff = MotorPara[3, :, 0]
        kd = MotorPara[4, :, 0]
        kp = MotorPara[5, :, 0]

        sent=[]
        for i in T_LIMIT:
            sent.append()
            struct.pack(">ii", a, b)

        for i in Pdes:

            self.serial_can.write(i)

            time.sleep(0.1)

    def motor_read(self):
        while True:
            n = self.serial_uart.inWaiting()  # 等待数据的到来，并得到数据的长度
            if n:  # 如果有数据
                n = self.serial_uart.read(8)  # 读取n位数据
                s = [hex(x) for x in bytes(n)]
                id=int(s[0],16)<<4+int(s[1],16)
                p=int(s[2],16)<<4+int(s[3],16)
                v=int(s[4],16)<<12+int(s[5],16)<<8+int(s[6],16)<<4+int(s[7],16)
                t=int(s[8],16)<<8+int(s[9],16)<<4+int(s[10],16)

            time.sleep(0.1)

if __name__ == '__main__':
    MotorPara=np.zeros([6,T_LIMIT,1])  #ID P V T Kd Kp

    Motorlist=[11]
    #Motor[0]=motor(Motorlist[0])
    ID=np.ones(100)*Motorlist[0]
    Pdes = np.linspace(0, 95.5, 100)
    Vdes = np.ones(100)*20
    tff=np.ones(100)*16
    kd=np.ones(100)
    kp= np.ones(100)

    MotorPara =np.array([ID,Pdes, Vdes,tff,kd,kp])
    #MotorPara=np.concatenate((Pdes, Vdes,tff,kd,kp), axis=0)

    print(MotorPara)






