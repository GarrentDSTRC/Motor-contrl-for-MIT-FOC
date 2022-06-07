import numpy as np
import time
import struct
import serial
import threading

T_LIMIT=100

P_MIN =-95.5
P_MAX =95.5
V_MIN =-30
V_MAX =30
T_MIN =-18
T_MAX =18
Kp_MIN=0
Kp_MAX =500
Kd_MIN =0
Kd_MAX =5
Test_Pos=6.0

class motor():

    def __init__(self, motor_id, com1='com4',com2='com4', bps1=115200,bps2=115200):
        self.id=motor_id

        self.disable = -4
        self.enable = -3
        self.setzero= -2

        # 设置串口通讯
        '''
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
        '''
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

        #limit data to be within bounds
        sent=[]
        for i in range(T_LIMIT):
            Pdes[i]=np.min([ np.max([Pdes[i],P_MIN]),P_MAX])
            Vdes[i] = np.min([ np.max([Vdes[i], V_MIN]), V_MAX])
            tff[i] = np.min([np.max([tff[i], T_MIN]), T_MAX])
            kd[i] = np.min([np.max([kd[i], Kd_MIN]), Kd_MAX])
            kp[i] = np.min([np.max([kp[i], Kp_MIN]), Kp_MAX])

            Pdes[i]=self.float_to_uint(Pdes[i],P_MIN,P_MAX,16)
            Vdes[i] = self.float_to_uint(Vdes[i], V_MIN, V_MAX, 12)
            tff[i] = self.float_to_uint(tff[i], T_MIN, T_MAX, 12)
            kd[i] = self.float_to_uint(kd[i], Kd_MIN, Kd_MAX, 12)
            kp[i] = self.float_to_uint(kp[i], Kp_MIN, Kp_MAX, 12)

            report=(int(Pdes[i])<<48)+(int(Vdes[i])<<36)+(int(kp[i])<<24)+(int(kd[i])<<12)+int(tff[i])
            sent.append(report)
            rbi=struct.pack(">Q", report)

            self.serial_can.write(rbi)

            time.sleep(0.1)

    def  float_to_uint(self, x,  x_min,  x_max, bits):
        span = x_max - x_min
        if(x < x_min):
            x = x_min
        elif(x > x_max) :
            x = x_max
        p=int( (x- x_min)*float((1<<bits)-1)/span  )
        #print(p)
        return p
    def uint_to_float(x_int, x_min,x_max,bits):
        span = x_max - x_min
        return  float(x_int)*span/float((1<<bits)-1) + x_min

    def motor_read(self):
        while True:
            n = self.serial_uart.inWaiting()  # 等待数据的到来，并得到数据的长度
            if n:  # 如果有数据
                n = self.serial_uart.read(8)  # 读取n位数据
                s = [hex(x) for x in bytes(n)]
                id=(int(s[0],16)<<4)+(int(s[1],16))
                p=(int(s[2],16)<<4)+(int(s[3],16))
                v=(int(s[4],16)<<12)+(int(s[5],16)<<8)+(int(s[6],16)<<4)+(int(s[7],16))
                t=(int(s[8],16)<<8)+(int(s[9],16)<<4)+(int(s[10],16))

                p=self.uint_to_float(p,P_MIN,P_MAX,16)
                v = self.uint_to_float(v, V_MIN, V_MAX, 12)
                t = self.uint_to_float(t, T_MIN, T_MAX, 12)

                print("P:",p,"\nV:",v,"\nT:",t)
            time.sleep(0.1)

if __name__ == '__main__':
    MotorPara=np.zeros([6,T_LIMIT,1])  #ID P V T Kd Kp

    def  ft_to_u( x,  x_min,  x_max, bits):
        span = x_max - x_min
        if(x < x_min):
            x = x_min
        elif(x > x_max) :
            x = x_max
        return int( (x- x_min)*float((1<<bits)-1)/span  )
    def u_to_f(x_int, x_min,x_max,bits):
        span = x_max - x_min
        return  float(x_int)*span/float((1<<bits)-1) + x_min
    A=ft_to_u(28.2,P_MIN,P_MAX,16)
    B=u_to_f(A,P_MIN,P_MAX,16)

    Motorlist=[11,12]

    MyMotor=[]
    MyMotor.append(motor(Motorlist[0]))

    ID=np.ones(100)*Motorlist[0]
    Pdes = np.linspace(0, 95.5, 100)
    Vdes = np.ones(100)*0
    tff=np.ones(100)*0
    kd=np.ones(100)*0
    kp= np.ones(100)

    MotorPara =np.array([ID,Pdes, Vdes,tff,kd,kp])
    MotorPara=np.expand_dims(MotorPara, axis=2)
    MyMotor[0].motor_Pctrl(MotorPara)

    print(MotorPara)






