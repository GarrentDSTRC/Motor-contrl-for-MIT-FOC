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

POSITION_ORDER= b"\x02\x02\x0B\x04\x9C\x7E\x03"
SETZERO_UART=b'\x02\x02\x5F\x01\x0E\xA0\x03'

crc16_tab = [ 0x0000, 0x1021, 0x2042, 0x3063, 0x4084,
        0x50a5, 0x60c6, 0x70e7, 0x8108, 0x9129, 0xa14a, 0xb16b, 0xc18c, 0xd1ad,
        0xe1ce, 0xf1ef, 0x1231, 0x0210, 0x3273, 0x2252, 0x52b5, 0x4294, 0x72f7,
        0x62d6, 0x9339, 0x8318, 0xb37b, 0xa35a, 0xd3bd, 0xc39c, 0xf3ff, 0xe3de,
        0x2462, 0x3443, 0x0420, 0x1401, 0x64e6, 0x74c7, 0x44a4, 0x5485, 0xa56a,
        0xb54b, 0x8528, 0x9509, 0xe5ee, 0xf5cf, 0xc5ac, 0xd58d, 0x3653, 0x2672,
        0x1611, 0x0630, 0x76d7, 0x66f6, 0x5695, 0x46b4, 0xb75b, 0xa77a, 0x9719,
        0x8738, 0xf7df, 0xe7fe, 0xd79d, 0xc7bc, 0x48c4, 0x58e5, 0x6886, 0x78a7,
        0x0840, 0x1861, 0x2802, 0x3823, 0xc9cc, 0xd9ed, 0xe98e, 0xf9af, 0x8948,
        0x9969, 0xa90a, 0xb92b, 0x5af5, 0x4ad4, 0x7ab7, 0x6a96, 0x1a71, 0x0a50,
        0x3a33, 0x2a12, 0xdbfd, 0xcbdc, 0xfbbf, 0xeb9e, 0x9b79, 0x8b58, 0xbb3b,
        0xab1a, 0x6ca6, 0x7c87, 0x4ce4, 0x5cc5, 0x2c22, 0x3c03, 0x0c60, 0x1c41,
        0xedae, 0xfd8f, 0xcdec, 0xddcd, 0xad2a, 0xbd0b, 0x8d68, 0x9d49, 0x7e97,
        0x6eb6, 0x5ed5, 0x4ef4, 0x3e13, 0x2e32, 0x1e51, 0x0e70, 0xff9f, 0xefbe,
        0xdfdd, 0xcffc, 0xbf1b, 0xaf3a, 0x9f59, 0x8f78, 0x9188, 0x81a9, 0xb1ca,
        0xa1eb, 0xd10c, 0xc12d, 0xf14e, 0xe16f, 0x1080, 0x00a1, 0x30c2, 0x20e3,
        0x5004, 0x4025, 0x7046, 0x6067, 0x83b9, 0x9398, 0xa3fb, 0xb3da, 0xc33d,
        0xd31c, 0xe37f, 0xf35e, 0x02b1, 0x1290, 0x22f3, 0x32d2, 0x4235, 0x5214,
        0x6277, 0x7256, 0xb5ea, 0xa5cb, 0x95a8, 0x8589, 0xf56e, 0xe54f, 0xd52c,
        0xc50d, 0x34e2, 0x24c3, 0x14a0, 0x0481, 0x7466, 0x6447, 0x5424, 0x4405,
        0xa7db, 0xb7fa, 0x8799, 0x97b8, 0xe75f, 0xf77e, 0xc71d, 0xd73c, 0x26d3,
        0x36f2, 0x0691, 0x16b0, 0x6657, 0x7676, 0x4615, 0x5634, 0xd94c, 0xc96d,
        0xf90e, 0xe92f, 0x99c8, 0x89e9, 0xb98a, 0xa9ab, 0x5844, 0x4865, 0x7806,
        0x6827, 0x18c0, 0x08e1, 0x3882, 0x28a3, 0xcb7d, 0xdb5c, 0xeb3f, 0xfb1e,
        0x8bf9, 0x9bd8, 0xabbb, 0xbb9a, 0x4a75, 0x5a54, 0x6a37, 0x7a16, 0x0af1,
        0x1ad0, 0x2ab3, 0x3a92, 0xfd2e, 0xed0f, 0xdd6c, 0xcd4d, 0xbdaa, 0xad8b,
        0x9de8, 0x8dc9, 0x7c26, 0x6c07, 0x5c64, 0x4c45, 0x3ca2, 0x2c83, 0x1ce0,
        0x0cc1, 0xef1f, 0xff3e, 0xcf5d, 0xdf7c, 0xaf9b, 0xbfba, 0x8fd9, 0x9ff8,
        0x6e17, 0x7e36, 0x4e55, 0x5e74, 0x2e93, 0x3eb2, 0x0ed1, 0x1ef0 ]

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

        self.data=[]
        self.crc=[]

    def motor_enable(self):
        try:
            self.serial_uart.write(struct.pack(">ii", -4,self.enable))
        except:
            print('cant write motor_enable')

    def motor_disable(self):
            self.serial_uart.write(struct.pack(">ii", -4,self.disable))

    def motor_setzero(self):
        try:
            #self.serial_can.write(struct.pack(">ii", -4,self.setzero))
            self.serial_uart.write(SETZERO_UART)
        except:
            print('cant write motor_setzero')

    def motor_sent(self,mode,MotorPara):
        #多线程
        threads = []
        t1 = threading.Thread(target=self.motor_read)
        threads.append(t1)

        if mode=='P' :
            t2 = threading.Thread(target=self.motor_Pctrl_can,args=(MotorPara))
        elif  mode=='V' :
            t2 = threading.Thread(target=self.motor_sent)
        elif mode=='T':
            t2 = threading.Thread(target=self.motor_sent)
        else:
            print("motor_sent wrong")

        threads.append(t2)
        for t in threads:
            t.start()

    def motor_Vctrl(self, MotorPara):
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

            Vdes[i]=Vdes[i]*60/(2*np.pi)
            report=bytearray(b'\x02\x05')
            vdata=struct.pack(">i",int(Vdes[i]))

            adata=b'\x08' + vdata
            split=struct.unpack(">ccccc",adata)

            #cksum = hex(self.crc16(split, 5))
            cksum=self.crc16(split,len(vdata)+1)
            report=report+adata+struct.pack('>H',cksum)+b'\x03'

            self.serial_uart.write(report)
            time.sleep(0.1)


    def motor_Pctrl_can(self,MotorPara):
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
                begin = self.serial_uart.read(1)  # 读取n位数据
                if begin=='02':
                    datalen = self.serial_uart.read(1)
                    data=self.serial_uart.read(int(datalen))
                    CRC16=self.serial_uart.read(2)
                    end=self.serial_uart.read(1)
                    if end=='03':
                        self.data.append(data)
                        self.crc.append(CRC16)
                    else: print('read data error')

                """
                s = [hex(x) for x in bytes(n)]
                id=(int(s[0],16)<<4)+(int(s[1],16))
                p=(int(s[2],16)<<4)+(int(s[3],16))
                v=(int(s[4],16)<<12)+(int(s[5],16)<<8)+(int(s[6],16)<<4)+(int(s[7],16))
                t=(int(s[8],16)<<8)+(int(s[9],16)<<4)+(int(s[10],16))

                p=self.uint_to_float(p,P_MIN,P_MAX,16)
                v = self.uint_to_float(v, V_MIN, V_MAX, 12)
                t = self.uint_to_float(t, T_MIN, T_MAX, 12)

                print("P:",p,"\nV:",v,"\nT:",t)
                """
            time.sleep(0.1)

    def crc16(self,buf, len):
        cksum = 0
        j=0
        for i in range(len) :
            cksum = crc16_tab[(((cksum >> 8) ^ int.from_bytes(buf[j], 'big', signed=True)  ) & 0xFF)] ^ (cksum << 8)
            #cksum = crc16_tab[(((cksum >> 8) ^ int.from_bytes(buf, 'big', signed=True)) & 0xFF)] ^ (cksum << 8)
            j+=1
        return cksum & 0xFFFF

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
    Vdes = np.ones(100)*20
    tff=np.ones(100)*0
    kd=np.ones(100)*0
    kp= np.ones(100)

    MotorPara =np.array([ID,Pdes, Vdes,tff,kd,kp])
    MotorPara=np.expand_dims(MotorPara, axis=2)
    #MyMotor[0].serial_uart.write(POSITION_ORDER)1

    MyMotor[0].motor_Vctrl(MotorPara)

    print(MyMotor[0].data,'\n',MyMotor[0].crc)






