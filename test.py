#!/usr/bin/env python
#encoding: utf8

import sys
#reload(sys)
#sys.setdefaultencoding("utf-8")

import struct
a='0x3e8'
c=int(a,16)


a = 11
b = 40
c=20
d=12
e = b'\xF0\xFF\xFF\xFF\xFF\xFF\xFF\xFC'
# pack

str = struct.pack(">i",b)
print (repr(str))
stru = struct.unpack(">i", str)


str1 = struct.pack(">h", c<<4)
print (repr(str1))

str2 = struct.pack(">h", d)
print (repr(str2))

#str3=str1[:6]+str1[3:] 用移位
print (repr(str3))

print('length: ', len(str) )          # length:  8
#print (str)                           # 乱码：
print (repr(str))                     # '\x14\x00\x00\x00\x90\x01\x00\x00'
str1 = struct.pack(">ii", a,-4)
print (repr(str1))
# unpack
str2 = struct.unpack("ii", str)
print('length: ', len(str2))         # length:  2
print(str2)                          # (20, 400)
print(repr(str2))                    # (20, 400)