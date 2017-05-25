import struct

evFmt = 'IHBB'
evSize = struct.calcsize(evFmt)
fp = open('/dev/input/js0', 'rb')

while True:
    data = fp.read(evSize)
    time, value, typ, number = struct.unpack(evFmt, data)
    print 'time:%s, value:%s, type:%s, number:%s'%(time, value, typ, number)





