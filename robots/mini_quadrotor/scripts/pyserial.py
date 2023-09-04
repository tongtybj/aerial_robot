#!/usr/bin/env python

import serial
import struct

import rospy
from std_msgs.msg import Float32


if __name__=="__main__":

    rospy.init_node('acid_voltage', anonymous=True)

    port = rospy.get_param('port', '/dev/ttyUSB0')
    baud = rospy.get_param('baud', 115200)
    ser = serial.Serial(port, baud)

    pub = rospy.Publisher('acid_vol', Float32, queue_size=10)

    v_ref = 2.048

    while not rospy.is_shutdown():

        header1 = ser.read(1)
        if header1 == b'\xff':
            header2 = ser.read(1)
            if header2 == b'\xfe':
                # print("get the header of xff xfe")
                pass
            else:
                # cannot get the correct header
                continue
        else:
            # cannot get the correct header
            continue

        data1 = ser.read(1)
        data2 = ser.read(1)
        bytes = struct.pack('@cc', data2, data1)
        #print(bytes)

        value_int16 = struct.unpack('h', bytes)[0]
        #print(value_int16)

        value_float = value_int16 * v_ref / 32768.0 - 1.233
        #print(value_float)

        msg =Float32(value_float)
        pub.publish(msg)

    ser.close()
