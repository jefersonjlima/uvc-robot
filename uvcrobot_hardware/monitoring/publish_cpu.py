#!/usr/bin/env python
import rospy
from std_msgs.msg import Float64
import numpy as np
if __name__ == '__main__':
    rospy.init_node('cpu_temp_publisher')
    pub = rospy.Publisher('raspberry/Temperature', Float64, queue_size = 10)
    rate = rospy.Rate(1)

    cpu_temp = Float64()
    while not rospy.is_shutdown():
        with open("/sys/class/thermal/thermal_zone0/temp", "r") as f:
            cpu_temp.data = np.float64(f.readline())/1000.0
        pub.publish(cpu_temp)
        rate.sleep()

