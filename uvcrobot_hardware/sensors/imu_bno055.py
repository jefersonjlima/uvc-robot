import logging
import sys
import time

import rospy
import math

from std_msgs.msg import Header, Float32
from sensor_msgs.msg import Imu, Temperature

import nav_msgs.msg
from nav_msgs.msg import Odometry
# for the tf broadcaster
import tf
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3

# sleep time for this node in seconds
sleepTime = 0.25

# initialise the ROS node
rospy.init_node('bno055_node', anonymous=False)

# Euler topics
pubH = rospy.Publisher('heading', Float32, queue_size=1)
pubR = rospy.Publisher('roll',    Float32, queue_size=1)
pubP = rospy.Publisher('pitch',   Float32, queue_size=1)
# Temperature
pubTemp = rospy.Publisher('temperature',   Temperature, queue_size=1)
# ROS IMU format messages
pubImu  = rospy.Publisher('imu/data', Imu, queue_size=1)

# for IMU readings
heading = 0.0
roll    = 0.0
pitch   = 0.0
sys     = 0.0
gyro    = 0.0
accel   = 0.0
mag     = 0.0
# Temperature in degrees Celsius
temp = 0.0
# Gyroscope data (in degrees per second):
xg = 0.0
yg = 0.0
zg = 0.0
# Accelerometer data (in meters per second squared)
xa = 0.0
ya = 0.0
za = 0.0
# the robot starts at the origin of the "odom" coordinate frame initially
# this will become a quaternion
x = 0.0
y = 0.0
z = 0.0
w = 0.0

# velocities - where to get?
vx  = 0.0
vy  = 0.0
vth = 0.0


# header frame for odometry message
seq = 0


# define temperature message
temp_msg = Temperature()
# ignore the covariance data, it is optional
temp_msg.variance = 0

# define IMU message
imu_msg = Imu()
# ignore the covariance data, it is optional
imu_msg.orientation_covariance[0]         = -1
imu_msg.angular_velocity_covariance[0]    = -1
imu_msg.linear_acceleration_covariance[0] = -1

from Adafruit_BNO055 import BNO055

# Create and configure the BNO sensor connection.
# Using I2C without a RST pin
bno = BNO055.BNO055()

# Initialize the BNO055 and stop if something went wrong.
if not bno.begin():
    rospy.logerr("Failed to initialize BNO055! Is the sensor connected?")

# Print system status
status, self_test, error = bno.get_system_status()
rospy.loginfo('System status:      0x{0:02X} '.format(status))

# Print self test
rospy.loginfo('Self test result:   0x{0:02X}'.format(self_test))
if self_test != 0x0F:
    rospy.logwarn('WARNING: Self test result is 0x{0:02X} instead if 0x0F!'.format(self_test))

# Print out an error if system status is in error mode.
if status == 0x01:
    rospy.logerr('System error:    {0}'.format(error))
    rospy.loginfo('See datasheet section 4.3.59 for the meaning.')

# Print BNO055 software revision and other diagnostic data.
sw, bl, accel, mag, gyro = bno.get_revision()
rospy.loginfo('Software version:   {0}'.format(sw))
rospy.loginfo('Bootloader version: {0}'.format(bl))
rospy.loginfo('Accelerometer ID:   0x{0:02X}'.format(accel))
rospy.loginfo('Magnetometer ID:    0x{0:02X}'.format(mag))
rospy.loginfo('Gyroscope ID:       0x{0:02X}\n'.format(gyro))


rospy.loginfo('Reading BNO055 data, press Ctrl-C to quit...')

while not rospy.is_shutdown():
    # for header time stamps
    current_time = rospy.Time.now()


    """ IMU message header (for IMU and temperature) """
    h = rospy.Header()
    h.stamp = current_time
    h.frame_id = 'imu_link' # @sa: tf_broadcaster.py
    h.seq = seq
    # increase sequence
    seq = seq + 1
    # add header to IMU message
    imu_msg.header = h


    """ IMU readings """
    # run some parts only on the real robot
    heading, roll, pitch = bno.read_euler()
    # Print everything out.
    # rospy.loginfo('Heading={0:0.2F} Roll={1:0.2F} Pitch={2:0.2F}\tSys_cal={3} Gyro_cal={4} Accel_cal={5} Mag_cal={6}'.format(heading, roll, pitch, sys, gyro, accel, mag))

    # publish Euler values
    pubH.publish(heading)
    pubR.publish(roll)
    pubP.publish(pitch)


    # run some parts only on the real robot
    x,y,z,w = bno.read_quaternion()
    imu_msg.orientation.x = x # is this the pose then?
    imu_msg.orientation.y = y
    imu_msg.orientation.z = z
    imu_msg.orientation.w = w
    # Print
    # rospy.loginfo('Quaternion: x={} y={} z={} w={}'.format(imu_msg.orientation.x, imu_msg.orientation.y, imu_msg.orientation.z, imu_msg.orientation.w))

    # run some parts only on the real robot
    xg,yg,zg = bno.read_gyroscope()

    imu_msg.angular_velocity.x = xg;
    imu_msg.angular_velocity.y = yg;
    imu_msg.angular_velocity.z = zg;
    # Print
    # rospy.loginfo('Gyroscope: x={} y={} z={}'.format(imu_msg.angular_velocity.x, imu_msg.angular_velocity.y, imu_msg.angular_velocity.z))



    imu_msg.linear_acceleration.x = xa;
    imu_msg.linear_acceleration.y = ya;
    imu_msg.linear_acceleration.z = za;
    # Print
    # rospy.loginfo('Accelerometer: x={} y={} z={}'.format(imu_msg.linear_acceleration.x, imu_msg.linear_acceleration.y, imu_msg.linear_acceleration.z))

    #
    # publish message
    #
    pubImu.publish(imu_msg)


    """ Temperature in degrees Celsius """
    # run some parts only on the real robot
    temp = bno.read_temp()
    # Print
    # rospy.loginfo('Temperature: {}°C'.format(temp))

    # add header to temperature message
    temp_msg.header = h

    #
    # publish message
    #
    temp_msg.temperature = temp
    pubTemp.publish(temp_msg)

