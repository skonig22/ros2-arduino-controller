import rclpy
from rclpy.node import Node

import serial
from geometry_msgs.msg import Twist
import numpy as np
from std_msgs.msg import Int16

class L298NController(Node):

    def __init__(self):
        super().__init__('L298N_controller')
        self.left_enc_pub = self.create_publisher(Int16, 'left_enc_ticks', 10)
        self.right_enc_pub = self.create_publisher(Int16, 'right_enc_ticks', 10)

        self.vel_sub = self.create_subscription(
            Twist,
            'cmd_vel',
            self.vel_callback,
            10)
        self.vel_sub

        self.arduino = serial.Serial(port='/dev/ttyACM0', baudrate=9600, timeout=.1)
        self.WHEEL_DIST = 1 # in m
        self.WHEEL_RADIUS = .4 # in m
        self.L_enc_val = 0
        self.R_enc_val = 0


    def vel_callback(self, msg):
        L_pwm,R_pwm = vel2pwm(msg.linear.x, msg.angular.w)
        pwm_vals = " "+str(L_pwm)+"   "+str(R_pwm) # create string to be sent to arduino

        self.arduino.write(bytes(pwm_vals, 'utf-8')) # send PWM values to arduino

        enc_count_str = self.arduino.readline() # printing the value
        enc_count_data = enc_count_str.decode() # convert from bytes to string

        ch = ':'
        if ch in enc_count_data:
            enc_vals = enc_count_data.split(":") #split encoder values from arduino so values can be assigned
            self.L_enc_val = enc_vals[0]
            self.R_enc_val = enc_vals[1]

            L_enc_msg = Float64()
            L_enc_msg.data = self.L_enc_val
            self.left_enc_pub.publish(L_enc_msg)

            R_enc_msg = Float64()
            R_enc_msg.data = self.R_enc_val
            self.right_enc_pub.publish(R_enc_msg)
    
    def vel2pwm(self, goal_vel, goal_omega):
        M1_omega = (goal_vel + goal_omega*self.WHEEL_DIST/2)/self.WHEEL_RADIUS
        M2_omega = (goal_vel - goal_omega*self.WHEEL_DIST/2)/self.WHEEL_RADIUS

        M1_rpm = M1_omega*60/(2*np.pi)
        M2_rpm = M2_omega*60/(2*np.pi)

        M1_pwm = 3.46*M1_rpm - 4.83
        M2_pwm = 3.46*M2_rpm - 4.83

        return M1_pwm, M2_pwm

def main(args=None):
    rclpy.init(args=args)

    L298N_controller = L298NController()

    rclpy.spin(L298N_controller)

    L298N_controller.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
 