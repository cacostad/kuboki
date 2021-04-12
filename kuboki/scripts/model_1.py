#!/usr/bin/python

import rospy

from geometry_msgs.msg import Twist
from std_msgs.msg import Float64

import numpy as np
from rospy.numpy_msg import numpy_msg

class velocity_publisher:

    def __init__(self):
        # Parametros

        r = 0.035
       
        # Left Wheel

        alpha_left = np.pi/2
        betha_left = 0.0
        # l_left = 0.15
        l_left = 0.177

        # Rigth Wheel

        alpha_rigth = -np.pi/2
        betha_rigth = np.pi
        # l_left = 0.15
        l_rigth = 0.177

        J1 = np.array([( np.sin(alpha_left+betha_left), -np.cos(alpha_left+betha_left) , -l_left*np.cos(betha_left)),
                       ( np.sin(alpha_rigth+betha_rigth), -np.cos(alpha_rigth+betha_rigth) , -l_left*np.cos(betha_rigth))])

        J2 = r*np.identity(2)

        self.Jacobiano = np.matmul(np.linalg.pinv(J2),J1)
        
        print("J1")
        print(J1)

        print("J2")
        print(J2)

        # Suscriber

        self.cmd_vel_subscriptor = rospy.Subscriber('/cmd_vel',Twist, self.cmd_vel_cb,queue_size = 10)

        # Publisher

        
        self.pub_wheel_left = rospy.Publisher("/left_wheel_ctrl/command",Float64,queue_size=10)
        self.pub_wheel_rigth = rospy.Publisher("/right_wheel_ctrl/command",Float64,queue_size=10)


        # Polling

        while not rospy.is_shutdown():

            rospy.loginfo("Waiting")

            # Keyboard Command
            command_char = raw_input()
            rospy.loginfo("received:"+command_char)

            # Q

            if(command_char == 'q'):
                rospy.loginfo("Quit")
                rospy.signal_shutdown("Request shutdown")

            # s
            if(command_char == 's'):
                rospy.loginfo("Stop")

                msf_Float = Float64()
                msf_Float = 0.0

                
                self.pub_wheel_left.publish(msf_Float)
                self.pub_wheel_rigth.publish(msf_Float)

            # y
            if(command_char == 'y'):
                rospy.loginfo("up")

                msf_Float = Float64()
                msf_Float = msgFloat.data + 0.05

                
                self.pub_wheel_left.publish(msf_Float)
                self.pub_wheel_rigth.publish(msf_Float)
            
            # n
            if(command_char == 'n'):
                rospy.loginfo("back")

                msf_Float = Float64()
                msf_Float = msgFloat.data - 0.05

                
                self.pub_wheel_left.publish(msf_Float)
                self.pub_wheel_rigth.publish(msf_Float)

            # g
            if(command_char == 'g'):
                rospy.loginfo("left")

                msf_Float = Float64()
                msf_Float = msgFloat.data + 0.05

                
                self.pub_wheel_rigth.publish(msf_Float)
            
            # j
            if(command_char == 'j'):
                rospy.loginfo("right")

                msf_Float = Float64()
                msf_Float = msgFloat.data + 0.05

                
                self.pub_wheel_left.publish(msf_Float)



    def cmd_vel_cb(self,cmd_vel):

        if cmd_vel.linear.x > 1:
            cmd_vel.linear.x = 1
        if cmd_vel.linear.y > 1:
            cmd_vel.linear.y = 1
        if cmd_vel.angular.z > 180:
            cmd_vel.linear.z = 180



        command = np.array([0,0,0], dtype=np.float)

        command[0] = cmd_vel.linear.x
        command[1] = cmd_vel.linear.y
        command[2] = cmd_vel.angular.z

        print("command:", command)

        result = np.matmul(self.Jacobiano,command)

        print("Result:", result)

        # Send Velocity to wheels
        
        # left
        msgFloat = Float64()
        msgFloat.data = result[0]
        self.pub_wheel_left.publish(msgFloat)

        # Right
        msgFloat = Float64()
        msgFloat.data = result[1]
        self.pub_wheel_rigth.publish(msgFloat)