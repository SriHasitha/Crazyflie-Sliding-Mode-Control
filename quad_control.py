#!/usr/bin/env python3
from math import pi, sqrt, atan2, cos, sin, asin, acos
from turtle import position
import numpy as np
from numpy import NaN
import rospy
import tf
from std_msgs.msg import Empty, Float32
from nav_msgs.msg import Odometry
from mav_msgs.msg import Actuators
from geometry_msgs.msg import Twist, Pose2D
import pickle
import os

class Quadrotor():
    def __init__(self):
        # publisher for rotor speeds
        self.motor_speed_pub = rospy.Publisher("/crazyflie2/command/motor_speed", Actuators, queue_size=10)
        # subscribe to Odometry topic
        self.odom_sub = rospy.Subscriber("/crazyflie2/ground_truth/odometry", Odometry, self.odom_callback, queue_size=1)
        rospy.Rate(100)
        self.t0 = None
        self.t = None
        self.t_series = []
        self.x_series = []
        self.y_series = []
        self.z_series = []
        self.mutex_lock_on = False
        rospy.on_shutdown(self.save_data)
        # TODO: include initialization codes if needed
        self.xd = 0
        self.yd = 0
        self.zd = 0
        self.xd_dot = 0
        self.yd_dot = 0
        self.zd_dot = 0
        self.xd_ddot = 0
        self.yd_ddot = 0
        self.zd_ddot = 0
        self.w1 = 0
        self.w2 = 0
        self.w3 = 0
        self.w4 = 0


    def traj_evaluate(self):
        # TODO: evaluating the corresponding trajectories designed in Part 1 to return the desired positions, velocities and accelerations
        if self.t<=5:
            self.xd = 0
            self.yd = 0
            self.zd = (self.t**3*(6*self.t**2-75*self.t +250))/3125
            self.xd_dot = 0
            self.yd_dot = 0
            self.zd_dot = (self.t**3*(12*self.t - 75))/3125 + (3*self.t**2*(6*self.t**2 - 75*self.t + 250))/3125
            self.xd_ddot = 0          
            self.yd_ddot = 0
            self.zd_ddot = (6*self.t*(6*self.t**2 - 75*self.t + 250))/3125 + (6*self.t**2*(12*self.t - 75))/3125 + (12*self.t**3)/3125  
        elif self.t<=20:
            self.xd = (4664065662093477*self.t**5)/590295810358705651712 - self.t**4/2025 + (22*self.t**3)/2025 - (8*self.t**2)/81 + (32*self.t)/81 - 47/81
            self.yd = 0
            self.zd = 1
            self.xd_dot = (23320328310467385*self.t**4)/590295810358705651712 - (4*self.t**3)/2025 + (22*self.t**2)/675 - (16*self.t)/81 + 32/81
            self.yd_dot = 0
            self.zd_dot = 0
            self.xd_ddot = (23320328310467385*self.t**3)/147573952589676412928 - (4*self.t**2)/675 + (44*self.t)/675 - 16/81
            self.yd_ddot = 0
            self.zd_ddot = 0
        elif self.t<=35:
            self.xd = 1
            self.yd = (4664065662093477*self.t**5)/590295810358705651712 - (11*self.t**4)/10125 + (118*self.t**3)/2025 - (616*self.t**2)/405 + (1568*self.t)/81 - 7808/81
            self.zd = 1
            self.xd_dot = 0
            self.yd_dot = (23320328310467385*self.t**4)/590295810358705651712 - (44*self.t**3)/10125 + (118*self.t**2)/675 - (1232*self.t)/405 + 1568/81
            self.zd_dot = 0
            self.xd_ddot = 0
            self.yd_ddot = (23320328310467385*self.t**3)/147573952589676412928 - (44*self.t**2)/3375 + (236*self.t)/675 - 1232/405
            self.zd_ddot = 0
        elif self.t<=50:
            self.xd = - (4664065662093477*self.t**5)/590295810358705651712 + (17*self.t**4)/10125 - (286*self.t**3)/2025 + (476*self.t**2)/81 - (9800*self.t)/81 + 80000/81
            self.yd = 1
            self.zd = 1
            self.xd_dot = - (23320328310467385*self.t**4)/590295810358705651712 + (68*self.t**3)/10125 - (286*self.t**2)/675 + (952*self.t)/81 - 9800/81
            self.yd_dot = 0
            self.zd_dot = 0
            self.xd_ddot = - (23320328310467385*self.t**3)/147573952589676412928 + (68*self.t**2)/3375 - (572*self.t)/675 + 952/81
            self.yd_ddot = 0
            self.zd_ddot = 0
        elif self.t<=65:
            self.xd = 0
            self.yd = - (4664065662093477*self.t**5)/590295810358705651712 + (23*self.t**4)/10125 - (526*self.t**3)/2025 + (1196*self.t**2)/81 - (33800*self.t)/81 + 5159302209836171/1099511627776
            self.zd = 1
            self.xd_dot = 0
            self.yd_dot = - (23320328310467385*self.t**4)/590295810358705651712 + (92*self.t**3)/10125 - (526*self.t**2)/675 + (2392*self.t)/81 - 33800/81
            self.zd_dot = 0
            self.xd_ddot = 0
            self.yd_ddot = - (23320328310467385*self.t**3)/147573952589676412928 + (92*self.t**2)/3375 - (1052*self.t)/675 + 2392/81
            self.zd_ddot = 0

    def wrap2pi(self,e):
        e = np.arctan2(np.sin(e),np.cos(e))
        #e = (e+np.pi)%(2*np.pi)-np.pi
        return e
        
    def smc_control(self, xyz, xyz_dot, rpy, rpy_dot):
        # obtain the desired values by evaluating the corresponding trajectories
        self.traj_evaluate()
        # TODO: implement the Sliding Mode Control laws designed in Part 2 to calculate the control inputs "u"
        m = 27e-3
        l = 46e-3
        Ix = 16.571710e-6
        Iy = 16.571710e-6
        Iz = 29.261652e-6
        Ip = 12.65625e-8
        Kf = 1.28192e-8
        Km = 5.964552e-3
        w_max = 2618
        w_min = 0
        g = 9.81

        #rospy.loginfo(self.xd)
        #rospy.loginfo(self.yd)
        #rospy.loginfo(self.zd)
        #rospy.loginfo_once(self.t)
        #rospy.loginfo_once(xyz)
        #rospy.loginfo_once(xyz_dot)
        #rospy.loginfo_once(rpy)
        #rospy.loginfo_once(rpy_dot)

        omega = self.w1-self.w2+self.w3-self.w4
        #control input 1
        e1 = xyz[2,0]-self.zd
        e1_dot = xyz_dot[2,0]-self.zd_dot
        lambda1 = 8.5
        s1 = e1_dot + lambda1*e1
        bz = 0.9
        satz = min(max(s1/bz, -1), 1)
        rho1 = 0
        K1 = 10
        fz = -g
        gz = (1/m)*(cos(rpy[0,0])*cos(rpy[1,0]))
        ur1 = -(rho1+K1)*(satz)
        u1 = (1/gz)*(-fz + self.zd_ddot-lambda1*e1_dot + ur1)

        kp = 100
        kd = 5
        Fx = m*(-kp*(xyz[0,0]-self.xd)-kd*(xyz_dot[0,0]-self.xd_dot)+self.xd_ddot)
        Fy = m*(-kp*(xyz[1,0]-self.yd)-kd*(xyz_dot[1,0]-self.yd_dot)+self.yd_ddot)
        #rospy.loginfo(Fx)
        #rospy.loginfo(Fy)
        thetad = asin(Fx/u1)
        phid = asin(-Fy/u1)
        
        #rospy.loginfo(phid)

        psid = 0
        phid_dot = 0;phid_ddot = 0
        thetad_dot = 0; thetad_ddot =0
        psid_dot = 0; psid_ddot = 0

        #control input 2
        e2 = self.wrap2pi(rpy[0,0]-phid)
        e2_dot = rpy_dot[0,0]-phid_dot
        lambda2 = 14
        s2 = e2_dot + lambda2*e2
        sat_phi = min(max(s2/bz, -1), 1)
        rho2 = 0
        K2 = 150
        fphi = rpy_dot[1,0]*rpy_dot[2,0]*((Iy-Iz)/Ix) - (Ip/Ix)*omega*rpy_dot[1,0]
        gphi = 1/Ix
        ur2 = -(rho2+K2)*(sat_phi)
        u2 = (1/gphi)*(-fphi + phid_ddot-lambda2*(e2_dot) + ur2)

        #control input 3
        e3 = rpy[1,0]-thetad
        e3 = self.wrap2pi(e3)
        
        #rospy.loginfo('ex: %f',(xyz[0,0]-self.xd))
        #rospy.loginfo('theta: %f',rpy[1,0])
        #rospy.loginfo('thetad: %f',thetad)
        #rospy.loginfo('etheta: %f',e3)

        e3_dot = rpy_dot[1,0]-thetad_dot
        lambda3 = 18.5
        s3 = e3_dot + lambda3*e3
        rho3 = 0
        K3 = 120
        sat_theta = min(max(s3/bz, -1), 1)
        ftheta = rpy_dot[0,0]*rpy_dot[2,0]*((Iz-Ix)/Iy) + (Ip/Iy)*omega*rpy_dot[0,0]
        gtheta = 1/Iy
        ur3 = -(rho3+K3)*(sat_theta)
        u3 = (1/gtheta)*(-ftheta + thetad_ddot-lambda3*(e3_dot) + ur3)
        
        #control input 4
        e4 = rpy[2,0]-psid
        e4 = self.wrap2pi(e4)
        e4_dot = rpy_dot[2,0]-psid_dot
        lambda4 = 5
        s4 = e4_dot + lambda4*e4
        sat_psi = min(max(s4/bz, -1), 1)
        rho4 = 0
        K4 = 25
        fpsi = rpy_dot[0,0]*rpy_dot[1,0]*((Ix-Iy)/Iz)
        gpsi = 1/Iz
        ur4 = -(rho4+K4)*(sat_psi)
        u4 = (1/gpsi)*(-fpsi + psid_ddot-lambda4*(e4_dot) + ur4)

        #rospy.loginfo('u1: %f',u1)
        #rospy.loginfo('u2: %f',u2)
        #rospy.loginfo('u3: %f',u3)
        #rospy.loginfo('u4: %f',u4)

        # REMARK: wrap the roll-pitch-yaw angle errors to [-pi to pi]
        # TODO: convert the desired control inputs "u" to desired rotor velocities "motor_vel" by using the "allocation matrix"
        alloc = np.matrix([[1/(4*Kf), -sqrt(2)/(4*Kf*l), -sqrt(2)/(4*Kf*l), -1/(4*Km*Kf)], [1/(4*Kf), -sqrt(2)/(4*Kf*l), sqrt(2)/(4*Kf*l), 1/(4*Km*Kf)], [1/(4*Kf), sqrt(2)/(4*Kf*l), sqrt(2)/(4*Kf*l), -1/(4*Km*Kf)], [1/(4*Kf), sqrt(2)/(4*Kf*l), -sqrt(2)/(4*Kf*l), 1/(4*Km*Kf)]],dtype=float)
        w = np.sqrt(np.matmul(alloc,np.transpose(np.matrix([u1,u2,u3,u4],dtype=float))))
        for i in range(len(w)):
            if w[i] > w_max:
                w[i] = w_max
            if w[i] < w_min:
                w[i] = w_min
        #rospy.loginfo('w: %f %f %f %f',w[0],w[1],w[2],w[3])
        motor_vel = w
        self.w1 = w[0]
        self.w2 = w[1]
        self.w3 = w[2]
        self.w4 = w[3]

        #rospy.loginfo_once(alloc)
        #rospy.loginfo_once(motor_vel)

        # TODO: maintain the rotor velocities within the valid range of [0 to 2618]
        # publish the motor velocities to the associated ROS topic
        motor_speed = Actuators()
        motor_speed.angular_velocities = [motor_vel[0,0], motor_vel[1,0], motor_vel[2,0], motor_vel[3,0]]
        self.motor_speed_pub.publish(motor_speed)
    
    # odometry callback function (DO NOT MODIFY)
    def odom_callback(self, msg):
        if self.t0 == None:
            self.t0 = msg.header.stamp.to_sec()
        self.t = msg.header.stamp.to_sec() - self.t0
        # convert odometry data to xyz, xyz_dot, rpy, and rpy_dot
        w_b = np.asarray([[msg.twist.twist.angular.x], [msg.twist.twist.angular.y], [msg.twist.twist.angular.z]])
        v_b = np.asarray([[msg.twist.twist.linear.x], [msg.twist.twist.linear.y], [msg.twist.twist.linear.z]])
        xyz = np.asarray([[msg.pose.pose.position.x], [msg.pose.pose.position.y], [msg.pose.pose.position.z]])
        q = msg.pose.pose.orientation
        T = tf.transformations.quaternion_matrix([q.x, q.y, q.z, q.w])
        T[0:3, 3] = xyz[0:3, 0]
        R = T[0:3, 0:3]
        xyz_dot = np.dot(R, v_b)
        rpy = tf.transformations.euler_from_matrix(R, 'sxyz')
        rpy_dot = np.dot(np.asarray([[1, np.sin(rpy[0])*np.tan(rpy[1]), np.cos(rpy[0])*np.tan(rpy[1])], [0, np.cos(rpy[0]), -np.sin(rpy[0])], [0, np.sin(rpy[0])/np.cos(rpy[1]), np.cos(rpy[0])/np.cos(rpy[1])]]), w_b)
        rpy = np.expand_dims(rpy, axis=1)
        # store the actual trajectory to be visualized later
        if (self.mutex_lock_on is not True):
            self.t_series.append(self.t)
            self.x_series.append(xyz[0, 0])
            self.y_series.append(xyz[1, 0])
            self.z_series.append(xyz[2, 0])
        # call the controller with the current states
        self.smc_control(xyz, xyz_dot, rpy, rpy_dot)
    # save the actual trajectory data
    def save_data(self):
        # TODO: update the path below with the correct path
        with open("log.pkl","wb") as fp:
            self.mutex_lock_on = True
            pickle.dump([self.t_series,self.x_series,self.y_series,self.z_series], fp)
if __name__ == '__main__':
    rospy.init_node("quadrotor_control")
    rospy.loginfo("Press Ctrl + C to terminate")
    whatever = Quadrotor()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        rospy.loginfo("Shutting down")
