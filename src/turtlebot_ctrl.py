#!/usr/bin/env python
#encoding=utf-8

import rospy
from geometry_msgs.msg import Twist, Point, PoseStamped, Quaternion
from nav_msgs.msg import Odometry, Path
import tf
from math import radians, copysign, sqrt, pow, pi, atan2, fabs, sin, cos
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from tf import TransformBroadcaster
import numpy as np

def sgn(x):
    if x > 0:
        return 1.
    elif x < 0:
        return -1.
    else:
        return 0.

def sat(x, ep):         # ep should > 0
    if fabs(x) < ep:
        return x / ep
    elif x >= ep:
        return 1.
    else:
        return -1.

def data_saturation(x, upper, lower):
    if x >= upper:
        return upper
    elif x <= lower:
        return lower
    else:
        return x

def phi_t(t):
    K = 0.25
    return K * t

def psi_t(t):
    K = 0.5
    return K * t

bot_rate = 10.       # hz
ros_rate = 50.

class turtlebot(object):
    def __init__(self, name):
        self.name = name

        self.twist_pub = rospy.Publisher('/' + self.name + '/cmd_vel', Twist, queue_size = 1)
        self.pose_sub  = rospy.Subscriber('/' + self.name + '/odom', Odometry, self.odom_cb)

        self.robot_traj_pub = rospy.Publisher('/' + self.name + '/path',     Path, queue_size = 1)
        self.ref_traj_pub   = rospy.Publisher('/' + self.name + '/ref_path', Path, queue_size = 1)
        self.robot_traj = Path()
        self.ref_traj   = Path()

        self.x = 0
        self.y = 0
        self.yaw = 0            # in degree

        self.xr = 0
        self.yr = 0
        self.theta_r = 0        # in rads

        self.vx = 0
        self.wz = 0

        self.Kx_norminal = 1        # 5
        self.Ky_norminal = 0.25     # 1.25
        self.Kt_norminal = 0.015    # 0.33

        self.Kx_adaptive = 1        # 5
        self.Ky_adaptive = 0.25     # 1.25
        self.Kt_adaptive = 0.015    # 0.33
        self.v_hat = 0
        self.w_hat = 0
        self.v_hat_max = 0.15        # integration limit
        self.w_hat_max = 0.25

        self.Kx_robust = 1          # 5
        self.Ky_robust = 0.25       # 1.25
        self.Kt_robust = 0.015      # 0.33

        self.uv = 0
        self.uw = 0

    def odom_cb(self, data):
        # about 10Hz
        (roll, pitch, yaw) = euler_from_quaternion([data.pose.pose.orientation.x, 
                                                    data.pose.pose.orientation.y, 
                                                    data.pose.pose.orientation.z, 
                                                    data.pose.pose.orientation.w])
        self.yaw = yaw * 180. / pi
        self.x   = data.pose.pose.position.x
        self.y   = data.pose.pose.position.y


        self.update_target_vel()
        self.publish_path_4_rviz()

    def update_target_vel(self):
        move_cmd = Twist()
        move_cmd.linear.x = self.vx
        move_cmd.angular.z =self.wz
        self.twist_pub.publish(move_cmd)

    def update_err_dynamics(self, vr, wr, dt = ros_rate):
        self.xr += vr * cos(self.theta_r) * dt
        self.yr += vr * sin(self.theta_r) * dt
        self.theta_r += wr * dt

        self.xe      =  cos(self.yaw * pi / 180.) * (self.xr - self.x) + sin(self.yaw * pi / 180.) * (self.yr - self.y)
        self.ye      = -sin(self.yaw * pi / 180.) * (self.xr - self.x) + cos(self.yaw * pi / 180.) * (self.yr - self.y)
        self.theta_e = ae1 = self.theta_r - self.yaw * pi / 180.

    def control_norminal(self, vr, wr):
        # 1991
        #self.uv = vr * cos(self.theta_e) + self.Kx_norminal * self.xe
        #self.uw = wr + vr * (self.Ky_norminal * self.ye + self.Kt_norminal * sin(self.theta_e))

        # Yu2015 Non-constraint
        self.uv = vr + self.Kx_norminal * self.xe
        self.uw = wr + self.Ky_norminal * vr * (self.ye * cos(self.theta_e / 2) - self.xe * sin(self.theta_e / 2)) + self.Kt_norminal * sin(self.theta_e / 2)

        self.vx = self.uv
        self.wz = self.uw

    def control_adaptive(self, vr, wr, dt = ros_rate):
        # 1991
        #self.v_hat += -self.xe * dt
        #self.v_hat =  data_saturation(self.v_hat, self.v_hat_max, -self.v_hat_max)

        #self.w_hat += -sin(self.theta_e) / self.Ky_adaptive * dt
        #self.w_hat =  data_saturation(self.w_hat, self.w_hat_max, -self.w_hat_max)        
        
        #self.uv = vr * cos(self.theta_e) + self.Kx_adaptive * self.xe - self.v_hat
        #self.uw = wr + vr * (self.Ky_adaptive * self.ye + self.Kt_adaptive * sin(self.theta_e)) - self.w_hat

        # Yu2015 Non-constraint
        self.v_hat += -self.xe * dt
        self.v_hat =  data_saturation(self.v_hat, self.v_hat_max, -self.v_hat_max)

        self.w_hat += -2 * sin(self.theta_e / 2) * dt
        self.w_hat =  data_saturation(self.w_hat, self.w_hat_max, -self.w_hat_max)

        self.uv = vr + self.Kx_adaptive * self.xe - self.v_hat
        self.uw = wr + self.Ky_adaptive * vr * (self.ye * cos(self.theta_e / 2) - self.xe * sin(self.theta_e / 2)) + self.Kt_adaptive * sin(self.theta_e / 2) - self.w_hat

        self.vx = self.uv
        self.wz = self.uw

    def control_robust(self, vr, wr, t):
        #self.uv = vr * cos(self.theta_e) + self.Kx_robust * self.xe + phi_t(t) * sat(self.xe, 0.25)
        #self.uw = wr + vr * (self.Ky_robust * self.ye + self.Kt_robust * sin(self.theta_e)) + psi_t(t) * sat(sin(self.theta_e), 0.5)
        
        # 1991
        #self.uv = vr * cos(self.theta_e) + self.Kx_robust * self.xe + 0.15 * sat(self.xe, 0.25)
        #self.uw = wr + vr * (self.Ky_robust * self.ye + self.Kt_robust * sin(self.theta_e)) + 0.25 * sat(sin(self.theta_e), 0.5)

        # Yu2015 Non-constraint
        self.uv = vr + self.Kx_robust * self.xe - self.v_hat + 0.05 * sat(self.xe, 0.25)
        self.uw = wr + self.Ky_robust * vr * (self.ye * cos(self.theta_e / 2) - self.xe * sin(self.theta_e / 2)) + self.Kt_robust * sin(self.theta_e / 2) + 0.015 * sat(sin(self.theta_e / 2), 0.5)


        self.vx = self.uv
        self.wz = self.uw
   
    def publish_path_4_rviz(self):
            bot_pose_t = PoseStamped()

            bot_pose_t.header.stamp = rospy.Time.now()
            bot_pose_t.header.frame_id = '/' + self.name + '/odom'
            bot_pose_t.pose.position.x = self.x
            bot_pose_t.pose.position.y = self.y
            bot_pose_t.pose.position.z = 0
            [bot_pose_t.pose.orientation.x, bot_pose_t.pose.orientation.y, bot_pose_t.pose.orientation.z, bot_pose_t.pose.orientation.w] = quaternion_from_euler(0., 0., self.yaw * pi / 180.)

            self.robot_traj.poses.append(bot_pose_t)
            self.robot_traj.header.stamp = rospy.Time.now()            
            self.robot_traj.header.frame_id = '/' + self.name + '/odom'
            self.robot_traj_pub.publish(self.robot_traj)

            ref_pose_t = PoseStamped()
            ref_pose_t.header.frame_id = '/' + self.name + '/odom'
            ref_pose_t.pose.position.x = self.xr
            ref_pose_t.pose.position.y = self.yr
            ref_pose_t.pose.position.z = 0
            [ref_pose_t.pose.orientation.x, ref_pose_t.pose.orientation.y, ref_pose_t.pose.orientation.z, ref_pose_t.pose.orientation.w] = quaternion_from_euler(0., 0., self.theta_r)

            self.ref_traj.poses.append(ref_pose_t)
            self.ref_traj.header.stamp = rospy.Time.now()            
            self.ref_traj.header.frame_id = '/' + self.name + '/odom'
            self.ref_traj_pub.publish(self.ref_traj)

def main():
    rospy.init_node('turtlebot_ctrl', anonymous=False)

    rate = rospy.Rate(25)	# 25Hz
    rospy.sleep(5)

    bot_1 = turtlebot(name='turtlebot_1')
    #bot_1.vx = 0.5
    #bot_1.wz = pi / 16
    time_before = rospy.Time.now().to_nsec() / 1e9 # ns -> s
    time_now    = rospy.Time.now().to_nsec() / 1e9


    while not rospy.is_shutdown():

        time_before = time_now
        time_now = rospy.Time.now().to_nsec() / 1e9 # ns -> s
        dt_ = time_now - time_before
        t = time_now

        vr = 0.25
        wr = 1.5*3.38321412225*0.24*cos(0.24*t)/(1+(3.38321412225*sin(0.24*t))**2)                    # 8 knots
        #wr = 0.25                                                                                    # circle

        bot_1.update_err_dynamics(vr, wr, dt=dt_)
        #bot_1.control_norminal(vr, wr)
        bot_1.control_adaptive(vr, wr, dt=dt_)
        #bot_1.control_robust(vr, wr, t)
        
        rate.sleep()


if __name__ == '__main__':
     try:
         main()
     except rospy.ROSInterruptException:
         pass
