#!/usr/bin/env python
from __future__ import print_function
import rospy
import sys
from pymavlink4swarm import MAVLink
import pymavlink4swarm as pymavlink
import time
from inf_uwb_ros.msg import *
import time
from geometry_msgs.msg import Pose, PoseStamped
from pyquaternion import Quaternion
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import threading
from sensor_msgs.msg import Joy
from swarmtal_msgs.msg import  drone_onboard_command
import math

class fifo(object):
    def __init__(self):
        self.buf = []
    def write(self, data):
        self.buf += data
        return len(data)
    def read(self):
        return self.buf.pop(0)


class SwarmCommander():
    def __init__(self):
        self.node_msg_count = {}
        f = fifo()
        self.self_id = 7

        self.mav = MAVLink(f, srcSystem=0)
        rospy.init_node('swarm_station')
        self.uwb_pub = rospy.Publisher('/uwb_node/send_broadcast_data', data_buffer, queue_size=10)
        self.remote_node_list = []
        self.remote_node_pub = {}

        self.remote_node_vo_pose_pubs = {}

        self.incoming_data_sub = rospy.Subscriber("/uwb_node/incoming_broadcast_data", incoming_broadcast_data, self.on_remote_data, queue_size=100)

        self.joysub = rospy.Subscriber("/joy", Joy, self.on_joy_message, queue_size=1, tcp_nodelay=True)

        self.target_id = -1

        self.joy = None

        self.in_mission = None

        self.simple_mission_timer = rospy.Timer(rospy.Duration(0.02), self.simple_mission_callbck)
        self.start_misssion_time = rospy.get_rostime()

    def on_joy_message(self, joy):

        if self.joy is None:
            rospy.loginfo("DETECTED JOY!")
            rospy.loginfo(joy)

        self.joy = joy

        # Start key for takeoff
        if joy.buttons[7] == 1:
            self.takeoff_cmd(self.target_id)
            return

        #Back for landing
        if joy.buttons[6] == 1:
            self.landing_cmd(self.target_id, True)
            return

        #Two fire button, toggle att landing
        if joy.axes[2] < -0.99 and joy.axes[5] < -0.99:
            # self.toggle_arm_disarm(self.target_id, arm=False)
            self.landing_cmd(self.target_id, True)
            return

        #LB and RB for arm
        if joy.buttons[5] == 1 and joy.buttons[4] == 1:
            self.toggle_arm_disarm(self.target_id, arm=True)
            return

        if joy.buttons[0] == 1:
            self.start_simple_mission("A")
            return

        if joy.buttons[1] == 1:
            self.start_simple_mission("B")
            return

        if joy.buttons[3] == 1:
            self.start_simple_mission("Y")
            return

        if joy.buttons[2] == 1:
            rospy.loginfo("Abort Simple Mission")
            self.abort_simple_mission()
            return

    def drone_onboard_cmd_tomav(self, target, cmd):
        msg = self.mav.swarm_remote_command_encode(0, target, cmd.command_type,
                                                   cmd.param1, cmd.param2, cmd.param3, cmd.param4, cmd.param5,
                                                   cmd.param6, cmd.param7, cmd.param8, cmd.param9, cmd.param10)
        return msg


    def start_simple_mission(self, mission="A"):
        rospy.loginfo("Will start simple mission {} by UWB command".format(mission))
        self.in_mission = mission
        self.start_misssion_time = rospy.get_rostime()


    def abort_simple_mission(self):
        self.in_mission = None


    def simple_mission_callbck(self, e):
        if self.in_mission is None:
            return
        dt = 0
        if e.last_real is not None:
            dt = (e.current_real - e.last_real).to_sec()

        t = (e.current_real - self.start_misssion_time).to_sec()
        rospy.loginfo_throttle(1.0, "Act mission {} with {:5.3f}s".format(self.in_mission, t))

        def mission_A(t):
            ox = 0
            oy = 0
            r = 0.5
            T = 15

            x = ox + math.sin(t*math.pi*2/T)*r
            y = oy + math.cos(t*math.pi*2/T)*r
            vx = math.cos(t*math.pi*2/T) * r * math.pi*2/T
            vy = -math.sin(t*math.pi*2/T) * r * math.pi*2/T

            return x, y, 1, vx, vy, 0

        def mission_B(t):
            ox = 0
            oy = 0
            r = 0.5
            T = 15

            x = ox + math.sin(t*math.pi*2/T)*r
            y = oy + math.sin(2*t*math.pi*2/T)*r
            vx = math.cos(t*math.pi*2/T) * r * math.pi*2/T
            vy = 2*math.cos(2*t*math.pi*2/T) * r * math.pi*2/T
            return x, y, 1, vx, vy, 0

        def mission_Y(t):
            ox = 0
            oy = 0
            oz = 1
            r = 0.5
            T = 15

            x = ox + math.sin(t*math.pi*2/T)*r
            y = oy + math.sin(2*t*math.pi*2/T)*r
            z = oz  + math.sin(2*t*math.pi*2/T+math.pi)*0.3
            vx = math.cos(t*math.pi*2/T) * r * math.pi*2/T
            vy = 2*math.cos(2*t*math.pi*2/T) * r * math.pi*2/T
            vz = 2*math.cos(2*t*math.pi*2/T + math.pi) * 0.3 * math.pi*2/T
            return x, y, z, vx, vy, vz


        x, y, z, vx, vy, vz = 0, 0, 0, 0, 0, 0

        if self.in_mission == "A":
            x, y, z, vx, vy, vz = mission_A(t)

        if self.in_mission == "B":
            x, y, z, vx, vy, vz = mission_B(t)

        if self.in_mission == "Y":
            x, y, z, vx, vy, vz = mission_Y(t)



        cmd = drone_onboard_command()
        cmd.command_type = drone_onboard_command.CTRL_POS_COMMAND

        cmd.param1 = int(x*10000)
        cmd.param2 = int(y*10000)
        cmd.param3 = int(z*10000)
        cmd.param4 = 666666
        cmd.param5 = int(vx*10000)
        cmd.param6 = int(vy*10000)
        cmd.param7 = int(vz*10000)
        cmd.param8 = 0
        cmd.param9 = 0
        cmd.param10 = 0


        msg = self.drone_onboard_cmd_tomav(self.target_id, cmd)

        print(msg)
        
        self.send_mavlink_msg(msg)




    def on_remote_data(self, data):
        """
        Header header
        uint32 remote_id
        uint32 remote_recv_time
        uint32 lps_time
        uint8[] data """
        self.parse_data(data.data, data.remote_id, data.lps_time)

    def on_vicon_pose_recv(self, pose):
        self.vicon_pose = pose.pose

    def toggle_arm_disarm(self, target, arm=True):
        # msg = self.mav.command_long_encode(target, 0, pymavlink.MAV_CMD_DO_SET_MODE, 0, 192, 0, 0, 0, 0, 0, 0)
        cmd = drone_onboard_command()
        cmd.command_type = drone_onboard_command.CTRL_ARM_COMMAND
        if arm:
            cmd.param1 = 1
            rospy.loginfo("Sending ARM")
        else:
            cmd.param1 = 0
            rospy.loginfo("Sending DISARM")

        msg = self.drone_onboard_cmd_tomav(target, cmd)
        self.send_mavlink_msg(msg)

    def vel_ctrl_cmd(self, target):
        pass

    def takeoff_cmd(self, target):
        rospy.loginfo("Sending takeoff")
        cmd = drone_onboard_command()
        cmd.command_type = drone_onboard_command.CTRL_TAKEOF_COMMAND
        cmd.param1 = 10000
        msg = self.drone_onboard_cmd_tomav(target, cmd)
        self.send_mavlink_msg(msg)

    def landing_cmd(self, target, att_landing=True):
        rospy.loginfo("Sending landing")
        cmd = drone_onboard_command()
        cmd.command_type = drone_onboard_command.CTRL_LANDING_COMMAND
        if att_landing:
            cmd.param1 = 1
        else:
            cmd.param1 = 0
        msg = self.drone_onboard_cmd_tomav(target, cmd)
        self.send_mavlink_msg(msg)

    def pathplaning_cmd(self, target):
        msg = self.mav.command_long_encode(target, 0, pymavlink.MAV_CMD_NAV_PATHPLANNING, 0, 0, 0, 0, 0, 0, 0, 0)
        self.send_mavlink_msg(msg)
        #Random fly to test our fuse algorithms

    def send_manual_control(self, rc_state):
        target = rc_state.target_id
        x = int(rc_state.rcA * 1000)
        y = int(rc_state.rcE * 1000)
        z = int(rc_state.rcT * 1000)
        r = int(rc_state.rcR * 1000) 
        buttons = 4*rc_state.sw1 + 2*rc_state.sw2 + rc_state.sw3
        msg = self.mav.manual_control_encode(target, x, y, z, r, buttons)
        self.send_mavlink_msg(msg)

    def send_mavlink_msg(self, msg):
        buf = msg.pack(self.mav, force_mavlink1=False)
        _buf = data_buffer()
        _buf.data = buf
        self.uwb_pub.publish(_buf)
        #print(buf)

    def on_drone_status(self, node_id, msg, nowlps):
        rospy.loginfo("ID {} DT {}ms".format(node_id, nowlps - msg.lps_time))
        rospy.loginfo(msg)
        if math.fabs(msg.x) > 5 or math.fabs(msg.y) > 5 or  math.fabs(msg.z) > 5:
            rospy.logwarn("ID {} VO invaild {:4.3f} {:4.3f} {:4.3f}".format(node_id, msg.x, msg.y, msg.z))
        if math.fabs(msg.bat_vol) < 14.8:
            rospy.logwarn("ID {} BAT invaild {:3.2f}".format(node_id, msg.bat_vol))

    #
    def parse_data(self, s, _id, now_lps):
        t1 = rospy.get_rostime()
        try:
            msgs = self.mav.parse_buffer(s)
            # print(len(s))
            if msgs is None:
                return
            for msg in msgs:
                if msg is not None:
                    if msg.get_type() == "DRONE_STATUS":
                        self.on_drone_status(_id, msg, now_lps)
                    else:
                        pass
                        # print(msg)
                    # return
        except Exception as inst:
            print(inst)
            pass

if __name__ == "__main__":
    sw_cmd = SwarmCommander()
    rospy.loginfo("start swarm commander")
    rospy.spin()
