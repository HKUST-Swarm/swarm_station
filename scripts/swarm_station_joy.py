#!/usr/bin/env python
from __future__ import print_function
import rospy
import pymavlink
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
        f = fifo()
        self.self_id = 7

        self.mav = MAVLink(f, srcSystem=0)
        rospy.init_node('swarm_station')
        self.uwb_pub = rospy.Publisher('/uwb_node/send_broadcast_data', data_buffer, queue_size=10)
        self.remote_node_list = []
        self.remote_node_pub = {}

        self.remote_node_vo_pose_pubs = {}

        self.vicon_pose = Pose()
        self.vicon_pose_sub = rospy.Subscriber("/SwarmNode7/pose", PoseStamped, self.on_vicon_pose_recv, queue_size=1);

        self.self_vo_pose = Pose()
        self.self_vo_yaw = 0
        self.uwb_recv_data = rospy.Subscriber("/uwb_node/remote_nodes", remote_uwb_info, self.on_remote_node_info, queue_size=1)

        self.joysub = rospy.Subscriber("/joy", Joy, self.on_joy_message, queue_size=1, tcp_nodelay=True)

        self.target_id = -1

    def on_joy_message(self, joy):
        self.joy = joy


        # Start key for takeoff
        if joy.buttons[7] == 1:
            self.takeoff_cmd(self.target_id)

        #Back for landing
        if joy.buttons[6] == 1:
            self.landing_cmd(self.target_id)

        #Two fire button, toggle takeoff
        if joy.axes[2] < -0.99 and joy.axes[5] < -0.99:
            self.toggle_arm_disarm(self.target_id, arm=False)

        #LB and RB for arm
        if joy.buttons[5] == 1 and joy.buttons[4] == 1:
            self.toggle_arm_disarm(self.target_id, arm=True)

    def drone_onboard_cmd_tomav(self, target, cmd):
        msg = self.mav.swarm_remote_command_encode(target, cmd.command_type,
                                                   cmd.param1, cmd.param2, cmd.param3, cmd.param4, cmd.param5,
                                                   cmd.param6, cmd.param7, cmd.param8, cmd.param9, cmd.param10)
        return msg



    def on_remote_node_info(self, info):
        return
        ptr = 0
        for i in range(len(info.node_ids)):
            if info.node_ids[i] == self.reference_node:
                ptr = i
                break
        for i in range(len(info.node_ids)):
            _id = info.node_ids[i]
            if info.data_available[i]:
                self.parse_data(info.datas[i].data, _id)
            # print("Finish parse data")

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

    def takeoff_cmd(self, target):
        rospy.loginfo("Sending takeoff")
        cmd = drone_onboard_command()
        cmd.command_type = drone_onboard_command.CTRL_TAKEOF_COMMAND
        cmd.param1 = 10000
        msg = self.drone_onboard_cmd_tomav(target, cmd)
        self.send_mavlink_msg(msg)

    def landing_cmd(self, target):
        rospy.loginfo("Sending landing")
        cmd = drone_onboard_command()
        cmd.command_type = drone_onboard_command.CTRL_LANDING_COMMAND
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
    
    def swarm_relative_fused_recv(self, msg):
        # print(msg)
        source_id = msg.source_id
        target_id = msg.target_id
        if source_id == 7 and target_id !=7:
            self.remote_node_list.append(target_id)
            if target_id not in self.remote_node_pub:
                self.remote_node_pub[target_id] = rospy.Publisher("/swarm_drone/estimate_pose_{}".format(target_id), PoseStamped);

            _pose = PoseStamped()
            remote_pose = _pose.pose
            _pose.header.frame_id = "world"
            _pose.header.stamp = rospy.Time.now()
            remote_pose.position.x = self.vicon_pose.position.x + msg.rel_x
            remote_pose.position.y = self.vicon_pose.position.y + msg.rel_y
            remote_pose.position.z = self.vicon_pose.position.z + msg.rel_z

            remote_pose.orientation.w = 1
            self.remote_node_pub[target_id].publish(_pose)
    
    def swarm_vo_info_recv(self, msg, is_self_node=False):
        if is_self_node:
            self.self_vo_pose.position.x = msg.x
            self.self_vo_pose.position.y = msg.y
            self.self_vo_pose.position.z = msg.z

            self.self_vo_pose.orientation.w = msg.q0
            self.self_vo_pose.orientation.x = msg.q1
            self.self_vo_pose.orientation.y = msg.q2
            self.self_vo_pose.orientation.z = msg.q3

            quaternion = (msg.q1, msg.q2, msg.q3, msg.q0)
            roll, pitch, yaw = euler_from_quaternion(quaternion)
            self.self_vo_yaw = yaw
        # print(self.self_vo_pose)


    def parse_data(self, s, _id):
        try:
            is_ref_node = _id == self.reference_node
            msg = self.mav.parse_char(s)
            if msg is not None:
                if msg.get_type() == "SWARM_RELATIVE_FUSED":
                    print("Rel",msg)
                    if is_ref_node:
                        self.swarm_relative_fused_recv(msg)
                if msg.get_type() == "SWARM_INFO":
                    # print(" {}: {:3.2f} {:3.2f} {:3.2f}".format(_id, msg.x, msg.y, msg.z))
                    self.swarm_vo_info_recv(msg, is_ref_node)

        except Exception as inst:
            print(inst)
            pass

if __name__ == "__main__":
    sw_cmd = SwarmCommander()
    rospy.loginfo("start swarm commander")
    rospy.spin()
