#!/usr/bin/env python
from __future__ import print_function

import sys
import time
import math

import rospy
from pymavlink4swarm import MAVLink
import pymavlink4swarm as pymavlink
from inf_uwb_ros.msg import *
from geometry_msgs.msg import Pose, PoseStamped
from tf.transformations import euler_from_quaternion, quaternion_from_euler, quaternion_matrix, quaternion_multiply
import threading
from sensor_msgs.msg import Joy, TimeReference
from swarmtal_msgs.msg import  drone_onboard_command
import numpy as np

from nav_msgs.msg import Path
MAX_TRAJ_LEN = 240

class fifo(object):
    def __init__(self):
        self.buf = []
    def write(self, data):
        self.buf += data
        return len(data)
    def read(self):
        return self.buf.pop(0)


_ALIGN_PARAM_2DRONE = {
    "pos": np.array([-0.37586, 0.1145, 0.1454]),
    "yaw": -0.04
}

_ALIGN_PARAM_3DRONE = {
    "pos": np.array([-1.92, -0.038, 0.148]),
    "yaw":0.03
}

_ALIGN_PARAM = _ALIGN_PARAM_3DRONE


def toKey(pose):
    return "{:10.9f} {:10.9f} {:10.9f} {:10.9f} {:10.9f} {:10.9f}  {:10.9f}".format(
        pose.pose.position.x,
        pose.pose.position.y,
        pose.pose.position.z,
        pose.pose.orientation.w,
        pose.pose.orientation.x,
        pose.pose.orientation.y,
        pose.pose.orientation.z
    )

class SwarmCommander():
    def __init__(self):
        self.node_msg_count = {}
        f = fifo()
        self.self_id = 7

        self.mavlinks = [MAVLink(fifo(), srcSystem=0) for i in range(10)]
        rospy.init_node('swarm_station_2')
        self.remote_node_list = []
        self.remote_node_pub = {}

        self.remote_node_vo_pose_pubs = {}

        self.target_id = -1

        self.joy = None

        self.in_mission = None

        self.self_vo = {}
        self.ids = set()

        self.based_fused = {}
        self.pubs = {}

        self.count = 0

        self.pose_trajs = {
            0: Path(),
            2: Path(),
            4: Path()
        }

        self.vicon_pose_trajs = {
            0: Path(),
            2: Path(),
            4: Path()
        }

        self.vicon_traj_pubs = {
            0: rospy.Publisher('/vicon_traj0', Path, queue_size=10),
            2: rospy.Publisher('/vicon_traj2', Path, queue_size=10),
            4: rospy.Publisher('/vicon_traj4', Path, queue_size=10)
        }

        self.vicon_re_pubs = {
            0: rospy.Publisher('/swarm_mocap/SwarmNodePose0_re', PoseStamped, queue_size=10),
            2: rospy.Publisher('/swarm_mocap/SwarmNodePose2_re', PoseStamped, queue_size=10),
            4: rospy.Publisher('/swarm_mocap/SwarmNodePose4_re', PoseStamped, queue_size=10)
        }

        self.vicon_traj_subs = {
            0 : rospy.Subscriber("/swarm_mocap/SwarmNodePose0", PoseStamped, self.on_vicon0, queue_size=100),
            2 : rospy.Subscriber("/swarm_mocap/SwarmNodePose2", PoseStamped, self.on_vicon2, queue_size=100),
            4 : rospy.Subscriber("/swarm_mocap/SwarmNodePose4", PoseStamped, self.on_vicon4, queue_size=100),
        }

        self.count_0 = 0
        self.count_1 = 0
        self.count_2 = 0
        self.count_4 = 0


        self.vicon_series = {
            0 :set(),
            2 :set(),
            4 :set()
        }
    
    def on_vicon0(self, pose0):
        if toKey(pose0) in self.vicon_series[0]:
            # print("Duplicate Key!")
            self.vicon_series[0].remove(toKey(pose0))
            return

        self.vicon_re_pubs[0].publish(pose0)
        self.vicon_series[0].add(toKey(pose0))
        self.count_0 += 1

        if self.count_0 % 5!=0:
            return
        
        self.vicon_pose_trajs[0].poses.append(pose0)

        if (len(self.vicon_pose_trajs[0].poses)) > MAX_TRAJ_LEN:
            del self.vicon_pose_trajs[0].poses[0]
        self.vicon_pose_trajs[0].header = pose0.header
        self.vicon_traj_pubs[0].publish(self.vicon_pose_trajs[0])

    def on_vicon2(self, pose0):
        if toKey(pose0) in self.vicon_series[2]:
            # print("Duplicate Key!")
            self.vicon_series[2].remove(toKey(pose0))
            return

        self.vicon_re_pubs[2].publish(pose0)
        self.vicon_series[2].add(toKey(pose0))
        
        self.count_2 += 1
        if self.count_2 % 5!=0:
            return

    
        self.vicon_pose_trajs[2].poses.append(pose0)
        if (len(self.vicon_pose_trajs[2].poses)) > MAX_TRAJ_LEN:
            del self.vicon_pose_trajs[2].poses[0]

        self.vicon_pose_trajs[2].header = pose0.header
        self.vicon_traj_pubs[2].publish(self.vicon_pose_trajs[2])
            
    def on_vicon4(self, pose0):
        
        if toKey(pose0) in self.vicon_series[4]:
            # print("Duplicate Key!")
            self.vicon_series[4].remove(toKey(pose0))
            return

        self.vicon_re_pubs[4].publish(pose0)
        self.vicon_series[4].add(toKey(pose0))


        self.count_4 += 1
        if self.count_4 % 5!=0:
            return

        self.vicon_pose_trajs[4].poses.append(pose0)
        if (len(self.vicon_pose_trajs[4].poses)) > MAX_TRAJ_LEN:
            del self.vicon_pose_trajs[4].poses[0]
        self.vicon_pose_trajs[4].header = pose0.header
        self.vicon_traj_pubs[4].publish(self.vicon_pose_trajs[4])

if __name__ == "__main__":
    sw_cmd = SwarmCommander()
    rospy.loginfo("start swarm commander")
    rospy.spin()
