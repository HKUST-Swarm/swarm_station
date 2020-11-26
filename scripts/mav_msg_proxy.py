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
    "yaw": -0.01
}

_ALIGN_PARAM_3DRONE = {
    "pos": np.array([-1.92, -0.038, 0.148]),
    "yaw":0.03
}

_ALIGN_PARAM = _ALIGN_PARAM_2DRONE

MAX_TRAJ_LEN = 200

class SwarmCommander():
    def __init__(self):
        self.node_msg_count = {}
        f = fifo()
        self.self_id = 7

        self.mavlinks = [MAVLink(fifo(), srcSystem=0) for i in range(10)]
        rospy.init_node('swarm_station')
        self.uwb_pub = rospy.Publisher('/uwb_node/send_broadcast_data', data_buffer, queue_size=10)
        self.remote_node_list = []
        self.remote_node_pub = {}

        self.remote_node_vo_pose_pubs = {}

        self.incoming_data_sub = rospy.Subscriber("/uwb_node/incoming_broadcast_data", incoming_broadcast_data, self.on_remote_data, queue_size=100)
        self.incoming_data_sub_tr = rospy.Subscriber("/uwb_node/time_ref", TimeReference, self.on_uwb_timeref, queue_size=100)

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

        self.traj_pubs = {
            0: rospy.Publisher('/traj0', Path, queue_size=10),
            2: rospy.Publisher('/traj2', Path, queue_size=10),
            4: rospy.Publisher('/traj4', Path, queue_size=10)
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

        # self.vicon_traj_subs = {
        #     0 : rospy.Subscriber("/swarm_mocap/SwarmNodePose0", PoseStamped, self.on_vicon0, queue_size=100),
        #     2 : rospy.Subscriber("/swarm_mocap/SwarmNodePose2", PoseStamped, self.on_vicon2, queue_size=100),
        #     4 : rospy.Subscriber("/swarm_mocap/SwarmNodePose4", PoseStamped, self.on_vicon4, queue_size=100),
        # }
    
    def on_vicon0(self, pose0):
        if self.count % 5 ==0:
            self.vicon_pose_trajs[0].poses.append(pose0)
            self.vicon_pose_trajs[0].header = pose0.header
            self.vicon_traj_pubs[0].publish(self.vicon_pose_trajs[0])

    def on_vicon2(self, pose0):
        if self.count % 5 ==0:
            self.vicon_pose_trajs[2].poses.append(pose0)
            self.vicon_pose_trajs[2].header = pose0.header
            self.vicon_traj_pubs[2].publish(self.vicon_pose_trajs[2])
            
    def on_vicon4(self, pose0):
        if self.count % 5 ==0:
            self.vicon_pose_trajs[4].poses.append(pose0)
            self.vicon_pose_trajs[4].header = pose0.header
            self.vicon_traj_pubs[4].publish(self.vicon_pose_trajs[4])

    def on_uwb_timeref(self, timeref):
        self.timeref = timeref

    def lps_2_ros(self, lps_time):
        base = self.timeref.header.stamp - rospy.Duration(self.timeref.time_ref.to_sec())
        return base + rospy.Duration(lps_time / 1000.0)

    def on_remote_data(self, data):
        """
        Header header
        uint32 remote_id
        uint32 remote_recv_time
        uint32 lps_time
        uint8[] data """
        self.parse_data(data.data, data.remote_id, data.lps_time)

    def get_based_fused(self, _from_id, _to_id):
        if (_from_id in self.based_fused and _to_id in self.based_fused[_from_id]):
            return self.based_fused[_from_id][_to_id]
        return None

    def on_based_fused(self, _id, base_fused):
        if not(_id in self.based_fused):
            self.based_fused[_id] = {}
        self.based_fused[_id][base_fused.target_id] = np.array([
            base_fused.rel_x/1000.0,
            base_fused.rel_y/1000.0,
            base_fused.rel_z/1000.0,
            base_fused.rel_yaw_offset/1000.0])
        
    def on_rt_data(self, _id, msg):
        self.count += 1

        self.self_vo[_id] = vo = np.array([
            msg.x,
            msg.y,
            msg.z,
            msg.yaw/1000.0
        ])
        
        if not(_id in self.pubs):
            self.pubs[_id] = {}
        self.pubs[_id][_id] = rospy.Publisher('/swarm_local_{}/drone_pose_{}'.format(_id, _id), PoseStamped, queue_size=10)
        p = PoseStamped()
        fix_x = 0
        fix_y = 0
        fix_z = 0
        fix_yaw = 0

        if _id == 0:
            fix_x = _ALIGN_PARAM["pos"][0]
            fix_y = _ALIGN_PARAM["pos"][1]
            fix_z = _ALIGN_PARAM["pos"][2]
            fix_yaw = _ALIGN_PARAM["yaw"]


        p.pose.position.x = vo[0] + fix_x
        p.pose.position.y = vo[1] + fix_y
        p.pose.position.z = vo[2] + fix_z
        quat = quaternion_from_euler(0, 0, vo[3])
        p.pose.orientation.w = quat[3]
        p.pose.orientation.x = quat[0]
        p.pose.orientation.y = quat[1]
        p.pose.orientation.z = quat[2]
        p.header.frame_id = "world"
        p.header.stamp = self.lps_2_ros(msg.lps_time)
        if _id == 0 and self.count % 5 == 0:   
            self.pose_trajs[_id].poses.append(p)
            if (len(self.pose_trajs[_id].poses)) > MAX_TRAJ_LEN:
                del self.pose_trajs[_id].poses[0]
            self.pose_trajs[_id].header = p.header
            self.traj_pubs[_id].publish(self.pose_trajs[_id])
    
        self.pubs[_id][_id].publish(p)

        for _id_from in self.based_fused:
            if _id_from == 0:
                fix_x = _ALIGN_PARAM["pos"][0]
                fix_y = _ALIGN_PARAM["pos"][1]
                fix_z = _ALIGN_PARAM["pos"][2]
                fix_yaw = _ALIGN_PARAM["yaw"]

            if not(_id_from in self.pubs):
                self.pubs[_id_from] = {}
            if _id in self.based_fused[_id_from]:
                if not(_id in self.pubs[_id_from]):
                    self.pubs[_id_from][_id] = rospy.Publisher('/swarm_local_{}/drone_pose_{}'.format(_id_from, _id), PoseStamped, queue_size=10)
                # var other_vo_origin = this.other_vo_origin[base_id][self_id];
                # var euler_a = new THREE.Euler(0, 0, other_vo_origin.yaw, 'XYZ' );
                # var b_position = pos.clone();
                # b_position.applyEuler(euler_a);
                # b_position.add(new THREE.Vector3(other_vo_origin.x, other_vo_origin.y, other_vo_origin.z));
                # var quat_a = new THREE.Quaternion();
                # quat_a.setFromEuler(euler_a);
                # quat_a.multiply(quat);
                rel_base = self.based_fused[_id_from][_id]
                quata = quaternion_from_euler(0, 0, rel_base[3])
                quatb = quaternion_from_euler(0, 0, vo[3] + fix_yaw)
                
                quat = quaternion_multiply(quata, quatb)
                _pos = vo[0:3]
                _pos = np.dot(quaternion_matrix(quata)[0:3,0:3], _pos)
                _pos = _pos + rel_base[0:3]
                p = PoseStamped()
                p.pose.position.x = _pos[0] + fix_x
                p.pose.position.y = _pos[1] + fix_y
                p.pose.position.z = _pos[2] + fix_z
                p.pose.orientation.w = quat[3]
                p.pose.orientation.x = quat[0]
                p.pose.orientation.y = quat[1]
                p.pose.orientation.z = quat[2]
                p.header.frame_id = "world"
                p.header.stamp = self.lps_2_ros(msg.lps_time)
                self.pubs[_id_from][_id].publish(p)


                if _id_from == 0 and self.count % 5 == 0:   
                    self.pose_trajs[_id].poses.append(p)
                    if (len(self.pose_trajs[_id].poses)) > MAX_TRAJ_LEN:
                        del self.pose_trajs[_id].poses[0]

                    self.pose_trajs[_id].header = p.header
                    self.traj_pubs[_id].publish(self.pose_trajs[_id])

    def parse_data(self, s, _id, now_lps):
        t1 = rospy.get_time()

        try:
            msgs = self.mavlinks[_id].parse_buffer(s)
            # print(len(s))
            # print("Time cost ", (rospy.get_time() -t1)*1000 )
            if msgs is None:
                return
            for msg in msgs:
                if msg is not None:
                    if msg.get_type() == "NODE_BASED_FUSED":
                        self.on_based_fused(_id, msg)
                    if msg.get_type() == "NODE_REALTIME_INFO":
                        self.on_rt_data(_id, msg)
            # print("Time cost ", (rospy.get_time() -t1)*1000 )

        except Exception as inst:
            print(inst)
            pass

if __name__ == "__main__":
    sw_cmd = SwarmCommander()
    rospy.loginfo("start swarm commander")
    rospy.spin()
