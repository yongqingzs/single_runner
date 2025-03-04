#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import PoseStamped, Point
from nav_msgs.msg import Odometry
import math
from pyquaternion import Quaternion
import tf
import sys

DEBUG_FLAG = True

if len(sys.argv) < 2:
    sys.argv.append("iris")
    sys.argv.append("0")

vehicle_type = sys.argv[1]
vehicle_id = sys.argv[2]
local_pose = PoseStamped()
local_pose.header.frame_id = 'world'

# 创建坐标系变换的四元数（根据Fast-LIO2的坐标系调整）
# Fast-LIO2的雷达坐标系通常是: x向前, y向左, z向上
# PX4/MAVROS的坐标系是: x向前, y向右, z向下
# 因此需要绕x轴旋转180度（相当于y和z轴取反）
# quaternion = tf.transformations.quaternion_from_euler(-math.pi/2, 0, -math.pi/2)
# quaternion = tf.transformations.quaternion_from_euler(math.pi/2, 0, math.pi/2)
quaternion = tf.transformations.quaternion_from_euler(0, 0, 0)
q = Quaternion([quaternion[3], quaternion[0], quaternion[1], quaternion[2]])

def fastlio_callback(data):
    """
    处理来自Fast-LIO2的位姿估计
    data: Fast-LIO2发布的Odometry消息
    """
    if DEBUG_FLAG:
        print("Fast-LIO2 pose received")
    
    # 提取位置信息
    local_pose.pose.position.x = data.pose.pose.position.x
    local_pose.pose.position.y = data.pose.pose.position.y
    local_pose.pose.position.z = data.pose.pose.position.z
    
    # 提取方向信息并应用坐标变换
    q_ = Quaternion([
        data.pose.pose.orientation.w,
        data.pose.pose.orientation.x,
        data.pose.pose.orientation.y,
        data.pose.pose.orientation.z
    ])
    q_ = q_ * q
    
    local_pose.pose.orientation.w = q_[0]
    local_pose.pose.orientation.x = q_[1]
    local_pose.pose.orientation.y = q_[2]
    local_pose.pose.orientation.z = q_[3]
    
    # 输出调试信息
    if DEBUG_FLAG:
        print("before transform: ", data.pose.pose.position.x, data.pose.pose.position.y, data.pose.pose.position.z)
        print("after transform: ", local_pose.pose.position.x, local_pose.pose.position.y, local_pose.pose.position.z)

# 初始化ROS节点
rospy.init_node(vehicle_type+"_"+vehicle_id+'_fastlio_transfer')

# 订阅Fast-LIO2的Odometry话题
# 根据你的Fast-LIO2配置调整话题名称
# rospy.Subscriber("/fastlio/mapping/odometry", Odometry, fastlio_callback, queue_size=1)
rospy.Subscriber("Odometry", Odometry, fastlio_callback, queue_size=1)

# 发布到MAVROS的视觉位姿估计话题
position_pub = rospy.Publisher(vehicle_type+"_"+vehicle_id+"/mavros/vision_pose/pose", PoseStamped, queue_size=1)

# 设置发布频率
rate = rospy.Rate(30)

# 主循环
while not rospy.is_shutdown():
    if (local_pose.pose.position == Point()):
        # 如果还没有收到有效位姿，则继续等待
        continue
    else:
        # 更新时间戳并发布位姿
        local_pose.header.stamp = rospy.Time.now()
        position_pub.publish(local_pose)
    try:
        rate.sleep()
    except:
        continue