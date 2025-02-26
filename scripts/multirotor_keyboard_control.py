#!/usr/bin/env python3
import time
import rospy
from geometry_msgs.msg import Twist
import sys, select, os
import tty, termios
from std_msgs.msg import String

"""
命令行参数
1. 机型: multirotor_type
2. 数量: multirotor_num
3. 控制方式: control_type
"""
# only for test
DEBUG_FLAG = 1
if DEBUG_FLAG == 0:
    pass
elif DEBUG_FLAG == 1:
    sys.argv.append('solo')
    sys.argv.append('1')
    sys.argv.append('vel')
###############

MAX_LINEAR = 20
MAX_ANG_VEL = 3
LINEAR_STEP_SIZE = 0.01
ANG_VEL_STEP_SIZE = 0.01

cmd_vel_mask = False
ctrl_leader = False

msg2all = """
Control Your XTDrone!
To all drones  (press g to control the leader)
---------------------------
   1   2   3   4   5   6   7   8   9   0
        w       r    t   y        i
   a    s    d       g       j    k    l
        x       v    b   n        ,

w/x : increase/decrease forward  
a/d : increase/decrease leftward 
i/, : increase/decrease upward 
j/l : increase/decrease yaw
r   : return home
t/y : arm/disarm
v/n : takeoff/land
b   : offboard
s/k : hover and remove the mask of keyboard control
0   : mask the keyboard control (for single UAV)
0~9 : extendable mission(eg.different formation configuration)
      this will mask the keyboard control
g   : control the leader
c   : One click! increase upward to 0.30; offboard; arm
CTRL-C to quit
"""

msg2leader = """
Control Your XTDrone!
To the leader  (press g to control all drones)
---------------------------
   1   2   3   4   5   6   7   8   9   0
        w       r    t   y        i
   a    s    d       g       j    k    l
        x       v    b   n        ,

w/x : increase/decrease forward 
a/d : increase/decrease leftward 
i/, : increase/decrease upward 
j/l : increase/decrease yaw
r   : return home
t/y : arm/disarm
v/n : takeoff/land
b   : offboard
s/k : hover and remove the mask of keyboard control
g   : control all drones
c   : One click! increase upward to 0.31; offboard; arm
CTRL-C to quit
"""

e = """
Communications Failed
"""

def getKey():
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

def print_msg():
    if ctrl_leader:
        print(msg2leader)
    else:
        print(msg2all)      

if __name__=="__main__":

    settings = termios.tcgetattr(sys.stdin)

    multirotor_type = sys.argv[1]
    multirotor_num = int(sys.argv[2])
    control_type = sys.argv[3]
    
    if multirotor_num == 18:
        formation_configs = ['waiting', 'cuboid', 'sphere', 'diamond']
    elif multirotor_num == 9:
        formation_configs = ['waiting', 'cube', 'pyramid', 'triangle']
    elif multirotor_num == 6:
        formation_configs = ['waiting', 'T', 'diamond', 'triangle']
    elif multirotor_num == 1:
        formation_configs = ['stop controlling']
    
    cmd= String()
    twist = Twist()   
    
    rospy.init_node(multirotor_type + '_multirotor_keyboard_control')
    
    if control_type == 'vel':
        multi_cmd_vel_flu_pub = [None]*multirotor_num
        multi_cmd_pub = [None]*multirotor_num
        for i in range(multirotor_num):
            multi_cmd_vel_flu_pub[i] = rospy.Publisher('/xtdrone/'+multirotor_type+'_'+str(i)+'/cmd_vel_flu', Twist, queue_size=1)
            multi_cmd_pub[i] = rospy.Publisher('/xtdrone/'+multirotor_type+'_'+str(i)+'/cmd',String,queue_size=3)
        leader_cmd_vel_flu_pub = rospy.Publisher("/xtdrone/leader/cmd_vel_flu", Twist, queue_size=1)
        leader_cmd_pub = rospy.Publisher("/xtdrone/leader/cmd", String, queue_size=1)
 
    else:
        multi_cmd_accel_flu_pub = [None]*multirotor_num
        multi_cmd_pub = [None]*multirotor_num
        for i in range(multirotor_num):
            multi_cmd_accel_flu_pub[i] = rospy.Publisher('/xtdrone/'+multirotor_type+'_'+str(i)+'/cmd_accel_flu', Twist, queue_size=1)
            multi_cmd_pub[i] = rospy.Publisher('/xtdrone/'+multirotor_type+'_'+str(i)+'/cmd',String,queue_size=1)
        leader_cmd_accel_flu_pub = rospy.Publisher("/xtdrone/leader/cmd_accel_flu", Twist, queue_size=1)
        leader_cmd_pub = rospy.Publisher("/xtdrone/leader/cmd", String, queue_size=3)

    forward  = 0.0
    leftward  = 0.0
    upward  = 0.0
    angular = 0.0

    c_flag = 0
    c_start_time = None
    print_msg()
    while(1):
        key = getKey()
        if c_flag == 1:
            cmd = 'OFFBOARD'
            # print_msg()
            print('Offboard')
            key = ''
            c_flag += 1
        elif c_flag == 2:
            cmd = 'ARM'
            # print_msg()
            print('Arming')
            key = ''
            c_flag = 3
            c_start_time = time.time()
        elif c_flag == 3:
            if c_start_time is not None:
                elapsed_time = time.time() - c_start_time
                if elapsed_time > 20.0:
                    key = 's'
                    c_flag = 0
                    print("One click over!")

        if key == 'w' :
            forward = forward + LINEAR_STEP_SIZE
            print_msg()
            if control_type == 'vel':
                print("currently:\t forward vel %.2f\t leftward vel %.2f\t upward vel %.2f\t angular %.2f " % (forward, leftward, upward, angular))
            else:
                print("currently:\t forward vel %.2f\t leftward vel %.2f\t upward vel %.2f\t angular %.2f " % (forward, leftward, upward, angular))

        elif key == 'c':
            upward = 0.3
            c_flag = 1
            print("One click start!")
            print_msg()

        elif key == 'x' :
            forward = forward - LINEAR_STEP_SIZE
            print_msg()
            if control_type == 'vel':
                print("currently:\t forward vel %.2f\t leftward vel %.2f\t upward vel %.2f\t angular %.2f " % (forward, leftward, upward, angular))
            else:
                print("currently:\t forward vel %.2f\t leftward vel %.2f\t upward vel %.2f\t angular %.2f " % (forward, leftward, upward, angular))

        elif key == 'a' :

            leftward = leftward + LINEAR_STEP_SIZE
            print_msg()
            if control_type == 'vel':
                print("currently:\t forward vel %.2f\t leftward vel %.2f\t upward vel %.2f\t angular %.2f " % (forward, leftward, upward, angular))
            else:
                print("currently:\t forward vel %.2f\t leftward vel %.2f\t upward vel %.2f\t angular %.2f " % (forward, leftward, upward, angular))

        elif key == 'd' :
            leftward = leftward - LINEAR_STEP_SIZE
            print_msg()
            if control_type == 'vel':
                print("currently:\t forward vel %.2f\t leftward vel %.2f\t upward vel %.2f\t angular %.2f " % (forward, leftward, upward, angular))
            else:
                print("currently:\t forward vel %.2f\t leftward vel %.2f\t upward vel %.2f\t angular %.2f " % (forward, leftward, upward, angular))

        elif key == 'i' :
            upward = upward + LINEAR_STEP_SIZE
            print_msg()
            if control_type == 'vel':
                print("currently:\t forward vel %.2f\t leftward vel %.2f\t upward vel %.2f\t angular %.2f " % (forward, leftward, upward, angular))
            else:
                print("currently:\t forward vel %.2f\t leftward vel %.2f\t upward vel %.2f\t angular %.2f " % (forward, leftward, upward, angular))

        elif key == ',' :
            upward = upward - LINEAR_STEP_SIZE
            print_msg()
            if control_type == 'vel':
                print("currently:\t forward vel %.2f\t leftward vel %.2f\t upward vel %.2f\t angular %.2f " % (forward, leftward, upward, angular))
            else:
                print("currently:\t forward vel %.2f\t leftward vel %.2f\t upward vel %.2f\t angular %.2f " % (forward, leftward, upward, angular))

        elif key == 'j':
            angular = angular + ANG_VEL_STEP_SIZE
            print_msg()
            print("currently:\t forward vel %.2f\t leftward vel %.2f\t upward vel %.2f\t angular %.2f " % (forward, leftward, upward, angular))

        elif key == 'l':
            angular = angular - ANG_VEL_STEP_SIZE
            print_msg()
            print("currently:\t forward vel %.2f\t leftward vel %.2f\t upward vel %.2f\t angular %.2f " % (forward, leftward, upward, angular))

        elif key == 'r':
            cmd = 'AUTO.RTL'
            print_msg()
            print('Returning home')
        elif key == 't':
            cmd = 'ARM'
            print_msg()
            print('Arming')
        elif key == 'y':
            cmd = 'DISARM'
            print_msg()
            print('Disarming')
        elif key == 'v':
            cmd = 'AUTO.TAKEOFF'
            print_msg()
            #print('Takeoff mode is disenabled now')
        elif key == 'b':
            cmd = 'OFFBOARD'
            print_msg()
            print('Offboard')
        elif key == 'n':
            cmd = 'AUTO.LAND'
            print_msg()
            print('Landing')
        elif key == 'g':
            ctrl_leader = not ctrl_leader
            print_msg()
        elif key in ['k', 's']:
            cmd_vel_mask = False
            forward   = 0.0
            leftward   = 0.0
            upward   = 0.0
            angular  = 0.0
            cmd = 'HOVER'
            print_msg()
            print("currently:\t forward vel %.2f\t leftward vel %.2f\t upward vel %.2f\t angular %.2f " % (forward, leftward, upward, angular))
            print('Hover')
        else:
            for i in range(10):
                if key == str(i):
                    cmd = formation_configs[i]
                    print_msg()
                    print(cmd)
                    cmd_vel_mask = True
            if (key == '\x03'):
                break

        if forward > MAX_LINEAR:
            forward = MAX_LINEAR
        elif forward < -MAX_LINEAR:
            forward = -MAX_LINEAR
        if leftward > MAX_LINEAR:
            leftward = MAX_LINEAR
        elif leftward < -MAX_LINEAR:
            leftward = -MAX_LINEAR
        if upward > MAX_LINEAR:
            upward = MAX_LINEAR
        elif upward < -MAX_LINEAR:
            upward = -MAX_LINEAR
        if angular > MAX_ANG_VEL:
            angular = MAX_ANG_VEL
        elif angular < -MAX_ANG_VEL:
            angular = - MAX_ANG_VEL
            
        twist.linear.x = forward; twist.linear.y = leftward ; twist.linear.z = upward
        twist.angular.x = 0.0; twist.angular.y = 0.0;  twist.angular.z = angular

        for i in range(multirotor_num):
            if ctrl_leader:
                if control_type == 'vel':
                    leader_cmd_vel_flu_pub.publish(twist)
                else:
                    leader_cmd_accel_flu_pub.publish(twist)
                leader_cmd_pub.publish(cmd)
            else:
                if not cmd_vel_mask:
                    if control_type == 'vel':
                        multi_cmd_vel_flu_pub[i].publish(twist)   
                    else:
                        multi_cmd_accel_flu_pub[i].publish(twist)
                multi_cmd_pub[i].publish(cmd)
                
        cmd = ''

    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)

