#!/usr/bin/env python2 
# -*- coding: utf-8 -*-

import rospy #导入rospy库
import actionlib #导入actionlib 库
import os,inspect #导入os库
from actionlib_msgs.msg import * #导入actionlib的所有模块
from geometry_msgs.msg import Pose, Point, Quaternion, Twist #导入四个消息数据类型，姿态，目标点，四元数，运动消息Twist
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal #导入movebase的两个消息数据类型
from tf.transformations import quaternion_from_euler #导入tf变换库的欧拉角转四元数库
from math import pi #导入圆周率pi
from std_msgs.msg import String #导入标准消息的字符串消息数据格式
from std_msgs.msg import Int8
from darknet_msgs.msg import darknet
import yaml
import time
import threading
import shutil
from sr_pkg.srv import *

nav_goals=[]
goal_dict=dict()
place_name=""

#初始化节点
rospy.init_node('smartcar_test',anonymous=False)
square_size = 1.0
cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=5) #实例化一个消息发布函数
move_base = actionlib.SimpleActionClient("move_base", MoveBaseAction) #action服务器连接

goal_arrive_pub = rospy.Publisher('/goal_arrive',String,queue_size=1)
xf_pub = rospy.Publisher('xf_launch', String, queue_size=1) #实例化一个消息发布函数


#end_pub = rospy.Publisher('/move_base_simple/goal_arrive',Int8,queue_size=1)


#设置参数 
rospy.loginfo('等待move_base action服务器连接...')
move_base.wait_for_server(rospy.Duration(30))
rospy.loginfo('已连接导航服务')

yaml_path = os.path.dirname(os.path.abspath(inspect.getfile(inspect.currentframe()))) + "/goal.yaml"
#添加导航坐标点,输入x（前）坐标，y（左）坐标，th（平面朝向0～360度）
def nav_to(x,y,th):
    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id='map'
    goal.target_pose.header.stamp = rospy.Time.now()
    goal.target_pose.pose = pose_e(x,y,th)
    move(goal)

def get_state(timeout):
    start_time = rospy.Time.now().to_sec()
    while  not rospy.is_shutdown():
        state = move_base.get_state()
        #print(str(state))
        now_time = rospy.Time.now().to_sec()
        if state == 3:
           return True
        if (now_time-start_time)>timeout:
           return False

#写一个函数 用于任务完成提示。
def move(goal):
    move_base.send_goal(goal)
    if get_state(18):
        rospy.loginfo(place_name+'导航成功！')
    else:
        while not rospy.is_shutdown():
            rospy.loginfo('时间超时，进入恢复状态，重新导航。')
            move_for(0,-1.2,1)
            move_base.send_goal(goal)
            if get_state(8):
                break
    
def shutdown():
    rospy.loginfo('机器人任务停止')
    move_base.cancel_goal()
    rospy.sleep(2)
    cmd_vel_pub.publish(Twist)
    rospy.sleep(1)
    
def pose_e(x,y,th):#输入x（前）坐标，y（左）坐标，th（平面朝向0～360度）
    new_pose=Pose()
    new_pose.position.x=float(x)
    new_pose.position.y=float(y)
    #机器朝向，平面朝向弧度转化成四元数空间位姿
    q=quaternion_from_euler(0.0,0.0,float(th)/180.0*pi)
    new_pose.orientation.x=q[0]
    new_pose.orientation.y=q[1]
    new_pose.orientation.z=q[2]
    new_pose.orientation.w=q[3]
    return  new_pose


def yaml_read():

  global goal_dict
  global nav_goals

  with open(yaml_path, 'r') as f:
    if not f:
      print("Open YAML file failed!")
      return
      
    goal_dict = yaml.load(f, Loader=yaml.FullLoader)  
    nav_goals = goal_dict.keys()




def nav_callback(data):
    g=data.data
    goal=g.decode("utf-8")
    if goal in nav_goals:
        goal_data=goal_dict[goal]
        nav_to(goal_data[0],goal_data[1],goal_data[2])




def nav_place(place):
    global place_name
    place_name=place
    print("尝试导航去:"+place_name)
    goal=place
    if goal in nav_goals:
        goal_data=goal_dict[goal]
        nav_to(goal_data[0],goal_data[1],goal_data[2])
        
        

def move_for(xspeed,tspeed,time_second):
    twist_data=Twist()
    twist_data.linear.x=xspeed
    twist_data.angular.z=tspeed
    time_start=time.time()
    while time.time()-time_start<time_second:
        cmd_vel_pub.publish(twist_data)
    cmd_vel_pub.publish(Twist())

def move_forward_x(xspeed,time_second):
    twist=Twist()
    twist.linear.x=xspeed
    time_start=time.time()
    while time.time()-time_start<time_second:
        cmd_vel_pub.publish(twist)
    cmd_vel_pub.publish(Twist())

def move_forward_y(yspeed,time_second):
    twist=Twist()
    twist.linear.y=yspeed
    time_start=time.time()
    while time.time()-time_start<time_second:
        cmd_vel_pub.publish(twist)
    cmd_vel_pub.publish(Twist())

# def msg_sub():
#     rospy.Subscriber("/darknet_result",darknet,darknet_callback,queue_size=1)
#     rospy.Subscriber("/start_move",String,start_callback,queue_size=1)

def thread_job():
    rospy.spin() 

def start_callback():
    start=1

def publish_arrive_id(id):
    #goal_arrive_pub = rospy.Publisher('/goal_arrive',String,queue_size=1)
    arrvie_id = String()
    arrvie_id.data = id
    goal_arrive_pub.publish(arrvie_id)

#发布区域信号进行截图拍照
def capture_image(id):
    publish_arrive_id(id)
    print("已发布“{}”信号,开始截图!".format(id))
    while not rospy.is_shutdown():
        if(os.path.exists("/home/ucar/scau_sim_sr/src/sr_pkg/darknet/{}.jpg".format(id))):
            break
    rospy.sleep(1) 


#视觉处理
def darknet_callback(data):
    print("视觉信息回调处理中!")
    # 取最大值
    values = {
        'E': {'cuv': data.E_cuv , 'riv': data.E_riv, 'whv': data.E_whv, 'cov': data.E_cov},
        'D': {'cuv': data.D_cuv, 'riv': data.D_riv, 'whv': data.D_whv, 'cov': data.D_cov},
        'C': {'cuv': data.C_cuv, 'riv': data.C_riv, 'whv': data.C_whv, 'cov': data.C_cov},
        'B': {'cuv': data.B_cuv, 'riv': data.B_riv, 'whv': data.B_whv, 'cov': data.B_cov},
        'F': {'cuf': data.F_cuf, 'waf': data.F_waf, 'cof': data.F_cof}
    }
 

    # 找到每个区域中值最大的键
    darknet_max_results = {}
    for key, value_dict in values.items():
        if key == 'F':
            # 对'F'键进行特别处理，如果最大值小于3，则设置为3
            max_key = max(value_dict, key=value_dict.get)
            max_value = max(value_dict.values())
            if max_value < 3:
                darknet_max_results[key] = {max_key: 3}
            else:
                darknet_max_results[key] = {max_key: max_value}
        else:
            # 对其他键，只返回最大值对应的键
            if sum(value_dict.values()) != 0:
                # 存在非零结果,找到最大值
                darknet_max_results[key] = max(value_dict, key=value_dict.get)
            else:
                # 保持默认的cuv
                darknet_max_results[key] = 'cuv'  # 修改此行

    E_result = darknet_max_results['E']
    D_result = darknet_max_results['D']
    C_result = darknet_max_results['C']
    B_result = darknet_max_results['B']
    F_result = darknet_max_results['F']

    print("视觉处理完成!")
    print("B区域:", B_result)   
    print("C区域:", C_result)
    print("D区域:", D_result)
    print("E区域:", E_result)
    print("F区域:", F_result)

    # 结束程序
    rospy.signal_shutdown("结束运行") 
    sys.exit()



if __name__ == "__main__":
    global E_result
    global D_result
    global C_result
    global B_result  
    global F_result

    E_result = None
    D_result = None
    C_result = None
    B_result = None
    F_result = None


   
    yaml_read()
    # shutil.rmtree("/home/luo/scau_sim_sr/src/sr_pkg/darknet",ignore_errors=True)
    # os.mkdir("/home/luo/scau_sim_sr/src/sr_pkg/darknet")
    # msg_sub()
    add_thread = threading.Thread(target = thread_job)
    add_thread.start()

    # xf_data = String()
    # xf_data.data = 'Y'
    # xf_pub.publish(xf_data)
    # start=0

    # rospy.Subscriber("/start_move", String, start_callback,queue_size=10)

    # while True:
    #     if start==1:
    #         break
    # xfCallback= rospy.Subscriber("/start_move", String, queue_size=10)

    # print("等待语音启动!")
    # while(launch_flag==False & & ros::ok()){
    #     ros::spinOnce()
    # }
    # print("等待启动语音!")
    # sub = None
    # while True:
    #         sub = rospy.Subscriber("/start_move", String, queue_size=10)
    #         if sub != None:
    #             print("已接收/start_move")
    #             break
    # rospy.sleep(1)
    # # 退出循环后取消订阅  
    # sub.unregister()


#     print("Wait for start!!!")
#     move_forward_x(1.0,1.0)
# #E 
    # nav_place("E1") 
    # capture_image("E1")
    # nav_place("E2")
    # capture_image("E2")
# #D
    
    print("图片收集完成，开始视觉处理!")
    publish_arrive_id("begin_yolo")
    rospy.Subscriber("/darknet_result",darknet,darknet_callback,queue_size=1)
 
    # nav_place("A0")

    # 导航完成,结束程序
    # rospy.signal_shutdown("结束运行") 
    # sys.exit()

    
