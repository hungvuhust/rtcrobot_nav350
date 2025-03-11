#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import socket
from rtcrobot_interfaces.msg import Nav350Data, Nav350Reflector, Reflector, Nav350Mode, AddLandmark, EditLandmark
import math
import struct
# from rtcrobot_interfaces import enum_common as enum
# kien: because of transformations
# import tf2
import tf2_ros
import psycopg2
# import tf2_conversions
from tf2_ros import TransformBroadcaster
from std_msgs.msg import Bool, UInt8
from geometry_msgs.msg import TransformStamped, Twist
from nav_msgs.msg import Odometry
import numpy as np
# import rtcrobot_interfaces.mapdata as mapdata
import sys
# caution: path[0] is reserved for script path (or '' in REPL)
sys.path.insert(1, '/home/rtc02/robot_ws/src/rtcrobot_interfaces/rtcrobot_interfaces/')
import log
import enum_common as enum
import time
import traceback
from rclpy.executors import MultiThreadedExecutor
Code_method = 'sMN'
Code_read = 'sRN'
Code_write = 'sWN'
Code_start = '\x02'
Code_stop = '\x03'
Code_space = '\x20'


Cmd_getdata = 'mNPOSGetData'

Cmd_setmode = 'mNEVAChangeState'
Cmd_Layer = 'NEVACurrLayer'
Cmd_Refsize = 'NLMDReflSize'
Cmd_slidingMean = 'NPOSSlidingMean'
Mode_powerdown = '0'
Mode_standby = '1'
Mode_navigation = '4'
Sliding_Mean = '1'

# Cmd_getdata = 'sAN'
# Mr.Huy Code
Cmd_read_all_global_ID = 'mNLAYGetLayer'
Cmd_get_scan_data = 'mNLMDGetData'
Cmd_Method_Async = 'sMA'
Cmd_pose_landmark = 'mNLAYGetLandmark'
current_layer = '0'
Cmd_add_landmark = 'mNLAYAddLandmark'
Cmd_edit_landmark = 'mNLAYSetLandmark'
Cmd_delete_landmark = 'mNLAYDelLandmark'

Cmd_store_layout = 'mNLAYStoreLayout'

database="robotdb"
# server_ip = "192.168.0.86"
# server_ip="192.168.1.91"
server_ip="localhost"

# database = "postgres"
user="postgres"
password="rtc@123"
port=5432

Error_mark = 5

Position_Mode = [
    'initial positioning',
    'continuous positioning',
    'virtual positioning',
    'positioning stopped',
    'position invalid',
    'external'
]

Nav350_Error = [
    'no error',
    'wrong operating mode',
    'asynchrony Method terminated',
    'invalid data',
    'no position available',
    'timeout',
    'method already active',
    'general error'
]
nav_350 = Nav350Data()
pose_x = 0.0
pose_y = 0.0
pose_theta = 0.0
count_reflector = 0
pos_mode = ''
class Publisher_(Node):
    def __init__(self):
        super().__init__("pub_nav350_pose")
        self.pub_ = self.create_publisher(
                Nav350Data,
                'nav/pose', 10)
        self.pub_
        self.sub_vel = self.create_subscription(Twist,"cmd_vel", self.vel_callback, 10)
        self.velocity = Twist()
        self.publish_odom = self.create_publisher(Odometry, "odom", 10)
        self.odometry = Odometry()
        self.create_timer(0.05, self.spin_main)
        global nav_350, pose_x, pose_y, pose_theta, count_reflector, pos_mode
        self.map_frame_id = 'map'
        self.nav_child_id = 'nav_link'
        self.odom_child_id = 'odom'
        self.broadcaster = tf2_ros.TransformBroadcaster(self)
        self.trans = TransformStamped()
        self.trans_odom = TransformStamped()
        self.current_mode = ""
        self.current_reflector = 1
    def vel_callback(self, msg):
        self.velocity = msg
    def spin_main(self):
        global nav_350, pose_x, pose_y, pose_theta, count_reflector, pos_mode
        # print(pose_x)
        try:
            nav_350.x = pose_x
            nav_350.y = pose_y
            nav_350.theta = pose_theta
            nav_350.reflector = count_reflector
            nav_350.mode = pos_mode
            
            if (self.current_reflector != nav_350.reflector):
                if (nav_350.reflector < 3):
                    log.loginfo("[NAV350] Miss Reflector %s", nav_350.reflector)
                    self.current_reflector = nav_350.reflector
                else:
                    self.current_reflector = nav_350.reflector
            if (self.current_mode != nav_350.mode):
                if (self.current_mode != "Continous positioning"):
                    log.loginfo("[NAV350] Error Mode %s", nav_350.mode)
                    self.current_mode = nav_350.mode
                else:
                    self.current_mode = nav_350.mode
            
            # print(nav_350.mode)
            # if ((nav_350.mode) != "Continous positioning"):
            #     log.loginfo("[NAV350] Error Mode %s", nav_350.reflector)
                
            self.pub_.publish(nav_350)
            self.broadcast_tf()
        except Exception as e:
            print("[nav350] Error: %s", e)
    def get_quaternion_from_euler(self, roll, pitch, yaw):
        """
        Convert an Euler angle to a quaternion.
        
        Input
            :param roll: The roll (rotation around x-axis) angle in radians.
            :param pitch: The pitch (rotation around y-axis) angle in radians.
            :param yaw: The yaw (rotation around z-axis) angle in radians.
        
        Output
            :return qx, qy, qz, qw: The orientation in quaternion [x,y,z,w] format
        """
        qx = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
        qy = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
        qz = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
        qw = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
    
        return [qx, qy, qz, qw]
    
    def broadcast_tf(self):
        try:
            # print("oke")
            global nav_350, pose_x, pose_y, pose_theta, count_reflector, pos_mode
            self.trans.header.stamp = self.get_clock().now().to_msg()
            self.trans.header.frame_id = self.map_frame_id
            self.trans.child_frame_id = self.nav_child_id
            self.trans.transform.translation.x = -pose_x
            self.trans.transform.translation.y = -pose_y
            self.trans.transform.translation.z = 1.9753
            # if self.pos_theta >= 0 and self.pos_theta <= math.pi:
            #     self.pos_theta = self.pos_theta - math.pi
            # elif self.pos_theta < 0 and self.pos_theta >= -math.pi:
            #     self.pos_theta = math.pi + self.pos_theta
            q = self.get_quaternion_from_euler(0, 0, pose_theta)
            self.trans.transform.rotation.x = q[0]
            self.trans.transform.rotation.y = q[1]
            self.trans.transform.rotation.z = q[2]
            self.trans.transform.rotation.w = q[3]
            self.broadcaster.sendTransform(self.trans)
            
            self.trans_odom.header.stamp = self.get_clock().now().to_msg()
            self.trans_odom.header.frame_id = self.map_frame_id
            self.trans_odom.child_frame_id = self.odom_child_id
            self.trans_odom.transform.translation.x = -pose_x
            self.trans_odom.transform.translation.y = -pose_y
            self.trans_odom.transform.translation.z = 1.9753
            # if self.pos_theta >= 0 and self.pos_theta <= math.pi:
            #     self.pos_theta = self.pos_theta - math.pi
            # elif self.pos_theta < 0 and self.pos_theta >= -math.pi:
            #     self.pos_theta = math.pi + self.pos_theta
            q = self.get_quaternion_from_euler(0, 0, pose_theta)
            self.trans_odom.transform.rotation.x = q[0]
            self.trans_odom.transform.rotation.y = q[1]
            self.trans_odom.transform.rotation.z = q[2]
            self.trans_odom.transform.rotation.w = q[3]
            self.broadcaster.sendTransform(self.trans_odom)
            
            self.odometry.header.stamp = self.get_clock().now().to_msg()
            self.odometry.header.frame_id = ''
            self.odometry.child_frame_id = ''
            self.odometry.pose.pose.position.x = -pose_x
            self.odometry.pose.pose.position.y = -pose_y
            self.odometry.pose.pose.position.z = 1.9753
            self.odometry.pose.pose.orientation.x = q[0]
            self.odometry.pose.pose.orientation.y = q[1]
            self.odometry.pose.pose.orientation.z = q[2]
            self.odometry.pose.pose.orientation.w = q[3]
            self.odometry.twist.twist.linear.x = self.velocity.linear.x
            self.odometry.twist.twist.linear.y = self.velocity.linear.y
            self.odometry.twist.twist.angular.z = self.velocity.angular.z
            self.publish_odom.publish(self.odometry)
            
            
        except Exception as ex:
            log.loginfo('[NAV350]: %s', ex)
            
class NAV350(Node):

    def __init__(self):
        super().__init__("Nav350_Driver_dkm")
        self.nav_address = '192.168.17.71'
        self.nav_port = 2111
        self.nav350 = Nav350Data()
        self.nav350_get_all = self.create_publisher(Reflector,"all_reflector", 10)
        self.nav350_get_current = self.create_publisher(Reflector,"current_reflector", 10)
        self.nav350_mode = self.create_subscription(Nav350Mode,"nav350_mode", self.mode_callback, 10)
        
        self.nav350_add = self.create_subscription(AddLandmark,"add_landmark", self.add_callback, 10)
        self.nav350_delete = self.create_subscription(UInt8,"delete_landmark", self.delete_callback, 10)
        self.nav350_edit = self.create_subscription(EditLandmark,"edit_landmark", self.edit_callback, 10)
        
        self.add_landmark_ = AddLandmark()
        self.delete_landmark_ = UInt8()
        self.edit_landmark_ = EditLandmark()
        
        self.data_setMode = ""
        self.data_data = ""
        self.ros_Rate = 20
        self.nav_layer = 0
        self.count_publishpose = 0
        self.pos_mode = 0
        self.socket_error_flag = 0
        self.count_to_error_connect = 0
        self.count_losing_mirror = 0
        self.chek_sick = 0
        self.count_nav = 0
        
        self.gloabal_ID = []
        # log.loginfo(self.nav_address)
        
        # kien: for broadcaster TF
        
        self.lst_pos_x = []
        self.count_pos_x = 0
        self.flag = 0
        self.sub_flag = 0
        global nav_350, pose_x, pose_y, pose_theta, count_reflector, pos_mode
        
        self.flag_record = 0
        self.result_socket = 0
        log.loginfo("NAV350 Starting")
        log.loginfo('[NAV350] Spin started')
        self.create_timer(0.1, self.spin)
        self.create_publisher
        
    def mode_callback(self, msg):
        self.flag = msg.mode
        self.sub_flag = msg.sub_mode
    def vel_callback(self, msg):
        self.velocity = msg
    def spin(self):
        a1 = time.time()

        self.result_socket = self.socketConnect()
        # print(self.flag)
        # print(self.sub_flag)
        if (self.flag_record == 0):
            self.all_reflector = self.get_all_landmark_in_layer(current_layer)
            print(self.all_reflector)
            for i in self.all_reflector:
                print(i)
                self.get_pose_landmark(i)
                time.sleep(0.1)
                
            self.flag_record = 1
        if (self.result_socket == 1):
            
            self.count_to_error_connect = 0
            #GUI MODE XUONG THEO FLAG
            # 0, 4: NAVIGATION
            # 1: STANDBY
            if (self.flag == 4 or self.flag == 0):
                # print("Mode",self.flag)
                self.result_setMode = self.nav350_setMode(Mode_navigation)
                #SET USER LEVEL
                self.result_setAccess = self.nav350_setUserLevel()
            
                # SET CURRENT LAYER
                self.nav350_setCurrentLayer(self.nav_layer)
                
                
                self.nav350_getCurrentLayer()
                self.setRefsize(28)
                self.setSlidingMean(Sliding_Mean)
                self.getRefsize()
                self.getSlidingMean()
                self.flag = 5
                time.sleep(0.1)
            elif (self.flag == 1):
                self.result_setMode = self.nav350_setMode(Mode_standby)
                if (self.sub_flag == 1):
                    self.add_landmark(self.add_landmark_.x, self.add_landmark_.y, self.add_landmark_.landmark_type, self.add_landmark_.reflector_type, self.add_landmark_.size, self.add_landmark_.layer)
                    self.save_permanent()
                    self.flag = 5
                if self.sub_flag == 2:
                    self.delete_landmark(self.delete_landmark_.data)
                    self.save_permanent()
                    self.flag = 5
                if (self.sub_flag == 3):
                    self.edit_landmark(self.edit_landmark_.global_id, self.edit_landmark_.x, self.edit_landmark_.y, self.edit_landmark_.landmark_type, self.edit_landmark_.reflector_type, self.edit_landmark_.size, self.edit_landmark_.layer)
                    self.save_permanent()
                    self.flag = 5
            
            # CHECK THEO MODE
            if (self.result_setMode == Mode_standby + Code_stop):
                #SET USER LEVEL
                self.result_setAccess = self.nav350_setUserLevel()
            
                # SET CURRENT LAYER
                self.nav350_setCurrentLayer(self.nav_layer)
                print("ok")
                # self.add_landmark()
                self.publish_error(0)
                self.flag = 5
                self.result_setMode = 5
        
                
                # print("Mode",self.flag)
            
                
            #     if (self.nav350_getPoseData()):
            #         self.publish_error(0)
            # else:
            #     self.publish_error(enum.Nav350_Error.SetMode_Fail.value)
            #     flag = 2
        else:
            self.count_to_error_connect += 1
            self.count_publishpose = 0
        if (self.result_setMode == Mode_navigation + Code_stop):
            # print(self.result_setMode)
            if (self.nav350_getPoseData()):
                self.publish_error(0)
        
        
        if (self.count_publishpose == 50):
            log.loginfo("[NAV350] Get Data OK %s", self.count_publishpose)
            log.loginfo("[NAV350] NAV350 Init Successfull :))")
        a2 = time.time()
        # print(a2-a1)
        # rate.sleep()
        # time.sleep(0.05)
    def add_callback(self, msg):
        self.add_landmark_ = msg
    def edit_callback(self, msg):
        self.edit_landmark_ = msg
    def delete_callback(self, msg):
        self.delete_landmark_ = msg
    
    def insert_reflector_data(self, global_id, current_layer, x, y, theta):
        try:
            self.connection = psycopg2.connect(
                host=server_ip,
                database=database,
                user=user,
                port=port,
                password=password)
            # print(self.cursor)
            if self.connection:
                # Create a cursor to perform database operations
                self.cursor = self.connection.cursor()
                self.cursor.execute("UPDATE public.reflector SET current_layer = " + (current_layer)+", x = " + str(x) + ", y = " + str(y) + ", theta = " + str(theta) + " WHERE global_id = " + str(self.convert_pos_data(global_id)) + ";")
                self.connection.commit()
                print("Record successfully into maps table")

                return True
            else:
                return False
        except (Exception, psycopg2.Error) as error:
            print("Failed to insert record into map table", error)
            return False
    def socketConnect(self, sock=None):
        if sock is None:
            try:
                self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                socket.setdefaulttimeout(3.0)
                # self.publish_error(enum.Nav350_Error.ConnectSocket_Fail.value)
            except socket.error:
                log.loginfo('[NAV350] Error socket connect %s', traceback.format_exc())
            except Exception as exx:
                log.loginfo('[NAV350] Init socket error! %s',traceback.format_exc())
                self.publish_error(enum.Nav350_Error.ConnectSocket_Fail.value)
        else:
            self.sock = sock
            log.loginfo('[NAV350] Socket already exist :)')
            # return 0
        try:
            self.sock.connect((self.nav_address, self.nav_port))
            
            return 1
        except socket.timeout as exx:
            if (self.count_to_error_connect > Error_mark):
                self.publish_error(enum.Nav350_Error.ConnectSocket_TimeOut.value)
                log.loginfo('[NAV350] Connect Timeout: %s', exx)
            else:
                log.loginfo('[NAV350] Count to connect error: %s', self.count_to_error_connect)
            return 0
        except Exception as exx:
            if (self.count_to_error_connect > Error_mark):
                # self.publish_error(enum.Nav350_Error.ConnectSocket_Exception.value)
                log.loginfo('[nav350] Connect Exception: %s', exx)
            else:
                log.loginfo('[NAV350] Count to connect error: %s', self.count_to_error_connect)
            return 0

    def socketSend(self, msg):
        try:
            self.sock.settimeout(0.5)
            self.sock.sendall(msg.encode())
            data_confirm = self.sock.recv(1024)
            if (data_confirm != None):
                data_confirm = data_confirm.split()
            if (data_confirm[0] == b'\x02sFA'):
                self.publish_error(enum.Nav350_Error.GetData_Error.value)
                log.loginfo("[NAV350] Get data error: %s", data_confirm)
            self.data_data = self.sock.recv(1024)
            self.socket_error_flag = 0
            self.count_nav = 0
            return self.data_data.decode()
        except BrokenPipeError as exx:
            self.socket_error_flag = 1
            self.count_nav = self.count_nav + 1
            log.loginfo("[NAV350]: %s", exx)
            if (self.count_nav > 10):
                log.loginfo('[NAV350] loi duong truyen socket')
                self.publish_error(enum.Nav350_Error.Get_BrokenPipeError.value)
        except Exception as e:
            self.socket_error_flag = 1
            self.count_nav = self.count_nav + 1
            log.loginfo("[NAV350] Error Socket: %s", e)
            if self.count_nav > 10:
                log.loginfo('[NAV350] loi socketSend 10 lan')
                self.publish_error(enum.Nav350_Error.SocketSend_Exception.value)

    def s_socketSend(self, msg):
        try:
            self.sock.settimeout(0.5)
            self.sock.sendall(msg.encode())
            data_confirm = self.sock.recv(1024)
            data_confirm = data_confirm[:-1]
            self.count_nav = 0
            self.socket_error_flag = 0
            return data_confirm.decode()
        except Exception as e:
            self.count_nav = self.count_nav + 1
            self.socket_error_flag = 1
            log.loginfo("[NAV350] error s_socket: %s", e)
            if self.count_nav > 10:
                log.loginfo("[NAV350] loi s_socketSend 10 lan")
                self.publish_error(enum.Nav350_Error.SocketSend_Exception.value)
    
    def getSlidingMean(self):
        i = 0
        while i < 11:
            try:
                self.data_getSlidingMean = self.s_socketSend(Code_start + Code_read + Code_space + Cmd_slidingMean + Code_stop)
                if (self.data_getSlidingMean != None and self.socket_error_flag == 0):
                    self.data_getSlidingMean = self.data_getSlidingMean.split()
                    # log.loginfo('[NAV350] get sliding mean done: %s', self.data_getSlidingMean[2])
                    # log.loginfo('[NAV350] sau khi convert: %s', self.convert_pos_data(self.data_getSlidingMean[2]))
                # if self.data_getRefsize[2] == '0':
                    return self.data_getSlidingMean[2]
                else:
                    i += 1
                if (i == 10):
                    log.loginfo('[NAV350] get sliding mean fail')
                    self.publish_error(enum.Nav350_Error.SetMode_Fail.value)
                    i = 0
            except Exception as exx:
                i += 1
                if (i == 10):
                    log.loginfo('[NAV350] get sliding mean fail: %s', exx)
                    self.publish_error(enum.Nav350_Error.SetMode_Fail.value)
                    return 0
        return 0
    def get_all_landmark_in_layer (self, layer):
        i = 0
        while i <11:
            try: 
                self.gloabal_ID = []
                self.get_all_reflector = self.s_socketSend(Code_start + Code_method + Code_space + Cmd_read_all_global_ID + Code_space + layer + Code_stop)
                if (self.get_all_reflector != None and self.socket_error_flag == 0):
                    self.get_all_reflector = self.get_all_reflector.split()
                    # print(self.get_all_reflector)
                    if (self.get_all_reflector[2] == 1):
                        log.loginfo('[NAV350] get all landmark in layer fail')
                        self.publish_error(enum.Nav350_Error.Invalid_Mode.value)
                    if (self.get_all_reflector[2] == 3):
                        log.loginfo('[NAV350] get all landmark in layer fail')
                        self.publish_error(enum.Nav350_Error.Invalid_Data.value)
                    if (self.get_all_reflector[2] == 7):
                        log.loginfo('[NAV350] get all landmark in layer fail')
                        self.publish_error(enum.Nav350_Error.Genere_Error.value)
                    for j in range(4,len(self.get_all_reflector)):
                        self.gloabal_ID.append((self.get_all_reflector[j]))
                    # print(self.gloabal_ID)
                    # print
                    return self.gloabal_ID
                else:
                    i += 1
                if (i == 10):
                    log.loginfo('[NAV350] get all landmark in layer fail')
                    self.publish_error(enum.Nav350_Error.SetMode_Fail.value)
                    i = 0
            except Exception as exx:
                i += 1
                if (i == 10):
                    log.loginfo('[NAV350] get all landmark in layer fail: %s', exx)
                    self.publish_error(enum.Nav350_Error.SetMode_Fail.value)
                    return []
    def get_pose_landmark(self, global_id):
        i = 0
        while i <11:
            try: 
                self.get_pose_reflector = self.s_socketSend(Code_start + Code_method + Code_space + Cmd_pose_landmark + Code_space + '1' +Code_space + global_id + Code_stop)
                if (self.get_pose_reflector != None and self.socket_error_flag == 0):
                    self.get_pose_reflector = self.get_pose_reflector.split()
                # print(self.get_pose_reflector)
                # print(self.convert_pos_data(self.get_pose_reflector[5])/1000)
                # print(self.convert_pos_data(self.get_pose_reflector[6])/1000)
                self.insert_reflector_data(global_id, current_layer, self.convert_pos_data(self.get_pose_reflector[5])/1000, self.convert_pos_data(self.get_pose_reflector[6])/1000, self.convert_theta(self.get_pose_reflector[7]))
                if (i == 10):
                    log.loginfo('[NAV350] get pose landmark fail')
                    self.publish_error(enum.Nav350_Error.SetMode_Fail.value)
                    i = 0
                return 1
            except Exception as exx:
                i += 1
                if (i == 10):
                    log.loginfo('[NAV350] get pose landmark fail: %s', exx)
                    self.publish_error(enum.Nav350_Error.SetMode_Fail.value)
                    return 1
    def add_landmark(self, x, y, landmark_type, reflector_type, size_of_reflector, layer_ID):
        i = 0
        while i <11:
            try: 
                log.loginfo('[NAV350] add landmark x:%s, y:%s, landmark_type: %s, reflector_type: %s, size_of_reflector: %s, layer: %s',x, y, landmark_type, (reflector_type), str(self.decimal_to_hexadecimal(size_of_reflector)), str(self.decimal_to_hexadecimal(layer_ID)))
                self.add_reflector = self.s_socketSend(Code_start + Code_method + Code_space + Cmd_add_landmark + Code_space + '1' + Code_space +str(x) + Code_space + str(y)  + Code_space + str(landmark_type)+ Code_space + str(reflector_type) +Code_space+ str(self.decimal_to_hexadecimal(size_of_reflector)) +Code_space+ '1' +Code_space + str(self.decimal_to_hexadecimal(layer_ID)) + Code_stop)
                                                                                                                        #Number of landmarks     #x                 #y            #Landmark type     #reflector type  #size of reflector  #layer IDs        #Layer ID
                if (self.add_reflector != None and self.socket_error_flag == 0):
                    self.add_reflector = self.add_reflector.split()
                print(self.add_reflector)
                print(self.convert_pos_data(self.add_reflector[-1]))
                if (i == 10):
                    log.loginfo('[NAV350] add landmark fail')
                    self.publish_error(enum.Nav350_Error.SetMode_Fail.value)
                    i = 0
                return self.convert_pos_data(self.add_reflector[-1])
            except Exception as exx:
                i += 1
                if (i == 10):
                    log.loginfo('[NAV350] add landmark fail: %s', exx)
                    self.publish_error(enum.Nav350_Error.SetMode_Fail.value)
                    return 1
    
    def edit_landmark(self,ID, x, y, landmark_type, reflector_type, size_of_reflector, layer_ID):
        i = 0
        while i <11:
            try: 
                log.loginfo('[NAV350] add landmark ID:%s, x:%s, y:%s, landmark_type: %s, reflector_type: %s, size_of_reflector: %s, layer: %s',str(self.decimal_to_hexadecimal(ID)),x, y, landmark_type, (reflector_type), (str(self.decimal_to_hexadecimal(size_of_reflector))), str(self.decimal_to_hexadecimal(size_of_reflector)))

                self.edit_reflector = self.s_socketSend(Code_start + Code_method + Code_space + Cmd_edit_landmark + Code_space + '1' + Code_space + str(self.decimal_to_hexadecimal(ID)) + Code_space +str(x) + Code_space + str(y)  + Code_space + str(landmark_type)+ Code_space + str(reflector_type) +Code_space+ str(self.decimal_to_hexadecimal(size_of_reflector)) +Code_space+ '1' +Code_space + str(self.decimal_to_hexadecimal(layer_ID)) + Code_stop)
                                                                                                                        #Number of landmarks     #x                 #y            #Landmark type     #reflector type  #size of reflector  #layer IDs        #Layer ID
                if (self.edit_reflector != None and self.socket_error_flag == 0):
                    self.edit_reflector = self.edit_reflector.split()
                print(self.edit_reflector)
                print(self.convert_pos_data(self.edit_reflector[-1]))
                if (i == 10):
                    log.loginfo('[NAV350] Edit landmark fail')
                    self.publish_error(enum.Nav350_Error.SetMode_Fail.value)
                    i = 0
                return self.convert_pos_data(self.edit_reflector[-1])
            except Exception as exx:
                i += 1
                if (i == 10):
                    log.loginfo('[NAV350] add landmark fail: %s', exx)
                    self.publish_error(enum.Nav350_Error.SetMode_Fail.value)
                    return 1

    def delete_landmark(self, ID):
        i = 0
        while i <11:
            try: 
                self.delete_reflector = self.s_socketSend(Code_start + Code_method + Code_space + Cmd_delete_landmark + Code_space + '1' + Code_space + str(self.decimal_to_hexadecimal(ID)) + Code_stop)
                                                                                                                        #Number of landmarks     #x                 #y            #Landmark type     #reflector type  #size of reflector  #layer IDs        #Layer ID
             
                if (self.delete_reflector != None and self.socket_error_flag == 0):
                    self.delete_reflector = self.delete_reflector.split()
                # print(self.delete_reflector[-1])
                else:
                    i += 1
                if (i == 10):
                    log.loginfo('[NAV350] add landmark fail')
                    self.publish_error(enum.Nav350_Error.SetMode_Fail.value)
                    i = 0
                if (self.delete_reflector[-1] == '0'):
                    log.loginfo('[NAV350] delete landmark success')
                    # self.publish_error(enum.Nav350_Error.SetMode_Fail.value)
                elif (self.delete_reflector[-1] == '1'):
                    log.loginfo('[NAV350] delete landmark fail')
                    self.publish_error(enum.Nav350_Error.Invalid_Mode.value)
                elif (self.delete_reflector[-1] == '3'):
                    log.loginfo('[NAV350] delete landmark fail')
                    self.publish_error(enum.Nav350_Error.Invalid_Data.value)
                elif (self.delete_reflector[-1] == '7'):
                    log.loginfo('[NAV350] delete landmark fail')
                    self.publish_error(enum.Nav350_Error.General_Error.value)
                return 0
            except Exception as exx:
                i += 1
                if (i == 10):
                    log.loginfo('[NAV350] add landmark fail: %s', exx)
                    self.publish_error(enum.Nav350_Error.SetMode_Fail.value)
                    return 1
    def save_permanent(self):
        i = 0
        while i < 11:
            try:
                self.store_layout = self.s_socketSend(Code_start + Code_method + Code_space + Cmd_store_layout + Code_stop)
                if (self.store_layout!= None and self.socket_error_flag == 0):
                    self.store_layout.split()
                    return 0
            except Exception as exx:
                i += 1
                if (i == 10):
                    log.loginfo('[NAV350] add landmark fail: %s', exx)
                    self.publish_error(enum.Nav350_Error.SetMode_Fail.value)
                    return 1
    def setSlidingMean(self, slidingMean):
        i = 0
        while i < 11:
            try:
                # refsize = hex(refsize)
                # log.loginfo('[nav350] convert refsize:', str(refsize))
                self.data_setSlidingMean = self.s_socketSend(Code_start + Code_write + Code_space + Cmd_slidingMean + Code_space + str(slidingMean) + Code_stop)
                if (self.data_setSlidingMean != None and self.socket_error_flag == 0):
                    self.data_setSlidingMean = self.data_setSlidingMean.split()
                    # log.loginfo('[NAV350] set sliding mean result: %s', self.data_setSlidingMean)
                    # log.loginfo('[nav350] sau khi convert:', self.convert_pos_data(self.data_setRefsize[2]))
                # if self.data_getRefsize[2] == '0':
                    return self.data_setSlidingMean
                else:
                    i += 1
                if (i == 10):
                    log.loginfo('[NAV350] Set sliding mean fail')
                    self.publish_error(enum.Nav350_Error.SetMode_Fail.value)
                    i = 0
            except Exception as exx:
                i += 1
                if (i == 10):
                    log.loginfo('[NAV350] Set sliding mean fail: %s', exx)
                    self.publish_error(enum.Nav350_Error.SetMode_Fail.value)
                    return 0
        return 0

    def getRefsize(self):
        i = 0
        while i < 11:
            try:
                self.data_getRefsize = self.s_socketSend(Code_start + Code_read + Code_space + Cmd_Refsize + Code_stop)
                if (self.data_getRefsize != None and self.socket_error_flag == 0):
                    self.data_getRefsize = self.data_getRefsize.split()
                    # log.loginfo('[NAV350] get Ref done: %s', self.data_getRefsize[2])
                    # log.loginfo('[NAV350] sau khi convert: %s', self.convert_pos_data(self.data_getRefsize[2]))
                # if self.data_getRefsize[2] == '0':
                    return self.data_getRefsize[2]
                else:
                    i += 1
                if (i == 10):
                    log.loginfo('[NAV350] get Ref fail')
                    self.publish_error(enum.Nav350_Error.SetMode_Fail.value)
                    i = 0
            except Exception as exx:
                i += 1
                if (i == 10):
                    log.loginfo('[NAV350] get Ref fail: %s', exx)
                    self.publish_error(enum.Nav350_Error.SetMode_Fail.value)
                    return 0
        return 0

    def setRefsize(self, refsize):
        i = 0
        while i < 11:
            try:
                # refsize = hex(refsize)
                # log.loginfo('[nav350] convert refsize:', str(refsize))
                self.data_setRefsize = self.s_socketSend(Code_start + Code_write + Code_space + Cmd_Refsize + Code_space + str(refsize) + Code_stop)
                if (self.data_setRefsize != None and self.socket_error_flag == 0):
                    self.data_setRefsize = self.data_setRefsize.split()
                    # log.loginfo('[NAV350] set Ref result: %s', self.data_setRefsize)
                    # log.loginfo('[nav350] sau khi convert:', self.convert_pos_data(self.data_setRefsize[2]))
                # if self.data_getRefsize[2] == '0':
                    return self.data_setRefsize
                else:
                    i += 1
                if (i == 10):
                    log.loginfo('[NAV350] set Ref fail')
                    self.publish_error(enum.Nav350_Error.SetMode_Fail.value)
                    i = 0
            except Exception as exx:
                i += 1
                if (i == 10):
                    log.loginfo('[NAV350] set Ref fail: %s', exx)
                    self.publish_error(enum.Nav350_Error.SetMode_Fail.value)
                    return 0
        return 0

    def nav350_setMode(self, mode):
        i = 0
        while i < 11:
            try:
                self.data_setMode = self.socketSend(Code_start + Code_method + Code_space + Cmd_setmode + Code_space + mode + Code_stop)
                if (self.data_setMode != None and self.socket_error_flag == 0):
                    self.data_setMode = self.data_setMode.split()
                if self.data_setMode[2] == '0':
                    return self.data_setMode[3]
                else:
                    i += 1
                if (i == 10):
                    log.loginfo('[NAV350] set mode fail')
                    self.publish_error(enum.Nav350_Error.SetMode_Fail.value)
                    i = 0
            except Exception as exx:
                i += 1
                if (i == 10):
                    log.loginfo('[NAV350] set mode fail: %s', exx)
                    self.publish_error(enum.Nav350_Error.SetMode_Fail.value)
                    return 0
        return 0

    def nav350_setPoseDataFormat(self):
        try:
            data_setPoseDataFormat = self.s_socketSend(Code_start + 'sWN' + Code_space + 'NPOSPoseDataFormat' + Code_space + '1' + Code_space + '1' + Code_stop)
            data_setPoseDataFormat.split()
        except Exception as exx:
            log.loginfo('[NAV350] %s',exx)

    def nav350_setUserLevel(self):
        i = 0
        while i < 11:
            try:
                data_setUserLevel = self.s_socketSend(Code_start + Code_method + Code_space + 'SetAccessMode' + Code_space + '03' + Code_space + 'F4724744' + Code_stop)
                if (data_setUserLevel !=  None and self.socket_error_flag == 0):
                    data_setUserLevel = data_setUserLevel.split()
                if self.convert_pos_data(data_setUserLevel[2]) == 1:
                    self.nav350_setPoseDataFormat()
                    return 1
                else:
                    i += 1
                if (i == 10):
                    log.loginfo('[NAV350] set user level fail')
                    self.publish_error(enum.Nav350_Error.SetUserLevel_Fail.value)
            except Exception as exx:
                i += 1
                if (i == 10):
                    log.loginfo('[NAV350] set user level: %s', exx)
                    self.publish_error(enum.Nav350_Error.SetUserLevel_Fail.value)
                    return 0
        return 0

    def nav350_setCurrentLayer(self, layer):
        i = 0
        while i < 11:
            try:
                data_setLayer = self.s_socketSend(Code_start + Code_write + Code_space + Cmd_Layer + Code_space + str(layer) + Code_stop)
                
                if (data_setLayer != None and self.socket_error_flag == 0):
                    data_setLayer = data_setLayer.split()
                if (data_setLayer[0] == Code_start + 'sWA'):
                    return True
                else:
                    i += 1
                if (i == 10):
                    log.loginfo('[NAV350] set layer fail')
                    self.publish_error(enum.Nav350_Error.SetLayer_Fail.value)
            except Exception as exx:
                i += 1
                if (i == 10):
                    log.loginfo('[NAV350] set current layer fail: %s', exx)
                    self.publish_error(enum.Nav350_Error.SetLayer_Fail.value)
                    return 0
        return False

    def nav350_getCurrentLayer(self):
        i = 0
        while i < 11:
            try:
                data_getLayer = self.s_socketSend(Code_start + Code_read + Code_space + Cmd_Layer + Code_stop)

                if (data_getLayer != None and self.socket_error_flag == 0):
                    data_getLayer = data_getLayer.split()
                    if len(data_getLayer) >=3:
                        nav_350.layer = self.convert_pos_data(data_getLayer[2])
                        # self.robotPose.publish(self.nav350)
                        return 1
                    else:
                        i += 1
                else:
                    i += 1
                if (i == 10):
                    log.loginfo("[NAV350] get current layer fail")
                    self.publish_error(enum.Nav350_Error.GetLayer_Exception.value)
            except Exception as exx:
                i += 1
                if (i == 10):
                    log.loginfo('[NAV350] get current layer fail: %s', exx)
                    self.publish_error(enum.Nav350_Error.GetLayer_Exception.value)
                    return 0
        return 0

    def nav350_getPoseData(self):
        i = 0
        global pose_x, pose_y, pose_theta, pos_mode, count_reflector
        while i < 11:
            try:
                data_getPose = self.socketSend(Code_start + Code_method + Code_space + Cmd_getdata + Code_space + '1' + Code_space + '0' + Code_stop)
                # print(data_getPose)
                # print("-------------------------------------------")
                if (data_getPose != None and self.socket_error_flag == 0):
                    data_getPose = data_getPose.split()
                    # print
                    # for i in range(len(data_getPose)-1):
                    #     print(i, (data_getPose[i]))
                    # print(int(data_getPose[3], 16))
                    if len(data_getPose) >= 16:
                        if (self.count_publishpose > 10 and data_getPose[9] != '0\x03'):
                            pose_x = self.convert_pos_data(data_getPose[7])/1000
                            # print("DKM", pose_x)
                            pose_y = self.convert_pos_data(data_getPose[8])/1000
                            pose_theta = self.convert_theta(data_getPose[9])
                            pos_mode = Position_Mode[self.convert_pos_data(data_getPose[14])]
                            count_reflector = self.convert_pos_data(data_getPose[16])

                            if (count_reflector < 3):
                                if (time.time() - self.count_losing_mirror >= 0.3):
                                    self.publish_error(13)
                                    return 0
                            else:
                                self.count_losing_mirror = time.time()
                              
                            
                            return 1
                        else:
                            self.count_publishpose += 1
                    
                else:
                    i += 1
                if (i == 10):
                    log.loginfo("[NAV350] get pose fail")
                    self.publish_error(enum.Nav350_Error.GetPose_Fail.value)
            except Exception as exx:
                i += 1
                if (i == 10):
                    log.loginfo('[NAV350] get pose fail: %s', exx)
                    self.publish_error(enum.Nav350_Error.GetPose_Fail.value)
        return 0

    

    def get_quaternion_from_euler(self, roll, pitch, yaw):
        """
        Convert an Euler angle to a quaternion.
        
        Input
            :param roll: The roll (rotation around x-axis) angle in radians.
            :param pitch: The pitch (rotation around y-axis) angle in radians.
            :param yaw: The yaw (rotation around z-axis) angle in radians.
        
        Output
            :return qx, qy, qz, qw: The orientation in quaternion [x,y,z,w] format
        """
        qx = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
        qy = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
        qz = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
        qw = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
    
        return [qx, qy, qz, qw]
    
    def convert_pos_data(self, data):
        if (len(data) >= 7):
            return -(0xffffffff - int(data, 16))
        else:
            return int(data, 16)
    def decimal_to_hexadecimal(self, decimal_num):
        hex_characters = "0123456789ABCDEF"
        if decimal_num == 0:
            return "0"
        
        hexadecimal = ""
        while decimal_num > 0:
            remainder = decimal_num % 16
            hexadecimal = hex_characters[remainder] + hexadecimal
            decimal_num //= 16
    
        return hexadecimal
    def convert_theta(self, data):
        theta = self.convert_pos_data(data)/1000
        if theta <= 180:
            return theta * (math.pi/180)
        elif theta > 180:
            return (theta - 360) * (math.pi/180)

    def publish_error(self, error_msg):
        nav_350.error = error_msg
        # self.robotPose.publish(self.nav350)

def main(args =None):
    rclpy.init(args=args)
    try:
        minimal_publisher = NAV350()
        pub = Publisher_()
        executor = MultiThreadedExecutor()
        executor.add_node(minimal_publisher)
        executor.add_node(pub)
        try: 
            executor.spin()
        finally:
            rclpy.shutdown()

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    # minimal_publisher.destroy_node()
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()