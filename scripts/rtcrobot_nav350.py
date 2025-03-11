


#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import socket
from rtcrobot_interfaces.msg import Nav350Data, Nav350Reflector, Reflector
import math
import struct
# from rtcrobot_interfaces import enum_common as enum
# kien: because of transformations
# import tf2
import tf2_ros
# import tf_conversions
from std_msgs.msg import Bool
from geometry_msgs.msg import TransformStamped
# import rtcrobot_interfaces.mapdata as mapdata
import sys
# caution: path[0] is reserved for script path (or '' in REPL)
sys.path.insert(1, '/home/rtc02/robot_ws/src/rtcrobot_interfaces/rtcrobot_interfaces/')
import log
import enum_common as enum
import time
import traceback

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
Sliding_Mean = '5'

# Mr.Huy Code
Cmd_read_all_landmarks = 'mNLAYGetLayout'


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

class NAV350(Node):

    def __init__(self):
        super().__init__("Nav350_Driver")
        self.nav_address = '192.168.17.71'
        self.nav_port = 2111
        self.nav350 = Nav350Data()
        self.robotPose = self.create_publisher(Nav350Data,"robot_pose", 10)
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
        # log.loginfo(self.nav_address)
        
        # kien: for broadcaster TF
        self.map_frame_id = 'map'
        self.nav_child_id = 'nav350'
        self.broadcaster = tf2_ros.TransformBroadcaster(self)
        self.trans = TransformStamped()
        self.lst_pos_x = []
        self.count_pos_x = 0
        log.loginfo("NAV350 Starting")
        self.create_timer(0.01, self.spin)
    def spin(self):
        # rate = rclpy.(self.ros_Rate)
        result_socket = 0
        flag = 0
        # log.loginfo('[NAV350] Spin started')
        # while not rclpy.shutdown():
        result_socket = self.socketConnect()
        print("DKm",result_socket)
        if (result_socket == 1):
            self.count_to_error_connect = 0
            if (flag == 0 or flag == 2):
                self.result_setAccess = self.nav350_setUserLevel()
                self.nav350_setCurrentLayer(self.nav_layer)
                self.result_setMode = self.nav350_setMode(Mode_navigation)
                self.nav350_getCurrentLayer()
                self.setRefsize(28)
                self.setSlidingMean(Sliding_Mean)
                self.getRefsize()
                self.getSlidingMean()
                
                flag = 1
            if (self.result_setMode == Mode_navigation + Code_stop):
                if (self.nav350_getPoseData()):
                    self.publish_error(0)
            else:
                self.publish_error(enum.Nav350_Error.SetMode_Fail.value)
                flag = 2
        else:
            self.count_to_error_connect += 1
            self.count_publishpose = 0
            if (flag == 1):
                flag = 2
        if (self.count_publishpose == 50):
            log.loginfo("[NAV350] Get Data OK %s", self.count_publishpose)
            log.loginfo("[NAV350] NAV350 Init Successfull :))")
        # rate.sleep()
        # time.sleep(0.05)

    def socketConnect(self, sock=None):
        if sock is None:
            try:
                self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                socket.setdefaulttimeout(0.5)
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
                # self.publish_error(enum.Nav350_Error.ConnectSocket_TimeOut.value)
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
                # self.publish_error(enum.Nav350_Error.GetData_Error.value)
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
                # self.publish_error(enum.Nav350_Error.Get_BrokenPipeError.value)
        except Exception as e:
            self.socket_error_flag = 1
            self.count_nav = self.count_nav + 1
            log.loginfo("[NAV350] Error Socket: %s", e)
            if self.count_nav > 10:
                log.loginfo('[NAV350] loi socketSend 10 lan')
                # self.publish_error(enum.Nav350_Error.SocketSend_Exception.value)

    def s_socketSend(self, msg):
        try:
            self.sock.settimeout(0.3)
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
                self.get_all_reflector = self.s_socketSend(Code_start + Code_read + Code_space + Cmd_read_all_landmarks + Code_stop)
                if (self.get_all_reflector != None and self.socket_error_flag == 0):
                    print("get all landmark in layer: ", self.get_all_reflector)
                    self.get_all_reflector = self.get_all_reflector.split()
                    return self.get_all_reflector
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
                    return 0
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
                        self.nav350.layer = self.convert_pos_data(data_getLayer[2])
                        self.robotPose.publish(self.nav350)
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
        while i < 11:
            try:
                data_getPose = self.socketSend(Code_start + Code_method + Code_space + Cmd_getdata + Code_space + '1' + Code_space + '0' + Code_stop)
                if (data_getPose != None and self.socket_error_flag == 0):
                    data_getPose = data_getPose.split()
                    if len(data_getPose) >= 16:
                        if (self.count_publishpose > 10 and data_getPose[9] != '0\x03'):
                            self.pos_x = self.convert_pos_data(data_getPose[7])/1000
                            self.pos_y = self.convert_pos_data(data_getPose[8])/1000
                            self.pos_theta = self.convert_theta(data_getPose[9])
                            self.pos_mode = Position_Mode[self.convert_pos_data(data_getPose[14])]
                            self.count_reflector = self.convert_pos_data(data_getPose[16])
                            if (self.count_reflector < 3):
                                if (time.time() - self.count_losing_mirror >= 0.3):
                                    self.publish_error(13)
                                    return 0
                            else:
                                self.count_losing_mirror = time.time()
                            self.nav350.x = self.pos_x
                            self.nav350.y = self.pos_y
                            self.nav350.theta = self.pos_theta
                            self.nav350.reflector = self.count_reflector
                            self.nav350.mode = self.pos_mode
                            self.robotPose.publish(self.nav350)
                            # self.broadcast_tf()
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

    def broadcast_tf(self):
        try:
            self.trans.header.stamp = rospy.Time.now()
            self.trans.header.frame_id = self.map_frame_id
            self.trans.child_frame_id = self.nav_child_id
            self.trans.transform.translation.x = -self.pos_x
            self.trans.transform.translation.y = -self.pos_y
            self.trans.transform.translation.z = 2.0
            # if self.pos_theta >= 0 and self.pos_theta <= math.pi:
            #     self.pos_theta = self.pos_theta - math.pi
            # elif self.pos_theta < 0 and self.pos_theta >= -math.pi:
            #     self.pos_theta = math.pi + self.pos_theta
            q = tf2_conversions.transformations.quaternion_from_euler(0, 0, self.pos_theta)
            self.trans.transform.rotation.x = q[0]
            self.trans.transform.rotation.y = q[1]
            self.trans.transform.rotation.z = q[2]
            self.trans.transform.rotation.w = q[3]
            self.broadcaster.sendTransform(self.trans)
        except Exception as ex:
            log.loginfo('[NAV350]: %s', ex)

    def convert_pos_data(self, data):
        if (len(data) >= 7):
            return -(0xffffffff - int(data, 16))
        else:
            return int(data, 16)

    def convert_theta(self, data):
        theta = self.convert_pos_data(data)/1000
        if theta <= 180:
            return theta * (math.pi/180)
        elif theta > 180:
            return (theta - 360) * (math.pi/180)

    def publish_error(self, error_msg):
        self.nav350.error = error_msg
        self.robotPose.publish(self.nav350)

def main(args =None):
    rclpy.init(args=args)

    minimal_publisher = NAV350()

    rclpy.spin(minimal_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()