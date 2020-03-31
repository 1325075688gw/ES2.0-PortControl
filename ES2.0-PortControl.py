import os
import copy
import time
import json
import serial
import math
import binascii

from _ctypes import pointer
from ctypes import c_float, POINTER, c_int, cast
from collections import OrderedDict

HEADER_SIZE = 52
MAGIC_WORD = "0201040306050807"
SINGLE_POINT_DATA_SIZE = 20
SINGLE_TARGET_LIST_SIZE = 216
SINGLE_TARGET_INDEX_SIZE = 1


class Target:
    def __init__(self, tid, pos_x, pos_y, pos_z, vel_x, vel_y, vel_z, acc_x, acc_y, acc_z, ec, g, index_no):
        self.tid = tid
        self.pos_x = pos_x
        self.pos_y = pos_y
        self.pos_z = pos_z
        self.vel_x = vel_x
        self.vel_y = vel_y
        self.vel_z = vel_z
        self.acc_x = acc_x
        self.acc_y = acc_y
        self.acc_z = acc_z
        self.ec = ec
        self.g = g
        self.index_no = index_no

class Point:
    def __init__(self, pid, x, y, z):
        self.pid = pid
        self.x = x
        self.y = y
        self.z = z


class ReceivePointData:

    def __init__(self, data_port="COM3", user_port="COM4"):
        """
        初始化数据串口和用户串口
        :param data_port:
        :param user_port:
        """
        self.data_buffer = []  #
        '''
        port=串口号, 
        baudrate=波特率, 
        bytesize=数据位, 
        stopbits=停止位, 
        parity=校验位
        '''
        self.data_port = serial.Serial(port=data_port, baudrate=921600, bytesize=8, stopbits=1, parity="N")
        self.user_port = serial.Serial(port=user_port, baudrate=921600, bytesize=8, stopbits=1, parity="N")

    def open_port(self):
        """
        打开串口
        :param data_port:
        :param user_port:
        :return:
        """
        try:
            self.data_port.open()
            print("数据串口打开成功")
        except Exception as e:
            print("串口打开失败\n %s", e)

        try:
            self.user_port.open()
            print("用户串口打开成功")
        except Exception as e:
            print("用户串口打开失败\n %s", e)

    def send_config(self, path):
        """
        初始化雷达板子
        :param path:
        :return:
        """
        current_dir = os.path.dirname(__file__)
        file = open(current_dir + "/mmw_pplcount_demo_default.cfg", "r+")
        if file is None:
            print("配置文件不存在!")
            return
        for text in file.readlines():
            print("send config:" + text)
            self.user_port.write(text.encode('utf-8'))
            self.user_port.write('\n'.encode('utf-8'))
            time.sleep(0.2)
        file.close()

    def receive_data(self):
        """
        接收串口数据
        :return: 
        """
        point_cloud_list = []
        count = 0
        point_cloud_json = OrderedDict()
        target_json = OrderedDict()
        while True:
            if self.data_port is not None and self.data_port.isOpen():
                try:
                    if self.data_port.in_waiting:
                        # 读取串口数据到临时缓存
                        buffer = str(self.data_port.read(self.data_port.in_waiting))[2:-1]
                        # 将临时缓存区数据添加到data_buffer
                        self.data_buffer.extend(buffer)
                        # 数据处理
                        point_cloud = self.process_data()
                        point_cloud_list.extend(point_cloud)
                        count += 1
                        if point_cloud:
                            point_cloud_json.update({count: point_cloud})

                except Exception as e:
                    print(e)
                finally:
                    with open("PointCloud.json", "w") as file:
                        json.dump(point_cloud_json, file)
            time.sleep(0.01)

    def process_data(self):
        """
        缓冲区达到一定数量之后进行数据处理
        :return: 
        """

        point_cloud_list = []
        while len(self.data_buffer) >= HEADER_SIZE:
            # 从数据缓冲区获取一帧数据
            frame_data = self.get_frame()
            if frame_data is None:
                return
            if len(frame_data) == HEADER_SIZE * 2:
                print("缓冲区正在接收数据，请稍后...")
                continue

            # 每一帧我们都需要先将头部取出
            # 解析TLV头部
            index = HEADER_SIZE * 2
            tlv_type = int(self.convert_string("".join(frame_data[index: index + 8])), 16)
            index += 8
            point_cloud_len = int(self.convert_string("".join(frame_data[index: index + 8])), 16)
            point_num = point_cloud_len % SINGLE_POINT_DATA_SIZE
            if point_num != 0:
                print("point_cloud 缓存区正在接收数据，请稍后...")
                continue
            if tlv_type == 6:
                print("point_num: %s", point_num)
            else:
                print("TLV_type:  %s", tlv_type)
            for i in range(point_num):
                index += 8
                range = self.byte_to_float(self.convert_string("".join(frame_data[index:index + 8])))

                index += 8
                azimuth = self.byte_to_float(self.convert_string("".join(frame_data[index:index + 8])))

                index += 8
                elev = self.byte_to_float(self.convert_string("".join(frame_data[index:index + 8])))

                index += 8
                doppler = self.byte_to_float(self.convert_string("".join(frame_data[index:index + 8])))

                index += 8
                snr = self.byte_to_float(self.convert_string("".join(frame_data[index:index + 8])))

                x = range * math.cos(elev) * math.sin(azimuth)
                y = range * math.cos(elev) * math.cos(azimuth)
                z = range * math.sin(azimuth)

                point = Point(i + 1, x, y, z)
                point_cloud_list.append(point)

            # 每一帧后面的TLV，就不需要再计算HEADER了
            # 解析TLV头部
            index += 8
            tlv_type = int(self.convert_string("".join(frame_data[index: index + 8])), 16)
            index += 8
            target_list_len = int(self.convert_string("".join(frame_data[index: index + 8])), 16)
            target_list_num = target_list_len % SINGLE_TARGET_LIST_SIZE
            if target_list_num != 0:
                print("target_list 缓存区正在接收数据，请稍后...")
                continue
            print("TLV: %s", tlv_type)
            print("传递的聚类数：%s", target_list_num)
            target_list = []
            for i in range(target_list_num):
                index += 8
                tid = int(self.convert_string("".join(frame_data[index:index + 8])), 16)
                index += 8
                pos_x = byte_to_float(self.convert_string("".join(frame_data[index:index + 8])))
                index += 8
                pos_y = byte_to_float(self.convert_string("".join(frame_data[index:index + 8])))
                index += 8
                pos_z = byte_to_float(self.convert_string("".join(frame_data[index:index + 8])))
                index += 8
                vel_x = byte_to_float(self.convert_string("".join(frame_data[index:index + 8])))
                index += 8
                vel_y = byte_to_float(self.convert_string("".join(frame_data[index:index + 8])))
                index += 8
                vel_z = byte_to_float(self.convert_string("".join(frame_data[index:index + 8])))
                index += 8
                acc_x = byte_to_float(self.convert_string("".join(frame_data[index:index + 8])))
                index += 8
                acc_y = byte_to_float(self.convert_string("".join(frame_data[index:index + 8])))
                index += 8
                acc_z = byte_to_float(self.convert_string("".join(frame_data[index:index + 8])))
                index += 8
                ec = byte_to_float(self.convert_string("".join(frame_data[index:index + 32])))
                index += 32
                g = byte_to_float(self.convert_string("".join(frame_data[index:index + 8])))
                target = Target(tid, pos_x, pos_y, pos_z, vel_x, vel_y, vel_z, acc_x, acc_y, acc_z, ec, g, 0)
                target_list.append(target)

            # 每一帧后面的TLV，就不需要再计算HEADER了
            # 解析TLV头部
            index += 8
            tlv_type = int(self.convert_string("".join(frame_data[index: index + 8])), 16)
            index += 8
            target_index_len = int(self.convert_string("".join(frame_data[index: index + 8])), 16)
            target_index_num = target_index_len % SINGLE_TARGET_INDEX_SIZE
            if target_index_num != 0:
                print("target_index 缓存区正在接收数据，请稍后...")
                continue
            print("TLV: %s", tlv_type)
            for i in range(target_index_num):
                index += 8
                index_no = int(self.convert_string("".join(frame_data[index:index + 8])), 16)
                target_list[i].index_no = index_no
        
        return point_cloud_list


    def get_frame(self):
        """
        从数据缓冲区中获取一帧数据
        :return:
        """
        # 查找MAGIC_WORD
        start_index = self.data_buffer.index(MAGIC_WORD)
        if start_index == -1:
            return None
        # 去除MAGIC_WORD
        self.data_buffer = self.data_buffer[start_index+len(MAGIC_WORD):]
        start_index = 0
        if len(self.data_buffer) < HEADER_SIZE:
            return None
        # 获取数据长度
        packet_len = int(self.convert_string("".join(self.data_buffer[start_index + 40: start_index + 48])), 16)
        if packet_len > 60000:
            print("数据报大小超过60000，丢弃该帧")
            self.data_buffer = self.data_buffer[24:]
            return None
        # 数据实际长度不足期望长度，继续接受数据
        if len(self.data_buffer) < packet_len:
            return None
        # 数据实际长度满足期望长度，读取数据并返回
        ret = copy.deepcopy(self.data_buffer[start_index: start_index + packet_len * 2])
        del self.data_buffer[start_index: start_index + packet_len * 2]
        return ret

    def convert_string(self, string):
        try:
            # str1 = string[2:4] + string[0:2] + string[6:8] + string[4:6]
            str1 = string[6:8] + string[4:6] + string[2:4] + string[0:2]
            return str1
        except IndexError as idxerr:
            print(idxerr.__context__)

    def byte_to_float(self, s):
        i = int(s, 16)
        cp = pointer(c_int(i))
        fp = cast(cp, POINTER(c_float))
        return fp.contents.value


if __name__ == "__main__":
    pointData = ReceivePointData()
    pointData.open_port("COM4", "COM3")
    pointData.send_config("")
    pointData.receive_data()