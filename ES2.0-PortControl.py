import os
import copy
import time
from _ctypes import pointer
from ctypes import c_float, POINTER, c_int, cast

import serial
import math
import binascii

HEADER_SIZE = 52
MAGIC_WORD = "0201040306050807"
SINGLE_POINT_DATA_SIZE = 20

class Person:
    def __init__(self, pos_x, pos_y, pos_z, vel_x, vel_y, vel_z):
        self.pos_x = pos_x
        self.pos_y = pos_y
        self.pos_z = pos_z
        self.vel_x = vel_x
        self.vel_y = vel_y
        self.vel_z = vel_z

class ReceivePointData :

    def __init__(self):
        self.data_buffer = []
        self.data_port = serial.Serial()
        self.user_port = serial.Serial()

    def open_port(self, data_port, user_port):
        self.data_port.port = data_port # 串口号
        self.data_port.baudrate = 921600  # 波特率
        self.data_port.bytesize = 8  # 数据位
        self.data_port.stopbits = 1  # 停止位
        self.data_port.parity = "N"  # 校验位
        self.data_port.open()
        try:
            if self.data_port.isOpen():
                print("数据串口打开成功")
            else:
                print("数据串口打开失败")
        except Exception as e:
            print(e)

        self.user_port.port = user_port  # 串口号
        self.user_port.baudrate = 115200  # 波特率
        self.user_port.bytesize = 8  # 数据位
        self.user_port.stopbits = 1  # 停止位
        self.user_port.parity = "N"  # 校验位
        self.user_port.open()
        try:
            if self.user_port.isOpen():
                print("用户串口打开成功")
            else:
                print("用户串口打开失败")
        except Exception as e:
            print(e)

    def send_config(self, path):
        try:
            with open(path, "r") as file:
                for line in file.readlines():
                    print("send config: " + line)
                    self.user_port.write(line.encode("utf-8"))
                    self.user_port.write("\n".encode("utf-8"))
                    time.sleep(0.2)
        except Exception as e:
            print(e)
            return

    def receive_data(self):
        point_data_list = []
        while True:
            if self.data_port is not None and self.data_port.isOpen():
                try:
                    if self.data_port.in_waiting:
                        buffer = str(self.data_port.read(self.data_port.in_waiting))[2:-1]
                        self.data_buffer.extend(buffer)
                        self.process_data()
                    else:
                        print(1)
                except Exception as e:
                    print(e)

    # 缓冲数据达到一定大小后进行数据处理
    def process_data(self):
        while len(self.data_buffer) >= HEADER_SIZE:
            # 从数据缓冲区获取一帧数据
            frame_data = self.get_frame()
            if frame_data is None:
                return
            if len(frame_data) == HEADER_SIZE * 2:
                print("空数据帧")
                continue

            # 解析TLV头部
            index  = HEADER_SIZE * 2
            tlv_type = int(self.convert_string("".join(frame_data[index: index+8])), 16)
            index += 8
            point_cloud_len = int(self.convert_string("".join(frame_data[index: index+8])), 16)
            index += 8
            point_num = point_cloud_len // SINGLE_POINT_DATA_SIZE
            if tlv_type == 6:
                print("point_num: %s", point_num)
            else:
                print("tlv_type:  %s", tlv_type)



    # 从数据缓冲区中获取一帧数据
    def get_frame(self):
        # 查找MAGIC_WORD
        start_index = self.data_buffer.index(MAGIC_WORD)
        if start_index == -1:
            return None
        # 去除MAGIC_WORD
        self.data_buffer = self.data_buffer[start_index:]
        start_index = 0
        if len(self.data_buffer) < HEADER_SIZE:
            return None
        # 获取数据长度
        packet_len = int(self.convert_string("".join(self.data_buffer[start_index + 40: start_index + 48])), 16)
        if packet_len > 10000:
            print("数据报大小超过10000，丢弃该帧")
            self.data_buffer = self.data_buffer[24:]
            return None
        # 数据实际长度不足期望长度，继续接受数据
        if len(self.data_buffer) < packet_len:
            return None
        # 数据实际长度满足期望长度，读取数据并返回
        ret = copy.deepcopy(self.data_buffer[start_index: start_index+packet_len*2])
        del self.data_buffer[start_index : start_index+packet_len*2]
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