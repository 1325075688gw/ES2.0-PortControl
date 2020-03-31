import os
import copy
import time
import json
import serial
import math
import binascii
import _thread


from _ctypes import pointer
from ctypes import c_float, POINTER, c_int, cast
from collections import OrderedDict

HEADER_SIZE = 52
MAGIC_WORD = "0201040306050807"
SINGLE_POINT_DATA_SIZE = 20
SINGLE_TARGET_LIST_SIZE = 40
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

    def __init__(self, data_port="COM4", user_port="COM3"):
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
        self.user_port = serial.Serial(port=user_port, baudrate=115200, bytesize=8, stopbits=1, parity="N")

    def open_port(self):
        """
        打开串口
        :param data_port:
        :param user_port:
        :return:
        """
        self.data_port.open()
        if self.data_port.isOpen():
            print("数据串口打开成功！")
        else:
            print("数据串口打开失败！")

        self.user_port.open()
        if self.user_port.isOpen():
            print("用户串口打开成功！")
        else:
            print("用户串口打开失败！")

    def send_config(self):
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
                        buffer = str(binascii.b2a_hex(self.data_port.read(self.data_port.in_waiting)))[2:-1]
                        # 将临时缓存区数据添加到data_buffer
                        valid_data = []
                        for i in range(len(buffer)):
                            valid_data.append((buffer[i]))
                        # print(valid_data)
                        self.data_buffer.extend(valid_data)
                        # 数据处理
                        point_cloud = self.process_data()
                        if point_cloud ==None:
                            print("fff")
                        else:
                            point_cloud_list.extend(point_cloud)
                            count += 1
                            print(point_cloud)
                            if point_cloud:
                                point_cloud_json.update({count: point_cloud})

                except Exception as e:
                    print(e)
                # finally:
                #     with open("PointCloud.json", "w") as file:
                #         json.dump(point_cloud_json, file)
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
            num_tlv = 1
            if frame_data is None:
                return
            if len(frame_data) < HEADER_SIZE * 2:
                print("缓冲区正在接收数据，请稍后...")
                continue
            else:
                num_tlv = int(self.convert_string("".join(frame_data[48 * 2: 48 * 2 + 4])), 16)
                total_packet_len = int(self.convert_string("".join(frame_data[20 * 2: 20 * 2 + 8])), 16)
            if len(frame_data) < total_packet_len * 2:
                continue

            # 每一帧我们都需要先将头部取出
            # 解析TLV头部
            index = HEADER_SIZE * 2
            tlv_type = int(self.convert_string("".join(frame_data[index: index + 8])), 16)
            index += 8
            point_cloud_len = int(self.convert_string("".join(frame_data[index: index + 8])), 16)
            point_cloud_num = int((point_cloud_len - 8) / SINGLE_POINT_DATA_SIZE)
            print("tlv_type: {0}, point_num: {1}".format(tlv_type, point_cloud_num))

            for i in range(point_cloud_num_ys):
                index += 8
                range2 = self.byte_to_float(self.convert_string("".join(frame_data[index:index + 8])))

                index += 8
                azimuth = self.byte_to_float(self.convert_string("".join(frame_data[index:index + 8])))

                index += 8
                elev = self.byte_to_float(self.convert_string("".join(frame_data[index:index + 8])))

                index += 8
                doppler = self.byte_to_float(self.convert_string("".join(frame_data[index:index + 8])))

                index += 8
                snr = self.byte_to_float(self.convert_string("".join(frame_data[index:index + 8])))

                x = range2 * math.cos(elev) * math.sin(azimuth)
                y = range2 * math.cos(elev) * math.cos(azimuth)
                z = range2 * math.sin(azimuth)

                point = Point(i + 1, x, y, z)
                point_cloud_list.append(point)
            if num_tlv == 3:
                # 每一帧后面的TLV，就不需要再计算HEADER了
                # 解析TLV头部
                index += 8
                tlv_type = int(self.convert_string("".join(frame_data[index:index + 8])), 16)
                index += 8
                target_list_len = int(self.convert_string("".join(frame_data[index: index + 8])), 16)
                target_list_num = int((target_list_len-8) / SINGLE_TARGET_LIST_SIZE)
                print("tlv_type: {0}, target_list_num: {1}".format(tlv_type, target_list_num))
                target_list = []
                for i in range(target_list_num):
                    index += 8
                    tid = int(self.convert_string("".join(frame_data[index:index + 8])), 16)
                    index += 8
                    pos_x = self.byte_to_float(self.convert_string("".join(frame_data[index:index + 8])))
                    index += 8
                    pos_y = self.byte_to_float(self.convert_string("".join(frame_data[index:index + 8])))
                    index += 8
                    pos_z = self.byte_to_float(self.convert_string("".join(frame_data[index:index + 8])))
                    index += 8
                    vel_x = self.byte_to_float(self.convert_string("".join(frame_data[index:index + 8])))
                    index += 8
                    vel_y = self.byte_to_float(self.convert_string("".join(frame_data[index:index + 8])))
                    index += 8
                    vel_z = self.byte_to_float(self.convert_string("".join(frame_data[index:index + 8])))
                    index += 8
                    acc_x = self.byte_to_float(self.convert_string("".join(frame_data[index:index + 8])))
                    index += 8
                    acc_y = self.byte_to_float(self.convert_string("".join(frame_data[index:index + 8])))
                    index += 8
                    acc_z = self.byte_to_float(self.convert_string("".join(frame_data[index:index + 8])))
                    # index += 8
                    # ec = self.byte_to_float(self.convert_string("".join(frame_data[index:index + 32])))
                    # index += 32
                    # g = self.byte_to_float(self.convert_string("".join(frame_data[index:index + 8])))
                    target = Target(tid, pos_x, pos_y, pos_z, vel_x, vel_y, vel_z, acc_x, acc_y, acc_z, 0, 0, 0)
                    target_list.append(target)

                # 每一帧后面的TLV，就不需要再计算HEADER了
                # 解析TLV头部
                index += 8
                tlv_type = int(self.convert_string("".join(frame_data[index: index + 8])), 16)
                index += 8
                target_index_len = int(self.convert_string("".join(frame_data[index: index + 8])), 16)
                target_index_num = int((target_index_len-8) / SINGLE_TARGET_INDEX_SIZE)
                print("tlv_type: {0}, target_index_num: {1}".format(tlv_type, target_index_num))
                index += 8
                for i in range(target_index_num_ys):
                    index_no = int(self.convert_string("".join(frame_data[index:index + 2])), 16)
                    index += 2
                    # target_list[i].index_no = index_no
                    if index_no >= 253:
                        continue
                    else:
                        print("该点属于 {0} 聚类".format(index_no))


        return point_cloud_list


    def get_frame(self):
        """
        从数据缓冲区中获取一帧数据
        :return:
        """
        data_str = "".join(self.data_buffer)
        # print("data_str:" + data_str)
        start_index = data_str.index("0201040306050807")
        if start_index == -1:
            return None
        start_index = int(start_index)
        del self.data_buffer[0:start_index]
        start_index = 0
        if len(self.data_buffer) < HEADER_SIZE:
            return None
        packet_len = int(self.convert_string("".join(self.data_buffer[start_index + 40:start_index + 48])), 16)
        print("数据包大小:" + str(packet_len))
        # if packet_len > 30000:
        #     print("数据包大小超过30000，丢弃帧")
        #     del self.data_buffer[0:24]
        #     return None
        if len(self.data_buffer) < packet_len:
            return None
        ret = copy.deepcopy(self.data_buffer[start_index: start_index + packet_len * 2])
        del self.data_buffer[start_index: start_index + packet_len * 2]

        return ret

    def convert_string(self, string):
        if string == "":
            return "0"
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
    pointData = ReceivePointData("COM4", "COM3")
    pointData.open_port()
    pointData.send_config()
    _thread.start_new_thread(pointData.receive_data(), ())