#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from rosbag2_interfaces.srv import Pause, TogglePaused
from ais_gng_msgs.msg import TopologicalMap

import pathlib
import os
import sys
import termios
import fcntl

LabelList = [
    "human",
    "car",
]

ament_prefix_path = os.environ.get('AMENT_PREFIX_PATH')
install_path_str = ament_prefix_path.split(':')[0]
pkg_dir = pathlib.Path(install_path_str).parent.parent/'src'/'AiS-GNG'/'gng_classification'

# 待ちなしkey入力関数(引用：https://qiita.com/pukin/items/3b791b8b759dd704f765)
def getkey():
    fno = sys.stdin.fileno()
    attr_old = termios.tcgetattr(fno)
    attr = termios.tcgetattr(fno)
    attr[3] = attr[3] & ~termios.ECHO & ~termios.ICANON
    termios.tcsetattr(fno, termios.TCSADRAIN, attr)
    fcntl_old = fcntl.fcntl(fno, fcntl.F_GETFL)
    fcntl.fcntl(fno, fcntl.F_SETFL, fcntl_old | os.O_NONBLOCK)
    chr = 0
    try:
        c = sys.stdin.read(1)
        if len(c):
            while len(c):
                chr = (chr << 8) + ord(c)
                c = sys.stdin.read(1)
    finally:
        fcntl.fcntl(fno, fcntl.F_SETFL, fcntl_old)
        termios.tcsetattr(fno, termios.TCSANOW, attr_old)
    return chr

# サンプリングツール用ノードの定義
class SamplingNode(Node):

    def __init__(self):
        super().__init__('sampling')
        self.topo_sub_ = self.create_subscription(TopologicalMap,'/topological_map',self.topo_cb, 10)
        self.timer = self.create_timer(0.01, self.timer_cb)
        self.pause_client = self.create_client(Pause, '/rosbag2_player/pause')
        while not self.pause_client.wait_for_service(timeout_sec=1.0):
            pass
        self.start_client = self.create_client(TogglePaused, '/rosbag2_player/toggle_paused')
        while not self.start_client.wait_for_service(timeout_sec=1.0):
            pass
        self.pause_req = Pause.Request()
        self.start_req = TogglePaused.Request()
        out_dir = pkg_dir/'datasets'
        if (not out_dir.exists()):
            out_dir.mkdir()
        dir_list = [int(dir.name) for dir in out_dir.iterdir()]
        if ((out_dir/str(len(dir_list))).exists()):
            file_num = len([_ for _ in (out_dir/str(len(dir_list))).glob('*')])
            if (file_num==0):
                self.out_dir = out_dir/str(len(dir_list))
            else:
                self.out_dir = out_dir/str(len(dir_list)+1)
        else:
            self.out_dir = out_dir/'1'
        self.out_dir.mkdir(exist_ok=True)
        self.i = 0
        self.working_mode = False
        self.data_list = []

    # DynamicObjectのみを抽出してファイルに書き込むためのリストself.data_listを作成
    def topo_cb(self, msg):
        self.data_list = []
        for cluster in msg.clusters:
            flag = cluster.pos.z+cluster.scale.z/2 > 1.0 and cluster.scale.z < 0.7
            if not flag and (cluster.label==TopologicalMap.UNKNOWN_OBJECT or cluster.label==TopologicalMap.HUMAN or cluster.label==TopologicalMap.CAR):
                data = [cluster.id]
                for node_id in cluster.nodes:
                    node = msg.nodes[node_id]
                    data.append(node.pos.x)
                    data.append(node.pos.y)
                    data.append(node.pos.z)
                    data.append(node.normal.x)
                    data.append(node.normal.y)
                    data.append(node.normal.z)
                self.data_list.append(data)
    
    # Pause合図の送信
    def send_puse(self):
        self.feature = self.pause_client.call_async(self.pause_req)
    # start合図の送信
    def send_start(self):
        self.feature = self.start_client.call_async(self.start_req)

    # メインプロセス, Enterでpause/startを制御
    def timer_cb(self):
        s = getkey()
        if s == 10 and not self.working_mode:
            # 初期状態がstopだった場合の例外処理
            if len(self.data_list)==0:
                self.working_mode = True
                print("Push Enter again! To start!")
                return
            # ここからファイルに保存＆画面を止める処理
            self.send_puse()
            self.working_mode = True
            id_list = []
            for label in LabelList:
                label_id = input('Input ' + label + ' ID:\nExmaple:11,20,40\n')
                id_list.append(label_id.split(','))
            file_path = self.out_dir/f'{self.i}.txt'
            line = ''
            with open(file_path, mode='w') as f:
                for data in self.data_list:
                    unknown_flag = True
                    for id, label in zip(id_list,LabelList):
                        if str(data[0]) in id:
                            line += label+","
                            unknown_flag = False
                    if unknown_flag:
                        line += "unknown,"
                    line += ','.join(map(str, data[1:]))
                    line += '\n'
                f.write(line)
            print(f'write {self.i}.txt')
            self.i += 1
            print("Push Enter to restart!")
        # 再開
        elif s == 10 and self.working_mode:
            self.send_start()
            self.working_mode = False


def main(args=None):
    rclpy.init(args=args)
    node = SamplingNode()
    print("Press Enter to Control Start/Stop!")
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()