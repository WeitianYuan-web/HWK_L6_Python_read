#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
CAN通信上位机程序
用于与双手机械手进行CAN总线通信

CAN帧格式：
- 左手控制:  0x30-0x3F
- 左手反馈:  0x40-0x4F
- 右手控制: 0x50-0x5F
- 右手反馈: 0x60-0x6F
- 数据类型: n=0(位置), n=1(速度), n=2(扭矩)

依赖库安装:
pip install python-can
"""

import can
import struct
import time
import threading
from typing import List, Dict
from dataclasses import dataclass


# ==================== CAN ID 定义 ====================
CAN_ID_LEFT_CTRL_BASE = 0x30    # 左手控制ID基地址
CAN_ID_LEFT_FB_BASE = 0x40      # 左手反馈ID基地址
CAN_ID_RIGHT_CTRL_BASE = 0x50   # 右手控制ID基地址
CAN_ID_RIGHT_FB_BASE = 0x60     # 右手反馈ID基地址

DATA_TYPE_POSITION = 0          # 位置数据
DATA_TYPE_VELOCITY = 1          # 速度数据
DATA_TYPE_TORQUE = 2            # 扭矩/电流数据

JOINT_COUNT = 6                 # 每只手6个关节
JOINT_DATA_MAX = 1023           # 10bit数据最大值


# ==================== 数据类 ====================
@dataclass
class JointData:
    """关节数据类"""
    position: List[int] = None
    velocity: List[int] = None
    torque: List[int] = None
    
    def __post_init__(self):
        if self.position is None:
            self.position = [0] * JOINT_COUNT
        if self.velocity is None:
            self.velocity = [0] * JOINT_COUNT
        if self.torque is None:
            self.torque = [0] * JOINT_COUNT


# ==================== 数据打包/解包函数 ====================
def pack_joint_data(joint_data: List[int]) -> bytes:
    """
    将6个关节数据打包为8字节CAN数据
    
    Args:
        joint_data: 6个关节数据列表，每个数据10bit (0-1023)
        
    Returns:
        8字节的CAN数据
    """
    # 限制数据范围
    joint_data = [min(max(0, val), JOINT_DATA_MAX) for val in joint_data[:JOINT_COUNT]]
    
    # 打包数据：每个关节10bit，共60bit
    packed = 0
    for i in range(JOINT_COUNT):
        packed |= (joint_data[i] & 0x3FF) << (i * 10)
    
    # 转换为8字节数组（小端序）
    return struct.pack('<Q', packed)


def unpack_joint_data(can_data: bytes) -> List[int]:
    """
    从8字节CAN数据解包为6个关节数据
    
    Args:
        can_data: 8字节的CAN数据
        
    Returns:
        6个关节数据列表
    """
    # 转换为64位整数（小端序）
    packed = struct.unpack('<Q', can_data)[0]
    
    # 解包数据：每个关节10bit
    joint_data = []
    for i in range(JOINT_COUNT):
        joint_data.append((packed >> (i * 10)) & 0x3FF)
    
    return joint_data


# ==================== CAN通信类 ====================
class CANRobotHand:
    """双手机械手CAN通信类"""
    
    def __init__(self, channel='can0', bustype='socketcan', bitrate=500000):
        """
        初始化CAN通信
        
        Args:
            channel: CAN通道名称（如'can0', 'PCAN_USBBUS1'等）
            bustype: 总线类型（'socketcan', 'pcan', 'vector'等）
            bitrate: 波特率（默认500kbps）
        """
        self.channel = channel
        self.bustype = bustype
        self.bitrate = bitrate
        self.bus = None
        
        # 数据存储
        self.left_hand_feedback = JointData()
        self.right_hand_feedback = JointData()
        self.left_hand_control = JointData()
        self.right_hand_control = JointData()
        
        # 统计信息
        self.tx_count = 0
        self.rx_count = 0
        self.rx_count_by_id = {}
        
        # 接收线程
        self.rx_thread = None
        self.running = False
        
    def connect(self) -> bool:
        """
        连接到CAN总线
        
        Returns:
            连接是否成功
        """
        try:
            self.bus = can.interface.Bus(
                channel=self.channel,
                bustype=self.bustype,
                bitrate=self.bitrate
            )
            print(f"✓ CAN总线连接成功: {self.channel} @ {self.bitrate}bps")
            return True
        except Exception as e:
            print(f"✗ CAN总线连接失败: {e}")
            return False
    
    def disconnect(self):
        """断开CAN总线连接"""
        self.stop_receive()
        if self.bus:
            self.bus.shutdown()
            print("✓ CAN总线已断开")
    
    def send_joint_data(self, base_id: int, data_type: int, joint_data: List[int]) -> bool:
        """
        发送关节数据
        
        Args:
            base_id: 基础ID（0x30/0x40/0x50/0x60）
            data_type: 数据类型（0=位置, 1=速度, 2=扭矩）
            joint_data: 6个关节数据列表
            
        Returns:
            发送是否成功
        """
        if not self.bus:
            print("✗ CAN总线未连接")
            return False
        
        can_id = base_id + data_type
        can_data = pack_joint_data(joint_data)
        
        try:
            msg = can.Message(
                arbitration_id=can_id,
                data=can_data,
                is_extended_id=False
            )
            self.bus.send(msg)
            self.tx_count += 1
            return True
        except Exception as e:
            print(f"✗ 发送失败 (ID:0x{can_id:02X}): {e}")
            return False
    
    def send_left_control(self, position=None, velocity=None, torque=None):
        """发送左手控制命令"""
        if position is not None:
            self.left_hand_control.position = position
            self.send_joint_data(CAN_ID_LEFT_CTRL_BASE, DATA_TYPE_POSITION, position)
        if velocity is not None:
            self.left_hand_control.velocity = velocity
            self.send_joint_data(CAN_ID_LEFT_CTRL_BASE, DATA_TYPE_VELOCITY, velocity)
        if torque is not None:
            self.left_hand_control.torque = torque
            self.send_joint_data(CAN_ID_LEFT_CTRL_BASE, DATA_TYPE_TORQUE, torque)
    
    def send_right_control(self, position=None, velocity=None, torque=None):
        """发送右手控制命令"""
        if position is not None:
            self.right_hand_control.position = position
            self.send_joint_data(CAN_ID_RIGHT_CTRL_BASE, DATA_TYPE_POSITION, position)
        if velocity is not None:
            self.right_hand_control.velocity = velocity
            self.send_joint_data(CAN_ID_RIGHT_CTRL_BASE, DATA_TYPE_VELOCITY, velocity)
        if torque is not None:
            self.right_hand_control.torque = torque
            self.send_joint_data(CAN_ID_RIGHT_CTRL_BASE, DATA_TYPE_TORQUE, torque)
    
    def _process_received_message(self, msg: can.Message):
        """
        处理接收到的CAN消息
        
        Args:
            msg: CAN消息对象
        """
        can_id = msg.arbitration_id
        data = bytes(msg.data)
        
        if len(data) != 8:
            return
        
        # 统计
        self.rx_count += 1
        self.rx_count_by_id[can_id] = self.rx_count_by_id.get(can_id, 0) + 1
        
        # 解析数据
        id_base = can_id & 0xF0
        data_type = can_id & 0x0F
        joint_data = unpack_joint_data(data)
        
        # 根据ID分类存储
        if id_base == CAN_ID_LEFT_FB_BASE:
            if data_type == DATA_TYPE_POSITION:
                self.left_hand_feedback.position = joint_data
            elif data_type == DATA_TYPE_VELOCITY:
                self.left_hand_feedback.velocity = joint_data
            elif data_type == DATA_TYPE_TORQUE:
                self.left_hand_feedback.torque = joint_data
                
        elif id_base == CAN_ID_RIGHT_FB_BASE:
            if data_type == DATA_TYPE_POSITION:
                self.right_hand_feedback.position = joint_data
            elif data_type == DATA_TYPE_VELOCITY:
                self.right_hand_feedback.velocity = joint_data
            elif data_type == DATA_TYPE_TORQUE:
                self.right_hand_feedback.torque = joint_data
    
    def _receive_thread(self):
        """接收线程函数"""
        print("✓ 接收线程已启动")
        while self.running:
            try:
                msg = self.bus.recv(timeout=0.1)
                if msg:
                    self._process_received_message(msg)
            except Exception as e:
                if self.running:
                    print(f"✗ 接收错误: {e}")
        print("✓ 接收线程已停止")
    
    def start_receive(self):
        """启动接收线程"""
        if not self.bus:
            print("✗ CAN总线未连接")
            return False
        
        if self.running:
            print("⚠ 接收线程已在运行")
            return True
        
        self.running = True
        self.rx_thread = threading.Thread(target=self._receive_thread, daemon=True)
        self.rx_thread.start()
        return True
    
    def stop_receive(self):
        """停止接收线程"""
        if self.running:
            self.running = False
            if self.rx_thread:
                self.rx_thread.join(timeout=1.0)
    
    def print_status(self):
        """打印当前状态"""
        print("\n" + "="*70)
        print(f"  CAN通信状态 - TX:{self.tx_count}  RX:{self.rx_count}")
        print("="*70)
        
        print(f"\n【左手反馈】")
        print(f"  位置: {self.left_hand_feedback.position}")
        print(f"  速度: {self.left_hand_feedback.velocity}")
        print(f"  扭矩: {self.left_hand_feedback.torque}")
        
        print(f"\n【右手反馈】")
        print(f"  位置: {self.right_hand_feedback.position}")
        print(f"  速度: {self.right_hand_feedback.velocity}")
        print(f"  扭矩: {self.right_hand_feedback.torque}")
        
        print(f"\n【接收统计】")
        for can_id, count in sorted(self.rx_count_by_id.items()):
            print(f"  ID:0x{can_id:02X} -> {count} 条消息")
        print()


# ==================== 主程序 ====================
def main():
    """主程序"""
    print("\n" + "="*70)
    print("  双手机械手CAN通信上位机")
    print("="*70)
    
    # 创建CAN通信对象
    # Windows PCAN: bustype='pcan', channel='PCAN_USBBUS1'
    # Linux SocketCAN: bustype='socketcan', channel='can0'
    # Vector: bustype='vector', channel=0
    
    robot = CANRobotHand(
        channel='PCAN_USBBUS1',      # 根据实际情况修改
        bustype='pcan',  # 根据实际情况修改
        bitrate=1000000
    )
    
    # 连接CAN总线
    if not robot.connect():
        print("\n请检查：")
        print("1. CAN适配器是否连接")
        print("2. CAN通道名称是否正确")
        print("3. 驱动是否已安装")
        print("\nLinux SocketCAN 设置示例：")
        print("  sudo ip link set can0 type can bitrate 1000000")
        print("  sudo ip link set can0 up")
        return
    
    # 启动接收线程
    robot.start_receive()
    
    print("\n开始测试...")
    print("按 Ctrl+C 退出\n")
    
    try:
        cycle = 0
        left_torque = [1000, 1000, 500, 500, 500, 500]
        robot.send_left_control(torque=left_torque)
        time.sleep(1)
        while True:
            cycle += 1
            
            # 测试1: 发送左手位置控制命令（正弦波）
            import math
            angle = cycle * 0.1
            left_pos = [
                int(512 + 200 * math.sin(angle - 1)),
                int(512 + 200 * math.sin(angle + 2)),
                int(512 + 200 * math.sin(angle + 2)),
                int(512 + 200 * math.sin(angle + 3)),
                int(512 + 200 * math.sin(angle + 4)),
                int(512 + 200 * math.sin(angle + 5)),
            ]
            robot.send_left_control(position=left_pos)
            
            # 测试2: 发送右手速度控制命令
            #right_vel = [100 + cycle % 100, 150, 200, 250, 300, 350]
            #robot.send_right_control(velocity=right_vel)
            
            # 每100次循环打印一次状态
            if cycle % 100 == 0:
                robot.print_status()
            
           
            time.sleep(0.01)
            
    except KeyboardInterrupt:
        print("\n\n用户中断，正在退出...")
    
    finally:
        # 清理资源
        robot.disconnect()
        print("✓ 程序已退出\n")


if __name__ == "__main__":
    main()
