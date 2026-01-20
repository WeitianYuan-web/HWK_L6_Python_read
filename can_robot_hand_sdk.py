#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
双手机械手CAN通信SDK

本SDK提供了与双手机械手进行CAN总线通信的完整协议实现。

@file    can_robot_hand_sdk.py
@brief   双手机械手CAN通信协议SDK
@author  Your Name
@date    2025-01-07
@version 1.0.0

@section 协议说明
CAN帧ID分配：
- 左手控制:  0x30-0x3F (Base: 0x30)
- 左手反馈:  0x40-0x4F (Base: 0x40)
- 右手控制: 0x50-0x5F (Base: 0x50)
- 右手反馈: 0x60-0x6F (Base: 0x60)

数据类型偏移：
- n=0: 位置数据 (Position)
- n=1: 速度数据 (Velocity)
- n=2: 扭矩/电流数据 (Torque/Current)

数据格式：
- 每帧8字节，包含6个关节的数据
- 每个关节数据为10bit (0-1023)
- 采用小端序打包

@section 依赖库
pip install python-can

@section 使用示例
@code
from can_robot_hand_sdk import RobotHandSDK

# 创建SDK实例
sdk = RobotHandSDK(channel='PCAN_USBBUS1', bustype='pcan', bitrate=1000000)

# 连接CAN总线
if sdk.connect():
    # 启动接收
    sdk.start_receive()
    
    # 发送左手位置命令
    sdk.send_left_control(position=[512, 512, 512, 512, 512, 512])
    
    # 读取左手反馈
    left_fb = sdk.get_left_feedback()
    print(f"左手位置: {left_fb.position}")
    
    # 断开连接
    sdk.disconnect()
@endcode
"""

import can
import struct
import time
import threading
from typing import List, Dict, Optional, Callable
from dataclasses import dataclass, field
from enum import IntEnum


__version__ = "1.0.0"
__author__ = "Yuan Weitian"
__all__ = [
    'RobotHandSDK',
    'JointData',
    'HandType',
    'DataType',
    'CANProtocol',
    'pack_joint_data',
    'unpack_joint_data',
]


# ==================== 协议定义 ====================

class HandType(IntEnum):
    """
    @brief 手臂类型枚举
    """
    LEFT = 0    ##< 左手
    RIGHT = 1   ##< 右手


class DataType(IntEnum):
    """
    @brief 数据类型枚举
    """
    POSITION = 0    ##< 位置数据
    VELOCITY = 1    ##< 速度数据
    TORQUE = 2      ##< 扭矩/电流数据


class CANProtocol:
    """
    @brief CAN通信协议常量定义
    """
    # CAN ID基地址
    ID_LEFT_CTRL_BASE = 0x30      ##< 左手控制ID基地址
    ID_LEFT_FB_BASE = 0x40        ##< 左手反馈ID基地址
    ID_RIGHT_CTRL_BASE = 0x50     ##< 右手控制ID基地址
    ID_RIGHT_FB_BASE = 0x60       ##< 右手反馈ID基地址
    
    # 数据参数
    JOINT_COUNT = 6               ##< 每只手的关节数量
    JOINT_DATA_MAX = 1023         ##< 10bit数据最大值
    JOINT_DATA_MIN = 0            ##< 10bit数据最小值
    CAN_DATA_LENGTH = 8           ##< CAN数据帧长度（字节）
    
    # 默认通信参数
    DEFAULT_BITRATE = 1000000      ##< 默认波特率（1Mbps）
    DEFAULT_TIMEOUT = 0.1         ##< 默认超时时间（秒）


# ==================== 数据结构 ====================

@dataclass
class JointData:
    """
    @brief 关节数据类
    
    存储单只手的所有关节数据，包括位置、速度和扭矩。
    """
    position: List[int] = field(default_factory=lambda: [0] * CANProtocol.JOINT_COUNT)  ##< 位置数据 (0-1023)
    velocity: List[int] = field(default_factory=lambda: [0] * CANProtocol.JOINT_COUNT)  ##< 速度数据 (0-1023)
    torque: List[int] = field(default_factory=lambda: [0] * CANProtocol.JOINT_COUNT)    ##< 扭矩数据 (0-1023)
    
    def __post_init__(self):
        """初始化后检查"""
        self._ensure_list_size()
    
    def _ensure_list_size(self):
        """确保列表大小正确"""
        if len(self.position) != CANProtocol.JOINT_COUNT:
            self.position = (self.position + [0] * CANProtocol.JOINT_COUNT)[:CANProtocol.JOINT_COUNT]
        if len(self.velocity) != CANProtocol.JOINT_COUNT:
            self.velocity = (self.velocity + [0] * CANProtocol.JOINT_COUNT)[:CANProtocol.JOINT_COUNT]
        if len(self.torque) != CANProtocol.JOINT_COUNT:
            self.torque = (self.torque + [0] * CANProtocol.JOINT_COUNT)[:CANProtocol.JOINT_COUNT]
    
    def get_data(self, data_type: DataType) -> List[int]:
        """
        @brief 根据数据类型获取数据
        @param data_type 数据类型
        @return 对应的关节数据列表
        """
        if data_type == DataType.POSITION:
            return self.position
        elif data_type == DataType.VELOCITY:
            return self.velocity
        elif data_type == DataType.TORQUE:
            return self.torque
        else:
            return [0] * CANProtocol.JOINT_COUNT
    
    def set_data(self, data_type: DataType, data: List[int]):
        """
        @brief 根据数据类型设置数据
        @param data_type 数据类型
        @param data 关节数据列表
        """
        if data_type == DataType.POSITION:
            self.position = data[:CANProtocol.JOINT_COUNT]
        elif data_type == DataType.VELOCITY:
            self.velocity = data[:CANProtocol.JOINT_COUNT]
        elif data_type == DataType.TORQUE:
            self.torque = data[:CANProtocol.JOINT_COUNT]


# ==================== 数据打包/解包函数 ====================

def pack_joint_data(joint_data: List[int]) -> bytes:
    """
    @brief 将6个关节数据打包为8字节CAN数据
    
    数据格式：
    - Joint0: bit 0-9
    - Joint1: bit 10-19
    - Joint2: bit 20-29
    - Joint3: bit 30-39
    - Joint4: bit 40-49
    - Joint5: bit 50-59
    - Padding: bit 60-63 (填充0)
    
    @param joint_data 6个关节数据列表，每个数据10bit (0-1023)
    @return 8字节的CAN数据
    """
    # 限制数据范围
    joint_data = [
        min(max(CANProtocol.JOINT_DATA_MIN, val), CANProtocol.JOINT_DATA_MAX) 
        for val in joint_data[:CANProtocol.JOINT_COUNT]
    ]
    
    # 打包数据：每个关节10bit，共60bit
    packed = 0
    for i in range(CANProtocol.JOINT_COUNT):
        packed |= (joint_data[i] & 0x3FF) << (i * 10)
    
    # 转换为8字节数组（小端序）
    return struct.pack('<Q', packed)


def unpack_joint_data(can_data: bytes) -> List[int]:
    """
    @brief 从8字节CAN数据解包为6个关节数据
    
    @param can_data 8字节的CAN数据
    @return 6个关节数据列表
    @throws struct.error 如果数据长度不是8字节
    """
    if len(can_data) != CANProtocol.CAN_DATA_LENGTH:
        raise ValueError(f"CAN数据长度必须为{CANProtocol.CAN_DATA_LENGTH}字节，实际为{len(can_data)}字节")
    
    # 转换为64位整数（小端序）
    packed = struct.unpack('<Q', can_data)[0]
    
    # 解包数据：每个关节10bit
    joint_data = []
    for i in range(CANProtocol.JOINT_COUNT):
        joint_data.append((packed >> (i * 10)) & 0x3FF)
    
    return joint_data


# ==================== 主SDK类 ====================

class RobotHandSDK:
    """
    @brief 双手机械手CAN通信SDK主类
    
    提供完整的CAN通信接口，支持双手独立控制和反馈读取。
    """
    
    def __init__(self, 
                 channel: str = 'can0', 
                 bustype: str = 'socketcan', 
                 bitrate: int = CANProtocol.DEFAULT_BITRATE,
                 receive_timeout: float = CANProtocol.DEFAULT_TIMEOUT):
        """
        @brief 初始化SDK
        
        @param channel CAN通道名称
                       - Linux SocketCAN: 'can0', 'can1'
                       - Windows PCAN: 'PCAN_USBBUS1', 'PCAN_USBBUS2'
                       - Vector: 0, 1
        @param bustype 总线类型
                       - 'socketcan': Linux SocketCAN
                       - 'pcan': PEAK PCAN
                       - 'vector': Vector CANalyzer/CANoe
                       - 'kvaser': Kvaser
        @param bitrate 波特率（bps），默认1Mbps
        @param receive_timeout 接收超时时间（秒）
        """
        self.channel = channel
        self.bustype = bustype
        self.bitrate = bitrate
        self.receive_timeout = receive_timeout
        self.bus: Optional[can.Bus] = None
        
        # 数据存储
        self._left_hand_feedback = JointData()
        self._right_hand_feedback = JointData()
        self._left_hand_control = JointData()
        self._right_hand_control = JointData()
        
        # 统计信息
        self._tx_count = 0
        self._rx_count = 0
        self._rx_count_by_id: Dict[int, int] = {}
        self._tx_error_count = 0
        
        # 接收线程
        self._rx_thread: Optional[threading.Thread] = None
        self._running = False
        self._lock = threading.Lock()
        
        # 回调函数
        self._message_callbacks: List[Callable[[can.Message], None]] = []
        self._error_callback: Optional[Callable[[Exception], None]] = None
    
    # ==================== 连接管理 ====================
    
    def connect(self) -> bool:
        """
        @brief 连接到CAN总线
        
        @return True=连接成功, False=连接失败
        """
        try:
            self.bus = can.interface.Bus(
                channel=self.channel,
                bustype=self.bustype,
                bitrate=self.bitrate
            )
            print(f"✓ CAN总线连接成功: {self.channel} ({self.bustype}) @ {self.bitrate}bps")
            return True
        except Exception as e:
            print(f"✗ CAN总线连接失败: {e}")
            if self._error_callback:
                self._error_callback(e)
            return False
    
    def disconnect(self):
        """
        @brief 断开CAN总线连接
        """
        self.stop_receive()
        if self.bus:
            self.bus.shutdown()
            self.bus = None
            print("✓ CAN总线已断开")
    
    def is_connected(self) -> bool:
        """
        @brief 检查是否已连接
        
        @return True=已连接, False=未连接
        """
        return self.bus is not None
    
    # ==================== 数据发送 ====================
    
    def send_joint_data(self, base_id: int, data_type: DataType, joint_data: List[int]) -> bool:
        """
        @brief 发送关节数据（底层函数）
        
        @param base_id 基础ID（0x30/0x40/0x50/0x60）
        @param data_type 数据类型
        @param joint_data 6个关节数据列表
        @return True=发送成功, False=发送失败
        """
        if not self.is_connected():
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
            with self._lock:
                self._tx_count += 1
            return True
        except Exception as e:
            print(f"✗ 发送失败 (ID:0x{can_id:02X}): {e}")
            with self._lock:
                self._tx_error_count += 1
            if self._error_callback:
                self._error_callback(e)
            return False
    
    def send_left_control(self, 
                         position: Optional[List[int]] = None, 
                         velocity: Optional[List[int]] = None, 
                         torque: Optional[List[int]] = None) -> bool:
        """
        @brief 发送左手控制命令
        
        @param position 位置数据 (可选)
        @param velocity 速度数据 (可选)
        @param torque 扭矩数据 (可选)
        @return True=全部发送成功, False=存在发送失败
        """
        success = True
        if position is not None:
            self._left_hand_control.position = position[:CANProtocol.JOINT_COUNT]
            success &= self.send_joint_data(CANProtocol.ID_LEFT_CTRL_BASE, 
                                           DataType.POSITION, position)
        if velocity is not None:
            self._left_hand_control.velocity = velocity[:CANProtocol.JOINT_COUNT]
            success &= self.send_joint_data(CANProtocol.ID_LEFT_CTRL_BASE, 
                                           DataType.VELOCITY, velocity)
        if torque is not None:
            self._left_hand_control.torque = torque[:CANProtocol.JOINT_COUNT]
            success &= self.send_joint_data(CANProtocol.ID_LEFT_CTRL_BASE, 
                                           DataType.TORQUE, torque)
        return success
    
    def send_right_control(self, 
                          position: Optional[List[int]] = None, 
                          velocity: Optional[List[int]] = None, 
                          torque: Optional[List[int]] = None) -> bool:
        """
        @brief 发送右手控制命令
        
        @param position 位置数据 (可选)
        @param velocity 速度数据 (可选)
        @param torque 扭矩数据 (可选)
        @return True=全部发送成功, False=存在发送失败
        """
        success = True
        if position is not None:
            self._right_hand_control.position = position[:CANProtocol.JOINT_COUNT]
            success &= self.send_joint_data(CANProtocol.ID_RIGHT_CTRL_BASE, 
                                           DataType.POSITION, position)
        if velocity is not None:
            self._right_hand_control.velocity = velocity[:CANProtocol.JOINT_COUNT]
            success &= self.send_joint_data(CANProtocol.ID_RIGHT_CTRL_BASE, 
                                           DataType.VELOCITY, velocity)
        if torque is not None:
            self._right_hand_control.torque = torque[:CANProtocol.JOINT_COUNT]
            success &= self.send_joint_data(CANProtocol.ID_RIGHT_CTRL_BASE, 
                                           DataType.TORQUE, torque)
        return success
    
    # ==================== 数据接收 ====================
    
    def _process_received_message(self, msg: can.Message):
        """
        @brief 处理接收到的CAN消息
        
        @param msg CAN消息对象
        """
        can_id = msg.arbitration_id
        data = bytes(msg.data)
        
        if len(data) != CANProtocol.CAN_DATA_LENGTH:
            return
        
        # 统计
        with self._lock:
            self._rx_count += 1
            self._rx_count_by_id[can_id] = self._rx_count_by_id.get(can_id, 0) + 1
        
        # 解析数据
        id_base = can_id & 0xF0
        data_type = DataType(can_id & 0x0F)
        
        try:
            joint_data = unpack_joint_data(data)
        except Exception as e:
            print(f"✗ 数据解包失败 (ID:0x{can_id:02X}): {e}")
            return
        
        # 根据ID分类存储
        with self._lock:
            if id_base == CANProtocol.ID_LEFT_FB_BASE:
                self._left_hand_feedback.set_data(data_type, joint_data)
            elif id_base == CANProtocol.ID_RIGHT_FB_BASE:
                self._right_hand_feedback.set_data(data_type, joint_data)
        
        # 调用回调函数
        for callback in self._message_callbacks:
            try:
                callback(msg)
            except Exception as e:
                print(f"✗ 回调函数执行错误: {e}")
    
    def _receive_thread(self):
        """
        @brief 接收线程函数
        """
        print("✓ 接收线程已启动")
        while self._running:
            try:
                msg = self.bus.recv(timeout=self.receive_timeout)
                if msg:
                    self._process_received_message(msg)
            except Exception as e:
                if self._running:
                    print(f"✗ 接收错误: {e}")
                    if self._error_callback:
                        self._error_callback(e)
        print("✓ 接收线程已停止")
    
    def start_receive(self) -> bool:
        """
        @brief 启动接收线程
        
        @return True=启动成功, False=启动失败
        """
        if not self.is_connected():
            print("✗ CAN总线未连接")
            return False
        
        if self._running:
            print("⚠ 接收线程已在运行")
            return True
        
        self._running = True
        self._rx_thread = threading.Thread(target=self._receive_thread, daemon=True)
        self._rx_thread.start()
        return True
    
    def stop_receive(self):
        """
        @brief 停止接收线程
        """
        if self._running:
            self._running = False
            if self._rx_thread:
                self._rx_thread.join(timeout=1.0)
    
    # ==================== 数据读取 ====================
    
    def get_left_feedback(self) -> JointData:
        """
        @brief 获取左手反馈数据
        
        @return 左手关节数据的副本
        """
        with self._lock:
            # 返回深拷贝以避免线程安全问题
            return JointData(
                position=self._left_hand_feedback.position.copy(),
                velocity=self._left_hand_feedback.velocity.copy(),
                torque=self._left_hand_feedback.torque.copy()
            )
    
    def get_right_feedback(self) -> JointData:
        """
        @brief 获取右手反馈数据
        
        @return 右手关节数据的副本
        """
        with self._lock:
            return JointData(
                position=self._right_hand_feedback.position.copy(),
                velocity=self._right_hand_feedback.velocity.copy(),
                torque=self._right_hand_feedback.torque.copy()
            )
    
    def get_left_control(self) -> JointData:
        """
        @brief 获取左手控制数据
        
        @return 左手控制数据的副本
        """
        with self._lock:
            return JointData(
                position=self._left_hand_control.position.copy(),
                velocity=self._left_hand_control.velocity.copy(),
                torque=self._left_hand_control.torque.copy()
            )
    
    def get_right_control(self) -> JointData:
        """
        @brief 获取右手控制数据
        
        @return 右手控制数据的副本
        """
        with self._lock:
            return JointData(
                position=self._right_hand_control.position.copy(),
                velocity=self._right_hand_control.velocity.copy(),
                torque=self._right_hand_control.torque.copy()
            )
    
    # ==================== 统计信息 ====================
    
    def get_statistics(self) -> Dict:
        """
        @brief 获取统计信息
        
        @return 包含统计信息的字典
        """
        with self._lock:
            return {
                'tx_count': self._tx_count,
                'rx_count': self._rx_count,
                'tx_error_count': self._tx_error_count,
                'rx_count_by_id': self._rx_count_by_id.copy()
            }
    
    def reset_statistics(self):
        """
        @brief 重置统计信息
        """
        with self._lock:
            self._tx_count = 0
            self._rx_count = 0
            self._tx_error_count = 0
            self._rx_count_by_id.clear()
    
    def print_status(self):
        """
        @brief 打印当前状态
        """
        stats = self.get_statistics()
        left_fb = self.get_left_feedback()
        right_fb = self.get_right_feedback()
        
        print("\n" + "="*70)
        print(f"  CAN通信状态 - TX:{stats['tx_count']}  RX:{stats['rx_count']}  "
              f"TX错误:{stats['tx_error_count']}")
        print("="*70)
        
        print(f"\n【左手反馈】")
        print(f"  位置: {left_fb.position}")
        print(f"  速度: {left_fb.velocity}")
        print(f"  扭矩: {left_fb.torque}")
        
        print(f"\n【右手反馈】")
        print(f"  位置: {right_fb.position}")
        print(f"  速度: {right_fb.velocity}")
        print(f"  扭矩: {right_fb.torque}")
        
        if stats['rx_count_by_id']:
            print(f"\n【接收统计】")
            for can_id, count in sorted(stats['rx_count_by_id'].items()):
                print(f"  ID:0x{can_id:02X} -> {count} 条消息")
        print()
    
    # ==================== 回调函数管理 ====================
    
    def add_message_callback(self, callback: Callable[[can.Message], None]):
        """
        @brief 添加消息接收回调函数
        
        @param callback 回调函数，参数为can.Message对象
        """
        if callback not in self._message_callbacks:
            self._message_callbacks.append(callback)
    
    def remove_message_callback(self, callback: Callable[[can.Message], None]):
        """
        @brief 移除消息接收回调函数
        
        @param callback 要移除的回调函数
        """
        if callback in self._message_callbacks:
            self._message_callbacks.remove(callback)
    
    def set_error_callback(self, callback: Optional[Callable[[Exception], None]]):
        """
        @brief 设置错误回调函数
        
        @param callback 错误回调函数，参数为Exception对象
        """
        self._error_callback = callback
    
    # ==================== 上下文管理器支持 ====================
    
    def __enter__(self):
        """上下文管理器入口"""
        self.connect()
        self.start_receive()
        return self
    
    def __exit__(self, exc_type, exc_val, exc_tb):
        """上下文管理器退出"""
        self.disconnect()
        return False


# ==================== 便捷函数 ====================

def create_sdk(channel: str = 'can0', 
               bustype: str = 'socketcan', 
               bitrate: int = CANProtocol.DEFAULT_BITRATE,
               auto_connect: bool = True) -> RobotHandSDK:
    """
    @brief 创建SDK实例的便捷函数
    
    @param channel CAN通道名称
    @param bustype 总线类型
    @param bitrate 波特率
    @param auto_connect 是否自动连接
    @return SDK实例
    """
    sdk = RobotHandSDK(channel=channel, bustype=bustype, bitrate=bitrate)
    if auto_connect:
        if sdk.connect():
            sdk.start_receive()
    return sdk


if __name__ == "__main__":
    """SDK模块测试"""
    print(f"双手机械手CAN通信SDK v{__version__}")
    print(f"协议版本: {CANProtocol.ID_LEFT_CTRL_BASE:02X}h-{CANProtocol.ID_RIGHT_FB_BASE+0x0F:02X}h")
    print("\n使用 'from can_robot_hand_sdk import RobotHandSDK' 导入SDK")
