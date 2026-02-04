#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
@file tactile_control_visualizer.py
@brief 五指触觉传感器可视化与机械手控制集成程序
@details 共用一个CAN总线，实现：
         1. 触觉传感器数据接收与可视化 (CAN ID: 0x300-0x348 左手 / 0x400-0x448 右手)
         2. 机械手控制命令发送 (CAN ID: 0x30-0x3F 左手控制 / 0x50-0x5F 右手控制)
         3. 机械手反馈数据接收与显示 (CAN ID: 0x40-0x4F 左手反馈 / 0x60-0x6F 右手反馈)
         4. USB控制器数据读取与映射

@version 1.0
@date 2026-01-20

CAN协议说明:
- 触觉传感器: 0xHmn (H=3左手/4右手, m=手指ID, n=帧序号)
- 机械手控制: 0x3n左手 / 0x5n右手 (n=数据类型)
- 机械手反馈: 0x4n左手 / 0x6n右手 (n=数据类型)
"""

import can
import serial
import struct
import time
import threading
import numpy as np
from collections import deque
from dataclasses import dataclass, field
from typing import Optional, List, Tuple
import sys
import os

# 添加父目录到路径以导入SDK
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from can_robot_hand_sdk import CANProtocol, JointData, DataType, pack_joint_data, unpack_joint_data


# ============================================================================
# 配置类
# ============================================================================

class CANConfig:
    """
    @brief CAN配置常量
    """
    CHANNEL = 'PCAN_USBBUS1'
    INTERFACE = 'pcan'
    BITRATE = 1000000
    
    # 触觉传感器帧配置
    FRAMES_PER_FINGER = 9
    BYTES_PER_FRAME = 8
    TOTAL_BYTES = 72
    
    # 触觉传感器矩阵配置
    MATRIX_COLS = 12
    MATRIX_ROWS = 6
    
    # 手指配置
    NUM_FINGERS = 5
    FINGER_NAMES = ['拇指', '食指', '中指', '无名指', '小指']
    FINGER_NAMES_EN = ['Thumb', 'Index', 'Middle', 'Ring', 'Pinky']
    
    # 关节配置
    NUM_JOINTS = 6
    JOINT_NAMES = ['J0', 'J1', 'J2', 'J3', 'J4', 'J5']
    
    # CAN ID基址
    TACTILE_LEFT_BASE = 0x300
    TACTILE_RIGHT_BASE = 0x400


class SerialConfig:
    """
    @brief 串口配置常量
    """
    PORT = 'COM69'
    BAUDRATE = 115200
    TIMEOUT = 0.1
    PACKET_SIZE = 28
    FRAME_TAIL = b'\x00\x00\x80\x7f'


class ControlConfig:
    """
    @brief 控制参数配置
    """
    SEND_FREQUENCY = 200              # 提高发送频率到200Hz
    SEND_PERIOD = 1.0 / SEND_FREQUENCY
    
    ENABLE_FILTER = True
    FILTER_ALPHA = 0.6                # 提高响应速度 (0.3->0.6)
    
    # 关节角度范围 [最小角度, 最大角度]
    JOINT_ANGLE_RANGES = [
        (280.0, 382.0),
        (222.0, 276.0),
        (-24.0, 60.0),
        (-57.0, 28.0),
        (200.0, 271.0),
        (-10.0, 40.0),
    ]
    
    # 关节反向配置
    JOINT_REVERSED = [True, True, False, False, False, False]
    
    JOINT_POS_MIN = 0
    JOINT_POS_MAX = 1023


class VisualConfig:
    """
    @brief 可视化配置常量
    """
    WINDOW_WIDTH = 1600
    WINDOW_HEIGHT = 950
    FPS = 60
    
    # 热力图配置
    CELL_SIZE = 28
    CELL_MARGIN = 2
    FINGER_SPACING = 25
    
    # 颜色配置
    BG_COLOR = (30, 30, 40)
    TEXT_COLOR = (220, 220, 220)
    BORDER_COLOR = (80, 80, 100)
    
    # 进度条颜色
    BAR_BG_COLOR = (50, 50, 60)
    BAR_POS_COLOR = (50, 180, 100)
    BAR_VEL_COLOR = (100, 150, 255)
    BAR_TOR_COLOR = (255, 150, 50)


# ============================================================================
# 数据结构
# ============================================================================

@dataclass
class FingerData:
    """
    @brief 单个手指的触觉传感器数据
    """
    finger_id: int = 0
    raw_data: bytearray = field(default_factory=lambda: bytearray(72))
    matrix: np.ndarray = field(default_factory=lambda: np.zeros((CANConfig.MATRIX_ROWS, CANConfig.MATRIX_COLS), dtype=np.uint8))
    frame_mask: int = 0
    timestamp: float = 0.0
    valid: bool = False
    update_count: int = 0


@dataclass 
class RobotHandFeedback:
    """
    @brief 机械手反馈数据
    """
    position: List[int] = field(default_factory=lambda: [0] * 6)
    velocity: List[int] = field(default_factory=lambda: [0] * 6)
    torque: List[int] = field(default_factory=lambda: [0] * 6)
    timestamp: float = 0.0
    valid: bool = False


# ============================================================================
# 统一CAN管理器
# ============================================================================

class UnifiedCANManager:
    """
    @brief 统一CAN管理器
    @details 同时处理触觉传感器和机械手的CAN通信
    """
    
    def __init__(self, channel: str = None, interface: str = None, 
                 bitrate: int = None, hand: int = 0):
        """
        @brief 初始化CAN管理器
        @param hand 手选择 (0=左手, 1=右手)
        """
        self.channel = channel or CANConfig.CHANNEL
        self.interface = interface or CANConfig.INTERFACE
        self.bitrate = bitrate or CANConfig.BITRATE
        self.hand = hand
        
        self.bus: Optional[can.Bus] = None
        self._lock = threading.Lock()
        self._running = False
        self._thread: Optional[threading.Thread] = None
        
        # 触觉传感器数据
        self.tactile_base_id = CANConfig.TACTILE_LEFT_BASE if hand == 0 else CANConfig.TACTILE_RIGHT_BASE
        self._finger_data = [FingerData(finger_id=i) for i in range(CANConfig.NUM_FINGERS)]
        
        # 机械手反馈数据
        self.robot_fb_base = CANProtocol.ID_LEFT_FB_BASE if hand == 0 else CANProtocol.ID_RIGHT_FB_BASE
        self.robot_ctrl_base = CANProtocol.ID_LEFT_CTRL_BASE if hand == 0 else CANProtocol.ID_RIGHT_CTRL_BASE
        self._robot_feedback = RobotHandFeedback()
        
        # 统计信息
        self._stats = {
            'tactile_frames': 0,
            'robot_frames': 0,
            'tx_frames': 0,
            'finger_updates': [0] * 5,
            'start_time': 0.0
        }
    
    def _parse_tactile_can_id(self, can_id: int) -> Tuple[int, int, int]:
        """
        @brief 解析触觉传感器CAN ID
        """
        base = (can_id >> 8) & 0x0F
        
        if base == 0x03:
            hand = 0
        elif base == 0x04:
            hand = 1
        else:
            return (-1, -1, -1)
        
        finger_id = (can_id >> 4) & 0x0F
        frame_idx = can_id & 0x0F
        
        if finger_id > 4 or frame_idx > 8:
            return (-1, -1, -1)
        
        return (hand, finger_id, frame_idx)
    
    def _convert_to_matrix(self, raw_data: bytearray) -> np.ndarray:
        """
        @brief 将72字节原始数据转换为矩阵
        """
        matrix = np.zeros((CANConfig.MATRIX_ROWS, CANConfig.MATRIX_COLS), dtype=np.uint8)
        idx = 0
        for col in range(CANConfig.MATRIX_COLS):
            for row in range(CANConfig.MATRIX_ROWS):
                matrix[row, col] = raw_data[idx]
                idx += 1
        return matrix
    
    def _process_tactile_frame(self, msg: can.Message) -> bool:
        """
        @brief 处理触觉传感器CAN帧
        """
        hand, finger_id, frame_idx = self._parse_tactile_can_id(msg.arbitration_id)
        
        if hand < 0 or hand != self.hand:
            return False
        
        self._stats['tactile_frames'] += 1
        
        with self._lock:
            finger = self._finger_data[finger_id]
            
            offset = frame_idx * CANConfig.BYTES_PER_FRAME
            for i, byte in enumerate(msg.data[:8]):
                if offset + i < CANConfig.TOTAL_BYTES:
                    finger.raw_data[offset + i] = byte
            
            finger.frame_mask |= (1 << frame_idx)
            
            if finger.frame_mask == 0x1FF:
                finger.matrix = self._convert_to_matrix(finger.raw_data)
                finger.timestamp = time.time()
                finger.valid = True
                finger.update_count += 1
                finger.frame_mask = 0
                self._stats['finger_updates'][finger_id] += 1
                return True
        
        return False
    
    def _process_robot_feedback(self, msg: can.Message):
        """
        @brief 处理机械手反馈CAN帧
        """
        can_id = msg.arbitration_id
        data_type = can_id - self.robot_fb_base
        
        if data_type < 0 or data_type > 2:
            return
        
        self._stats['robot_frames'] += 1
        
        try:
            values = unpack_joint_data(msg.data)
            
            with self._lock:
                if data_type == DataType.POSITION:
                    self._robot_feedback.position = values
                elif data_type == DataType.VELOCITY:
                    self._robot_feedback.velocity = values
                elif data_type == DataType.TORQUE:
                    self._robot_feedback.torque = values
                
                self._robot_feedback.timestamp = time.time()
                self._robot_feedback.valid = True
        except Exception as e:
            pass
    
    def _process_frame(self, msg: can.Message):
        """
        @brief 处理接收到的CAN帧
        """
        can_id = msg.arbitration_id
        
        # 检查是否是触觉传感器数据 (0x300-0x4FF范围)
        if 0x300 <= can_id <= 0x4FF:
            self._process_tactile_frame(msg)
        # 检查是否是机械手反馈数据
        elif self.robot_fb_base <= can_id < self.robot_fb_base + 0x10:
            self._process_robot_feedback(msg)
    
    def _receive_loop(self):
        """
        @brief CAN接收循环
        """
        while self._running:
            try:
                msg = self.bus.recv(timeout=0.001)
                if msg:
                    self._process_frame(msg)
            except Exception as e:
                if self._running:
                    print(f"CAN接收错误: {e}")
                break
    
    def connect(self) -> bool:
        """
        @brief 连接CAN总线
        """
        if self.bus is not None:
            return True
        
        try:
            self.bus = can.interface.Bus(
                channel=self.channel,
                interface=self.interface,
                bitrate=self.bitrate
            )
            hand_name = "左手" if self.hand == 0 else "右手"
            print(f"CAN已连接: {self.channel} @ {self.bitrate}bps ({hand_name})")
            return True
        except Exception as e:
            print(f"CAN连接失败: {e}")
            return False
    
    def start(self):
        """
        @brief 开始接收数据
        """
        if not self.bus:
            if not self.connect():
                return
        
        if self._running:
            return
        
        self._running = True
        self._stats['start_time'] = time.time()
        self._thread = threading.Thread(target=self._receive_loop, daemon=True)
        self._thread.start()
    
    def stop(self):
        """
        @brief 停止接收
        """
        self._running = False
        if self._thread:
            self._thread.join(timeout=1.0)
            self._thread = None
    
    def disconnect(self):
        """
        @brief 断开连接
        """
        self.stop()
        if self.bus:
            self.bus.shutdown()
            self.bus = None
    
    def send_control(self, position: List[int] = None, velocity: List[int] = None, 
                     torque: List[int] = None):
        """
        @brief 发送机械手控制命令
        """
        if not self.bus:
            return
        
        try:
            if position:
                can_id = self.robot_ctrl_base + DataType.POSITION
                data = pack_joint_data(position)
                msg = can.Message(arbitration_id=can_id, data=data, is_extended_id=False)
                self.bus.send(msg)
                self._stats['tx_frames'] += 1
            
            if velocity:
                can_id = self.robot_ctrl_base + DataType.VELOCITY
                data = pack_joint_data(velocity)
                msg = can.Message(arbitration_id=can_id, data=data, is_extended_id=False)
                self.bus.send(msg)
                self._stats['tx_frames'] += 1
            
            if torque:
                can_id = self.robot_ctrl_base + DataType.TORQUE
                data = pack_joint_data(torque)
                msg = can.Message(arbitration_id=can_id, data=data, is_extended_id=False)
                self.bus.send(msg)
                self._stats['tx_frames'] += 1
        except Exception as e:
            print(f"CAN发送错误: {e}")
    
    def get_all_matrices(self) -> List[np.ndarray]:
        """
        @brief 获取所有触觉传感器矩阵
        """
        with self._lock:
            return [f.matrix.copy() for f in self._finger_data]
    
    def get_all_fingers_data(self) -> List[FingerData]:
        """
        @brief 获取所有手指数据
        """
        with self._lock:
            return [FingerData(
                finger_id=f.finger_id,
                matrix=f.matrix.copy(),
                valid=f.valid,
                timestamp=f.timestamp,
                update_count=f.update_count
            ) for f in self._finger_data]
    
    def get_robot_feedback(self) -> RobotHandFeedback:
        """
        @brief 获取机械手反馈数据
        """
        with self._lock:
            return RobotHandFeedback(
                position=self._robot_feedback.position.copy(),
                velocity=self._robot_feedback.velocity.copy(),
                torque=self._robot_feedback.torque.copy(),
                timestamp=self._robot_feedback.timestamp,
                valid=self._robot_feedback.valid
            )
    
    def get_stats(self) -> dict:
        """
        @brief 获取统计信息
        """
        elapsed = time.time() - self._stats['start_time'] if self._stats['start_time'] > 0 else 0
        total_tactile = sum(self._stats['finger_updates'])
        
        return {
            'tactile_frames': self._stats['tactile_frames'],
            'robot_frames': self._stats['robot_frames'],
            'tx_frames': self._stats['tx_frames'],
            'finger_updates': self._stats['finger_updates'].copy(),
            'total_tactile_updates': total_tactile,
            'elapsed': elapsed,
            'tactile_rate': total_tactile / elapsed if elapsed > 0 else 0
        }


# ============================================================================
# 控制器数据读取器
# ============================================================================

class ControllerReader:
    """
    @brief USB控制器数据读取器 (高性能优化版)
    @details 使用独立线程持续读取，保持最新数据可用
    """
    
    def __init__(self, port: str = None, baudrate: int = None, timeout: float = None):
        self.port = port or SerialConfig.PORT
        self.baudrate = baudrate or SerialConfig.BAUDRATE
        self.timeout = timeout or SerialConfig.TIMEOUT
        self.serial: Optional[serial.Serial] = None
        self.buffer = bytearray()
        
        self.filtered_angles = [0.0] * 6
        self.is_first_read = True
        self.last_angles = [0.0] * 6
        self._latest_angles: Optional[List[float]] = None
        
        self._lock = threading.Lock()
        self._running = False
        self._thread: Optional[threading.Thread] = None
        
        self.packet_count = 0
        self.error_count = 0
    
    def connect(self) -> bool:
        """
        @brief 连接串口
        """
        try:
            self.serial = serial.Serial(
                port=self.port,
                baudrate=self.baudrate,
                timeout=0.001  # 极短超时用于非阻塞读取
            )
            print(f"串口已连接: {self.port} @ {self.baudrate}bps")
            return True
        except Exception as e:
            print(f"串口连接失败: {e}")
            return False
    
    def disconnect(self):
        """
        @brief 断开串口
        """
        self.stop()
        if self.serial and self.serial.is_open:
            self.serial.close()
    
    def _read_loop(self):
        """
        @brief 独立线程读取循环 - 持续读取并保持最新数据
        """
        while self._running:
            try:
                if not self.serial or not self.serial.is_open:
                    time.sleep(0.001)
                    continue
                
                # 读取所有可用数据
                if self.serial.in_waiting > 0:
                    data = self.serial.read(self.serial.in_waiting)
                    self.buffer.extend(data)
                
                # 处理所有完整的数据包，只保留最新的
                latest_raw_angles = None
                while True:
                    frame_end = self.buffer.find(SerialConfig.FRAME_TAIL)
                    if frame_end == -1:
                        break
                    
                    if frame_end < SerialConfig.PACKET_SIZE - len(SerialConfig.FRAME_TAIL):
                        # 数据不足，丢弃前面的数据
                        self.buffer = self.buffer[frame_end + len(SerialConfig.FRAME_TAIL):]
                        continue
                    
                    packet_start = frame_end - (SerialConfig.PACKET_SIZE - len(SerialConfig.FRAME_TAIL))
                    if packet_start < 0:
                        self.buffer = self.buffer[frame_end + len(SerialConfig.FRAME_TAIL):]
                        continue
                    
                    packet = self.buffer[packet_start:frame_end]
                    self.buffer = self.buffer[frame_end + len(SerialConfig.FRAME_TAIL):]
                    
                    if len(packet) == SerialConfig.PACKET_SIZE - len(SerialConfig.FRAME_TAIL):
                        latest_raw_angles = list(struct.unpack('<6f', packet))
                        self.packet_count += 1
                    else:
                        self.error_count += 1
                
                # 如果有新数据，应用滤波并更新
                if latest_raw_angles is not None:
                    if ControlConfig.ENABLE_FILTER:
                        if self.is_first_read:
                            self.filtered_angles = latest_raw_angles.copy()
                            self.is_first_read = False
                        else:
                            for i in range(6):
                                self.filtered_angles[i] = (ControlConfig.FILTER_ALPHA * latest_raw_angles[i] + 
                                                           (1 - ControlConfig.FILTER_ALPHA) * self.filtered_angles[i])
                        angles = self.filtered_angles.copy()
                    else:
                        angles = latest_raw_angles
                    
                    with self._lock:
                        self._latest_angles = angles
                        self.last_angles = angles
                
                # 防止缓冲区过大
                if len(self.buffer) > SerialConfig.PACKET_SIZE * 10:
                    self.buffer = self.buffer[-SerialConfig.PACKET_SIZE * 2:]
                
                time.sleep(0.001)  # 1ms循环
                
            except Exception as e:
                self.error_count += 1
                time.sleep(0.001)
    
    def start(self):
        """
        @brief 启动读取线程
        """
        if self._running:
            return
        self._running = True
        self._thread = threading.Thread(target=self._read_loop, daemon=True)
        self._thread.start()
    
    def stop(self):
        """
        @brief 停止读取线程
        """
        self._running = False
        if self._thread:
            self._thread.join(timeout=1.0)
            self._thread = None
    
    def read_packet(self) -> Optional[List[float]]:
        """
        @brief 获取最新数据包 (非阻塞，立即返回)
        """
        with self._lock:
            angles = self._latest_angles
            self._latest_angles = None  # 消费后清除，避免重复使用
            return angles
    
    def get_latest_angles(self) -> List[float]:
        """
        @brief 获取最新角度 (总是返回，可能是旧数据)
        """
        with self._lock:
            return self.last_angles.copy()


# ============================================================================
# 角度映射函数
# ============================================================================

def normalize_angle(angle: float) -> float:
    """
    @brief 将原始角度限制在[-360°, 720°]范围内
    @details 控制器输出的原始角度工作空间为[-360°, 720°]。
             本函数只做“裁剪”，不再按360度做周期性折返，
             后续的关节映射逻辑直接在这个连续角度空间内工作。
    """
    # 先限制在物理给定的原始角度范围，避免异常值带来跳变
    if angle < -360.0:
        angle = -360.0
    elif angle > 720.0:
        angle = 720.0
    return angle


def map_angle_to_position(angle: float, angle_min: float, angle_max: float, reversed: bool = False) -> int:
    """
    @brief 将角度值映射到关节位置
    """
    # 将输入角度以及配置范围裁剪到统一工作空间[-360°, 720°]
    angle = normalize_angle(angle)
    angle_min = normalize_angle(angle_min)
    angle_max = normalize_angle(angle_max)

    # 避免除零：当最小角度和最大角度非常接近时，视为常量映射
    if abs(angle_max - angle_min) < 1e-6:
        ratio = 0.0
    else:
        if angle <= angle_min:
            ratio = 0.0
        elif angle >= angle_max:
            ratio = 1.0
        else:
            ratio = (angle - angle_min) / (angle_max - angle_min)
    
    if reversed:
        ratio = 1.0 - ratio
    
    position = int(ratio * (ControlConfig.JOINT_POS_MAX - ControlConfig.JOINT_POS_MIN) + ControlConfig.JOINT_POS_MIN)
    position = max(ControlConfig.JOINT_POS_MIN, min(ControlConfig.JOINT_POS_MAX, position))
    
    return position


def map_angles_to_positions(angles: List[float]) -> List[int]:
    """
    @brief 将6个角度值映射到6个关节位置
    """
    positions = []
    for i in range(6):
        angle_min, angle_max = ControlConfig.JOINT_ANGLE_RANGES[i]
        reversed_flag = ControlConfig.JOINT_REVERSED[i]
        position = map_angle_to_position(angles[i], angle_min, angle_max, reversed_flag)
        positions.append(position)
    return positions


# ============================================================================
# 集成可视化器
# ============================================================================

class IntegratedVisualizer:
    """
    @brief 集成可视化器 (高性能优化版)
    @details 显示触觉传感器热力图和机械手反馈数据
             控制逻辑在独立高频线程中运行
    """
    
    def __init__(self, can_manager: UnifiedCANManager, controller: Optional[ControllerReader] = None):
        self.can_manager = can_manager
        self.controller = controller
        
        try:
            import pygame
            self.pygame = pygame
        except ImportError:
            print("错误: 未安装pygame")
            print("请运行: pip install pygame")
            sys.exit(1)
        
        self.pygame.init()
        hand_name = '左手' if can_manager.hand == 0 else '右手'
        self.pygame.display.set_caption(f"触觉传感器与机械手控制 - {hand_name}")
        
        self.screen = self.pygame.display.set_mode(
            (VisualConfig.WINDOW_WIDTH, VisualConfig.WINDOW_HEIGHT)
        )
        self.clock = self.pygame.time.Clock()
        
        self._init_fonts()
        self._generate_colormap()
        self._calculate_layout()
        
        # 预渲染表面
        self._finger_surfaces = [
            self.pygame.Surface((
                CANConfig.MATRIX_ROWS * (VisualConfig.CELL_SIZE + VisualConfig.CELL_MARGIN),
                CANConfig.MATRIX_COLS * (VisualConfig.CELL_SIZE + VisualConfig.CELL_MARGIN)
            ))
            for _ in range(CANConfig.NUM_FINGERS)
        ]
        
        self._fps_history = deque(maxlen=60)
        self._running = True
        self._control_enabled = controller is not None
        
        # 控制相关 - 使用锁保护共享数据
        self._control_lock = threading.Lock()
        self._last_positions = [512] * 6
        self._controller_angles = [0.0] * 6
        self._control_rate = 0.0
        self._control_count = 0
        self._control_start_time = time.time()
        
        # 独立控制线程
        self._control_thread: Optional[threading.Thread] = None
        self._control_running = False
    
    def _init_fonts(self):
        """
        @brief 初始化字体
        """
        self.pygame.font.init()
        font_names = ['Microsoft YaHei', 'SimHei', 'STHeiti', 'Arial Unicode MS', 'Arial']
        self.font = None
        for font_name in font_names:
            try:
                self.font = self.pygame.font.SysFont(font_name, 18)
                self.font_small = self.pygame.font.SysFont(font_name, 13)
                self.font_large = self.pygame.font.SysFont(font_name, 24)
                break
            except:
                continue
        if self.font is None:
            self.font = self.pygame.font.Font(None, 18)
            self.font_small = self.pygame.font.Font(None, 13)
            self.font_large = self.pygame.font.Font(None, 24)
    
    def _generate_colormap(self):
        """
        @brief 生成颜色映射表
        """
        self.colormap = []
        key_colors = [
            (0,   (20, 20, 50)),
            (50,  (30, 60, 150)),
            (100, (50, 150, 200)),
            (150, (100, 200, 100)),
            (200, (255, 255, 50)),
            (230, (255, 150, 30)),
            (255, (255, 50, 30)),
        ]
        
        for i in range(256):
            color = None
            for j in range(len(key_colors) - 1):
                v1, c1 = key_colors[j]
                v2, c2 = key_colors[j + 1]
                if v1 <= i <= v2:
                    t = (i - v1) / (v2 - v1) if v2 > v1 else 0
                    r = int(c1[0] + (c2[0] - c1[0]) * t)
                    g = int(c1[1] + (c2[1] - c1[1]) * t)
                    b = int(c1[2] + (c2[2] - c1[2]) * t)
                    color = (r, g, b)
                    break
            if color is None:
                color = key_colors[-1][1]
            self.colormap.append(color)
    
    def _calculate_layout(self):
        """
        @brief 计算布局
        """
        # 热力图尺寸 (旋转后)
        heatmap_width = CANConfig.MATRIX_ROWS * (VisualConfig.CELL_SIZE + VisualConfig.CELL_MARGIN)
        heatmap_height = CANConfig.MATRIX_COLS * (VisualConfig.CELL_SIZE + VisualConfig.CELL_MARGIN)
        
        total_width = 5 * heatmap_width + 4 * VisualConfig.FINGER_SPACING
        start_x = (VisualConfig.WINDOW_WIDTH - total_width) // 2
        start_y = 100
        
        self.finger_positions = []
        for i in range(5):
            x = start_x + i * (heatmap_width + VisualConfig.FINGER_SPACING)
            self.finger_positions.append((x, start_y))
        
        self.heatmap_width = heatmap_width
        self.heatmap_height = heatmap_height
        
        # 机械手反馈区域
        self.feedback_y = start_y + heatmap_height + 80
        self.bar_width = 180
        self.bar_height = 20
        self.bar_spacing = 8
    
    def _draw_heatmap(self, surface, matrix: np.ndarray):
        """
        @brief 绘制热力图
        """
        surface.fill(VisualConfig.BG_COLOR)
        
        rotated = np.rot90(matrix, k=-1)
        rotated = np.fliplr(rotated)
        
        cell_size = VisualConfig.CELL_SIZE
        margin = VisualConfig.CELL_MARGIN
        
        rotated_rows, rotated_cols = rotated.shape
        
        for row in range(rotated_rows):
            for col in range(rotated_cols):
                value = rotated[row, col]
                color = self.colormap[value]
                x = col * (cell_size + margin)
                y = row * (cell_size + margin)
                self.pygame.draw.rect(surface, color, (x, y, cell_size, cell_size))
    
    def _draw_finger_labels(self):
        """
        @brief 绘制手指标签
        """
        fingers_data = self.can_manager.get_all_fingers_data()
        
        for i, (pos, data) in enumerate(zip(self.finger_positions, fingers_data)):
            x, y = pos
            
            name = CANConfig.FINGER_NAMES[i]
            text = self.font.render(f"{name}", True, VisualConfig.TEXT_COLOR)
            text_rect = text.get_rect(centerx=x + self.heatmap_width // 2, bottom=y - 5)
            self.screen.blit(text, text_rect)
            
            if data.valid:
                max_val = data.matrix.max()
                avg_val = data.matrix.mean()
                info_text = f"Max:{max_val:3d} Avg:{avg_val:.1f}"
            else:
                info_text = "等待数据..."
            
            info_surface = self.font_small.render(info_text, True, VisualConfig.TEXT_COLOR)
            info_rect = info_surface.get_rect(centerx=x + self.heatmap_width // 2, 
                                              top=y + self.heatmap_height + 5)
            self.screen.blit(info_surface, info_rect)
    
    def _draw_progress_bar(self, x: int, y: int, width: int, height: int, 
                           value: int, max_val: int, color: tuple, label: str):
        """
        @brief 绘制进度条
        """
        # 背景
        self.pygame.draw.rect(self.screen, VisualConfig.BAR_BG_COLOR, (x, y, width, height))
        
        # 填充
        fill_width = int(width * value / max_val) if max_val > 0 else 0
        if fill_width > 0:
            self.pygame.draw.rect(self.screen, color, (x, y, fill_width, height))
        
        # 边框
        self.pygame.draw.rect(self.screen, VisualConfig.BORDER_COLOR, (x, y, width, height), 1)
        
        # 标签
        label_surface = self.font_small.render(label, True, VisualConfig.TEXT_COLOR)
        self.screen.blit(label_surface, (x - 30, y + 2))
        
        # 数值
        value_surface = self.font_small.render(f"{value}", True, VisualConfig.TEXT_COLOR)
        self.screen.blit(value_surface, (x + width + 5, y + 2))
    
    def _draw_robot_feedback(self):
        """
        @brief 绘制机械手反馈数据
        """
        feedback = self.can_manager.get_robot_feedback()
        
        # 标题
        title = "机械手反馈数据"
        title_surface = self.font.render(title, True, VisualConfig.TEXT_COLOR)
        title_rect = title_surface.get_rect(centerx=VisualConfig.WINDOW_WIDTH // 2, y=self.feedback_y - 30)
        self.screen.blit(title_surface, title_rect)
        
        # 计算起始位置，使6个关节居中
        total_joint_width = 6 * (self.bar_width + 80) - 80
        start_x = (VisualConfig.WINDOW_WIDTH - total_joint_width) // 2
        
        for i in range(6):
            joint_x = start_x + i * (self.bar_width + 80)
            joint_y = self.feedback_y
            
            # 关节名称
            joint_name = f"J{i}"
            name_surface = self.font.render(joint_name, True, VisualConfig.TEXT_COLOR)
            name_rect = name_surface.get_rect(centerx=joint_x + self.bar_width // 2 + 15, y=joint_y)
            self.screen.blit(name_surface, name_rect)
            
            # 位置条
            self._draw_progress_bar(
                joint_x + 30, joint_y + 25, self.bar_width, self.bar_height,
                feedback.position[i], 1023, VisualConfig.BAR_POS_COLOR, "Pos"
            )
            
            # 速度条
            self._draw_progress_bar(
                joint_x + 30, joint_y + 25 + self.bar_height + self.bar_spacing, 
                self.bar_width, self.bar_height,
                feedback.velocity[i], 1023, VisualConfig.BAR_VEL_COLOR, "Vel"
            )
            
            # 力矩条
            self._draw_progress_bar(
                joint_x + 30, joint_y + 25 + 2 * (self.bar_height + self.bar_spacing), 
                self.bar_width, self.bar_height,
                feedback.torque[i], 1023, VisualConfig.BAR_TOR_COLOR, "Tor"
            )
        
        # 图例
        legend_y = self.feedback_y + 25 + 3 * (self.bar_height + self.bar_spacing) + 20
        legend_items = [
            ("位置 (Position)", VisualConfig.BAR_POS_COLOR),
            ("速度 (Velocity)", VisualConfig.BAR_VEL_COLOR),
            ("力矩 (Torque)", VisualConfig.BAR_TOR_COLOR),
        ]
        legend_x = VisualConfig.WINDOW_WIDTH // 2 - 200
        for text, color in legend_items:
            self.pygame.draw.rect(self.screen, color, (legend_x, legend_y, 15, 15))
            text_surface = self.font_small.render(text, True, VisualConfig.TEXT_COLOR)
            self.screen.blit(text_surface, (legend_x + 20, legend_y))
            legend_x += 150
    
    def _draw_controller_status(self):
        """
        @brief 绘制控制器状态
        """
        if not self._control_enabled:
            return
        
        y = self.feedback_y + 160
        
        # 标题
        title = "控制器输入"
        title_surface = self.font.render(title, True, VisualConfig.TEXT_COLOR)
        title_rect = title_surface.get_rect(centerx=VisualConfig.WINDOW_WIDTH // 2, y=y)
        self.screen.blit(title_surface, title_rect)
        
        # 获取数据副本 (线程安全)
        with self._control_lock:
            angles = self._controller_angles.copy()
            positions = self._last_positions.copy()
        
        # 角度和位置值
        angles_text = "角度: " + " | ".join([f"J{i}:{angles[i]:.1f}°" for i in range(6)])
        angles_surface = self.font_small.render(angles_text, True, VisualConfig.TEXT_COLOR)
        angles_rect = angles_surface.get_rect(centerx=VisualConfig.WINDOW_WIDTH // 2, y=y + 25)
        self.screen.blit(angles_surface, angles_rect)
        
        pos_text = "位置: " + " | ".join([f"J{i}:{positions[i]}" for i in range(6)])
        pos_surface = self.font_small.render(pos_text, True, VisualConfig.TEXT_COLOR)
        pos_rect = pos_surface.get_rect(centerx=VisualConfig.WINDOW_WIDTH // 2, y=y + 45)
        self.screen.blit(pos_surface, pos_rect)
    
    def _draw_colorbar(self):
        """
        @brief 绘制颜色条
        """
        bar_width = 20
        bar_height = self.heatmap_height
        x = VisualConfig.WINDOW_WIDTH - 70
        y = self.finger_positions[0][1]
        
        for i in range(bar_height):
            value = int(255 * (bar_height - 1 - i) / (bar_height - 1))
            color = self.colormap[value]
            self.pygame.draw.line(self.screen, color, (x, y + i), (x + bar_width, y + i))
        
        self.pygame.draw.rect(self.screen, VisualConfig.BORDER_COLOR, 
                             (x - 1, y - 1, bar_width + 2, bar_height + 2), 1)
        
        labels = ['255', '128', '0']
        positions = [y, y + bar_height // 2, y + bar_height]
        for label, pos in zip(labels, positions):
            text = self.font_small.render(label, True, VisualConfig.TEXT_COLOR)
            self.screen.blit(text, (x + bar_width + 5, pos - 7))
    
    def _draw_stats(self):
        """
        @brief 绘制统计信息
        """
        stats = self.can_manager.get_stats()
        
        current_fps = self.clock.get_fps()
        self._fps_history.append(current_fps)
        avg_fps = sum(self._fps_history) / len(self._fps_history) if self._fps_history else 0
        
        hand_name = "左手" if self.can_manager.hand == 0 else "右手"
        
        title = f"触觉传感器与机械手控制系统 - {hand_name}"
        title_surface = self.font_large.render(title, True, VisualConfig.TEXT_COLOR)
        title_rect = title_surface.get_rect(centerx=VisualConfig.WINDOW_WIDTH // 2, y=10)
        self.screen.blit(title_surface, title_rect)
        
        tactile_rate = stats['tactile_rate'] / 5 if stats['tactile_rate'] > 0 else 0
        
        # 获取控制频率
        with self._control_lock:
            control_rate = self._control_rate
        
        stats_text = (
            f"FPS: {avg_fps:.1f} | "
            f"控制: {control_rate:.0f} Hz | "
            f"触觉: {tactile_rate:.1f} Hz/指 | "
            f"TX: {stats['tx_frames']} | "
            f"运行: {stats['elapsed']:.1f}s"
        )
        stats_surface = self.font.render(stats_text, True, VisualConfig.TEXT_COLOR)
        stats_rect = stats_surface.get_rect(centerx=VisualConfig.WINDOW_WIDTH // 2, y=45)
        self.screen.blit(stats_surface, stats_rect)
        
        control_status = "控制器: 运行中" if self._control_enabled else "控制器: 已禁用"
        help_text = f"{control_status} | 按 Q/ESC 退出 | 按 R 重置统计 | 按 C 切换控制"
        help_surface = self.font_small.render(help_text, True, (150, 150, 150))
        help_rect = help_surface.get_rect(centerx=VisualConfig.WINDOW_WIDTH // 2, 
                                          y=VisualConfig.WINDOW_HEIGHT - 25)
        self.screen.blit(help_surface, help_rect)
    
    def _control_loop(self):
        """
        @brief 独立高频控制循环 (在单独线程中运行)
        @details 以200Hz频率运行，不受渲染帧率影响
        """
        last_send_time = time.time()
        rate_calc_time = time.time()
        rate_count = 0
        
        while self._control_running:
            if not self._control_enabled or not self.controller:
                time.sleep(0.001)
                continue
            
            current_time = time.time()
            
            # 检查是否到发送时间
            if (current_time - last_send_time) >= ControlConfig.SEND_PERIOD:
                # 获取最新角度数据
                angles = self.controller.get_latest_angles()
                
                if angles:
                    positions = map_angles_to_positions(angles)
                    
                    # 发送控制命令
                    self.can_manager.send_control(position=positions)
                    last_send_time = current_time
                    rate_count += 1
                    
                    # 更新共享数据
                    with self._control_lock:
                        self._controller_angles = angles
                        self._last_positions = positions
                        self._control_count += 1
            
            # 每秒计算一次控制频率
            if current_time - rate_calc_time >= 1.0:
                with self._control_lock:
                    self._control_rate = rate_count / (current_time - rate_calc_time)
                rate_count = 0
                rate_calc_time = current_time
            
            # 高频循环，最小延时
            time.sleep(0.001)  # 1ms = 1000Hz max
    
    def _start_control_thread(self):
        """
        @brief 启动控制线程
        """
        if self._control_thread is not None:
            return
        
        self._control_running = True
        self._control_thread = threading.Thread(target=self._control_loop, daemon=True)
        self._control_thread.start()
        print("控制线程已启动")
    
    def _stop_control_thread(self):
        """
        @brief 停止控制线程
        """
        self._control_running = False
        if self._control_thread:
            self._control_thread.join(timeout=1.0)
            self._control_thread = None
    
    def update(self) -> bool:
        """
        @brief 更新显示 (仅渲染，控制在独立线程)
        """
        for event in self.pygame.event.get():
            if event.type == self.pygame.QUIT:
                self._running = False
            elif event.type == self.pygame.KEYDOWN:
                if event.key in (self.pygame.K_q, self.pygame.K_ESCAPE):
                    self._running = False
                elif event.key == self.pygame.K_r:
                    self.can_manager._stats['start_time'] = time.time()
                    self.can_manager._stats['tactile_frames'] = 0
                    self.can_manager._stats['robot_frames'] = 0
                    self.can_manager._stats['tx_frames'] = 0
                    self.can_manager._stats['finger_updates'] = [0] * 5
                    with self._control_lock:
                        self._control_count = 0
                        self._control_rate = 0.0
                        self._control_start_time = time.time()
                elif event.key == self.pygame.K_c:
                    self._control_enabled = not self._control_enabled
                    status = "启用" if self._control_enabled else "禁用"
                    print(f"控制已{status}")
        
        if not self._running:
            return False
        
        # 清屏
        self.screen.fill(VisualConfig.BG_COLOR)
        
        # 绘制触觉热力图
        matrices = self.can_manager.get_all_matrices()
        for i, (surface, matrix, pos) in enumerate(zip(
            self._finger_surfaces, matrices, self.finger_positions
        )):
            self._draw_heatmap(surface, matrix)
            self.screen.blit(surface, pos)
            self.pygame.draw.rect(
                self.screen, VisualConfig.BORDER_COLOR,
                (pos[0] - 1, pos[1] - 1, self.heatmap_width + 2, self.heatmap_height + 2),
                1
            )
        
        self._draw_finger_labels()
        self._draw_colorbar()
        self._draw_robot_feedback()
        self._draw_controller_status()
        self._draw_stats()
        
        self.pygame.display.flip()
        self.clock.tick(VisualConfig.FPS)
        
        return True
    
    def run(self):
        """
        @brief 运行主循环
        """
        print(f"启动可视化 (目标帧率: {VisualConfig.FPS} FPS)")
        print(f"控制频率: {ControlConfig.SEND_FREQUENCY} Hz")
        print("按 Q 或 ESC 退出...")
        
        # 启动独立控制线程
        if self.controller:
            self._start_control_thread()
        
        try:
            while self.update():
                pass
        finally:
            self._stop_control_thread()
            self.close()
    
    def close(self):
        """
        @brief 关闭可视化器
        """
        self._stop_control_thread()
        self.pygame.quit()


# ============================================================================
# 模拟数据生成器
# ============================================================================

class SimulatedCANManager:
    """
    @brief 模拟CAN管理器
    """
    
    def __init__(self, hand: int = 0):
        self.hand = hand
        self._lock = threading.Lock()
        self._running = False
        self._thread = None
        
        self._finger_data = [FingerData(finger_id=i) for i in range(5)]
        self._robot_feedback = RobotHandFeedback()
        
        self._stats = {
            'tactile_frames': 0,
            'robot_frames': 0,
            'tx_frames': 0,
            'finger_updates': [0] * 5,
            'start_time': 0.0
        }
        self._time_offset = 0
    
    def _simulate_loop(self):
        while self._running:
            with self._lock:
                self._time_offset += 0.016
                
                for i, finger in enumerate(self._finger_data):
                    for row in range(CANConfig.MATRIX_ROWS):
                        for col in range(CANConfig.MATRIX_COLS):
                            phase = self._time_offset * 2 + i * 0.5
                            wave1 = np.sin(col * 0.5 + phase) * 0.5 + 0.5
                            wave2 = np.cos(row * 0.8 + phase * 0.7) * 0.5 + 0.5
                            center_col, center_row = 5.5, 2.5
                            dist = np.sqrt((col - center_col)**2 + (row - center_row)**2)
                            center_weight = max(0, 1 - dist / 6) * np.sin(phase) * 0.5 + 0.5
                            value = int((wave1 * wave2 * 0.5 + center_weight * 0.5) * 200 + 30)
                            finger.matrix[row, col] = min(255, max(0, value))
                    
                    finger.valid = True
                    finger.timestamp = time.time()
                    finger.update_count += 1
                    self._stats['finger_updates'][i] += 1
                
                # 模拟机械手反馈
                for i in range(6):
                    self._robot_feedback.position[i] = int(512 + 400 * np.sin(self._time_offset + i * 0.5))
                    self._robot_feedback.velocity[i] = int(abs(400 * np.cos(self._time_offset + i * 0.5)))
                    self._robot_feedback.torque[i] = int(200 + 150 * np.sin(self._time_offset * 2 + i))
                self._robot_feedback.valid = True
                self._robot_feedback.timestamp = time.time()
                
                self._stats['tactile_frames'] += 45
                self._stats['robot_frames'] += 3
            
            time.sleep(0.016)
    
    def connect(self) -> bool:
        print("使用模拟数据模式")
        return True
    
    def start(self):
        if self._running:
            return
        self._running = True
        self._stats['start_time'] = time.time()
        self._thread = threading.Thread(target=self._simulate_loop, daemon=True)
        self._thread.start()
    
    def stop(self):
        self._running = False
        if self._thread:
            self._thread.join(timeout=1.0)
    
    def disconnect(self):
        self.stop()
    
    def send_control(self, position=None, velocity=None, torque=None):
        self._stats['tx_frames'] += 1
    
    def get_all_matrices(self) -> List[np.ndarray]:
        with self._lock:
            return [f.matrix.copy() for f in self._finger_data]
    
    def get_all_fingers_data(self) -> List[FingerData]:
        with self._lock:
            return [FingerData(
                finger_id=f.finger_id,
                matrix=f.matrix.copy(),
                valid=f.valid,
                timestamp=f.timestamp,
                update_count=f.update_count
            ) for f in self._finger_data]
    
    def get_robot_feedback(self) -> RobotHandFeedback:
        with self._lock:
            return RobotHandFeedback(
                position=self._robot_feedback.position.copy(),
                velocity=self._robot_feedback.velocity.copy(),
                torque=self._robot_feedback.torque.copy(),
                timestamp=self._robot_feedback.timestamp,
                valid=self._robot_feedback.valid
            )
    
    def get_stats(self) -> dict:
        elapsed = time.time() - self._stats['start_time'] if self._stats['start_time'] > 0 else 0
        total_tactile = sum(self._stats['finger_updates'])
        return {
            'tactile_frames': self._stats['tactile_frames'],
            'robot_frames': self._stats['robot_frames'],
            'tx_frames': self._stats['tx_frames'],
            'finger_updates': self._stats['finger_updates'].copy(),
            'total_tactile_updates': total_tactile,
            'elapsed': elapsed,
            'tactile_rate': total_tactile / elapsed if elapsed > 0 else 0
        }


# ============================================================================
# 主程序
# ============================================================================

def main():
    import argparse
    
    parser = argparse.ArgumentParser(description='触觉传感器与机械手控制集成系统')
    parser.add_argument('--hand', type=int, default=1, choices=[0, 1],
                        help='选择手: 0=左手, 1=右手')
    parser.add_argument('--channel', type=str, default=None,
                        help='CAN通道')
    parser.add_argument('--interface', type=str, default=None,
                        help='CAN接口')
    parser.add_argument('--serial-port', type=str, default=None,
                        help='控制器串口')
    parser.add_argument('--simulate', action='store_true',
                        help='使用模拟数据')
    parser.add_argument('--no-control', action='store_true',
                        help='禁用控制器')
    parser.add_argument('--fps', type=int, default=60,
                        help='目标帧率')
    
    args = parser.parse_args()
    
    VisualConfig.FPS = args.fps
    if args.serial_port:
        SerialConfig.PORT = args.serial_port
    
    hand_name = "左手" if args.hand == 0 else "右手"
    print("=" * 70)
    print(f" 触觉传感器与机械手控制集成系统 - {hand_name}")
    print("=" * 70)
    
    # 创建CAN管理器
    if args.simulate:
        can_manager = SimulatedCANManager(hand=args.hand)
    else:
        can_manager = UnifiedCANManager(
            channel=args.channel,
            interface=args.interface,
            hand=args.hand
        )
    
    # 创建控制器
    controller = None
    if not args.no_control and not args.simulate:
        controller = ControllerReader()
        if not controller.connect():
            print("控制器连接失败，将禁用控制功能")
            controller = None
        else:
            # 启动控制器读取线程
            controller.start()
    
    try:
        if not can_manager.connect():
            print("CAN连接失败，切换到模拟模式...")
            can_manager = SimulatedCANManager(hand=args.hand)
            can_manager.connect()
        
        can_manager.start()
        
        visualizer = IntegratedVisualizer(can_manager, controller)
        visualizer.run()
        
    except KeyboardInterrupt:
        print("\n用户中断")
    finally:
        if controller:
            controller.disconnect()
        can_manager.disconnect()
        
        stats = can_manager.get_stats()
        print("\n" + "=" * 70)
        print(" 最终统计")
        print("=" * 70)
        print(f"  触觉传感器帧: {stats['tactile_frames']}")
        print(f"  机械手反馈帧: {stats['robot_frames']}")
        print(f"  发送命令帧: {stats['tx_frames']}")
        print(f"  运行时间: {stats['elapsed']:.2f}s")
        print("=" * 70)


if __name__ == "__main__":
    main()
