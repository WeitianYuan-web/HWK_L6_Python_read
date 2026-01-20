#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
@file tactile_visualizer_5finger.py
@brief 五指触觉传感器高性能可视化程序
@details 使用PyGame实现60Hz刷新率的触觉传感器可视化
         支持左手(0x300-0x348)和右手(0x400-0x448)

@version 1.0
@date 2026-01-20

CAN协议说明:
- CAN ID格式: 0xHmn (H=手臂标识, m=手指ID, n=帧序号)
- 左手: H=3 (0x300-0x348)
- 右手: H=4 (0x400-0x448)
- 每个手指: 12列×6行 = 72字节, 分9帧传输
"""

import can
import time
import threading
import numpy as np
from collections import deque
from dataclasses import dataclass, field
from typing import Optional, List, Tuple, Dict
import sys
import os


# ============================================================================
# 配置类
# ============================================================================

class CANConfig:
    """
    @brief CAN配置常量
    """
    # CAN接口配置
    CHANNEL = 'PCAN_USBBUS1'  # Windows PCAN: 'PCAN_USBBUS1', Linux: 'can0'
    INTERFACE = 'pcan'         # Windows PCAN: 'pcan', Linux: 'socketcan'
    BITRATE = 1000000          # 1Mbps
    
    # 帧配置
    FRAMES_PER_FINGER = 9      # 每手指9帧
    BYTES_PER_FRAME = 8        # 每帧8字节
    TOTAL_BYTES = 72           # 每手指72字节
    
    # 矩阵配置
    MATRIX_COLS = 12           # 矩阵列数
    MATRIX_ROWS = 6            # 矩阵行数
    
    # 手指配置
    NUM_FINGERS = 5
    FINGER_NAMES = ['拇指', '食指', '中指', '无名指', '小指']
    FINGER_NAMES_EN = ['Thumb', 'Index', 'Middle', 'Ring', 'Pinky']
    
    # CAN ID基址
    LEFT_HAND_BASE = 0x300     # 左手基址
    RIGHT_HAND_BASE = 0x400    # 右手基址


class VisualConfig:
    """
    @brief 可视化配置常量
    """
    # 窗口配置
    WINDOW_WIDTH = 1600
    WINDOW_HEIGHT = 750
    FPS = 60                   # 目标帧率
    
    # 热力图配置
    CELL_SIZE = 32             # 单元格大小(像素) - 增大显示
    CELL_MARGIN = 2            # 单元格间距
    HEATMAP_MARGIN = 20        # 热力图外边距
    
    # 手指热力图布局
    FINGER_SPACING = 30        # 手指热力图间距
    
    # 颜色配置 (R, G, B)
    BG_COLOR = (30, 30, 40)
    TEXT_COLOR = (220, 220, 220)
    BORDER_COLOR = (80, 80, 100)
    
    # 热力图颜色映射 (值: 0-255 -> 颜色)
    # 从深蓝到红色的渐变
    COLORMAP = None  # 动态生成


# ============================================================================
# 数据结构
# ============================================================================

@dataclass
class FingerData:
    """
    @brief 单个手指的传感器数据
    """
    finger_id: int = 0
    raw_data: bytearray = field(default_factory=lambda: bytearray(72))
    matrix: np.ndarray = field(default_factory=lambda: np.zeros((CANConfig.MATRIX_ROWS, CANConfig.MATRIX_COLS), dtype=np.uint8))
    frame_mask: int = 0        # 已接收帧掩码(9位)
    timestamp: float = 0.0
    valid: bool = False
    update_count: int = 0      # 更新计数


@dataclass
class HandData:
    """
    @brief 单手(5指)的传感器数据
    """
    hand_id: int = 0           # 0=左手, 1=右手
    fingers: List[FingerData] = field(default_factory=list)
    
    def __post_init__(self):
        if not self.fingers:
            self.fingers = [FingerData(finger_id=i) for i in range(CANConfig.NUM_FINGERS)]


# ============================================================================
# CAN数据接收器
# ============================================================================

class TactileCANReceiver:
    """
    @brief 触觉传感器CAN数据接收器
    @details 在独立线程中接收CAN数据，支持左右手
    """
    
    def __init__(self, channel: str = None, interface: str = None, 
                 bitrate: int = None, hand: int = 0):
        """
        @brief 初始化接收器
        @param channel CAN通道
        @param interface CAN接口类型
        @param bitrate 波特率
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
        
        # 根据手选择基址
        self.base_id = CANConfig.LEFT_HAND_BASE if hand == 0 else CANConfig.RIGHT_HAND_BASE
        
        # 初始化5指数据
        self._hand_data = HandData(hand_id=hand)
        
        # 统计信息
        self._stats = {
            'total_frames': 0,
            'finger_updates': [0] * 5,
            'start_time': 0.0,
            'fps_samples': deque(maxlen=60)
        }
        self._last_fps_time = time.time()
        self._fps_frame_count = 0
    
    def _parse_can_id(self, can_id: int) -> Tuple[int, int, int]:
        """
        @brief 解析CAN ID
        @param can_id CAN标识符
        @return (hand, finger_id, frame_idx) 或 (-1, -1, -1) 如果无效
        """
        base = (can_id >> 8) & 0x0F
        
        if base == 0x03:
            hand = 0  # 左手
        elif base == 0x04:
            hand = 1  # 右手
        else:
            return (-1, -1, -1)
        
        finger_id = (can_id >> 4) & 0x0F
        frame_idx = can_id & 0x0F
        
        if finger_id > 4 or frame_idx > 8:
            return (-1, -1, -1)
        
        return (hand, finger_id, frame_idx)
    
    def _convert_to_matrix(self, raw_data: bytearray) -> np.ndarray:
        """
        @brief 将72字节原始数据转换为6×12矩阵(行优先)
        @details 原始数据为列优先排列，转换为行优先便于显示
        @param raw_data 72字节原始数据
        @return 6×12 numpy数组
        """
        matrix = np.zeros((CANConfig.MATRIX_ROWS, CANConfig.MATRIX_COLS), dtype=np.uint8)
        idx = 0
        for col in range(CANConfig.MATRIX_COLS):
            for row in range(CANConfig.MATRIX_ROWS):
                matrix[row, col] = raw_data[idx]
                idx += 1
        return matrix
    
    def _process_frame(self, msg: can.Message) -> bool:
        """
        @brief 处理单个CAN帧
        @param msg CAN消息
        @return 是否有数据更新完成
        """
        hand, finger_id, frame_idx = self._parse_can_id(msg.arbitration_id)
        
        if hand < 0 or hand != self.hand:
            return False  # 无效帧或不是本手的数据
        
        self._stats['total_frames'] += 1
        
        with self._lock:
            finger = self._hand_data.fingers[finger_id]
            
            # 存储数据
            offset = frame_idx * CANConfig.BYTES_PER_FRAME
            for i, byte in enumerate(msg.data[:8]):
                if offset + i < CANConfig.TOTAL_BYTES:
                    finger.raw_data[offset + i] = byte
            
            # 更新帧掩码
            finger.frame_mask |= (1 << frame_idx)
            
            # 检查是否收齐9帧
            if finger.frame_mask == 0x1FF:
                finger.matrix = self._convert_to_matrix(finger.raw_data)
                finger.timestamp = time.time()
                finger.valid = True
                finger.update_count += 1
                finger.frame_mask = 0  # 重置
                self._stats['finger_updates'][finger_id] += 1
                return True
        
        return False
    
    def _receive_loop(self):
        """
        @brief CAN接收循环
        """
        while self._running:
            try:
                # 使用非阻塞接收以便快速响应停止请求
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
        @return 是否连接成功
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
        @brief 停止接收数据
        """
        self._running = False
        if self._thread:
            self._thread.join(timeout=1.0)
            self._thread = None
    
    def disconnect(self):
        """
        @brief 断开CAN连接
        """
        self.stop()
        if self.bus:
            self.bus.shutdown()
            self.bus = None
    
    def get_finger_data(self, finger_id: int) -> FingerData:
        """
        @brief 获取指定手指数据(线程安全)
        @param finger_id 手指ID (0-4)
        @return FingerData副本
        """
        with self._lock:
            src = self._hand_data.fingers[finger_id]
            return FingerData(
                finger_id=src.finger_id,
                raw_data=bytearray(src.raw_data),
                matrix=src.matrix.copy(),
                frame_mask=src.frame_mask,
                timestamp=src.timestamp,
                valid=src.valid,
                update_count=src.update_count
            )
    
    def get_all_fingers_data(self) -> List[FingerData]:
        """
        @brief 获取所有手指数据
        @return 5个FingerData的列表
        """
        return [self.get_finger_data(i) for i in range(CANConfig.NUM_FINGERS)]
    
    def get_all_matrices(self) -> List[np.ndarray]:
        """
        @brief 高性能获取所有矩阵数据
        @return 5个矩阵的列表
        """
        with self._lock:
            return [f.matrix.copy() for f in self._hand_data.fingers]
    
    def get_stats(self) -> dict:
        """
        @brief 获取统计信息
        @return 统计字典
        """
        elapsed = time.time() - self._stats['start_time'] if self._stats['start_time'] > 0 else 0
        total_updates = sum(self._stats['finger_updates'])
        
        return {
            'total_frames': self._stats['total_frames'],
            'finger_updates': self._stats['finger_updates'].copy(),
            'total_updates': total_updates,
            'elapsed': elapsed,
            'frame_rate': self._stats['total_frames'] / elapsed if elapsed > 0 else 0,
            'update_rate': total_updates / elapsed if elapsed > 0 else 0
        }


# ============================================================================
# PyGame可视化器
# ============================================================================

class TactileVisualizer:
    """
    @brief 高性能触觉传感器可视化器
    @details 使用PyGame实现60Hz刷新率
    """
    
    def __init__(self, receiver: TactileCANReceiver):
        """
        @brief 初始化可视化器
        @param receiver CAN数据接收器
        """
        self.receiver = receiver
        
        # 延迟导入pygame
        try:
            import pygame
            self.pygame = pygame
        except ImportError:
            print("错误: 未安装pygame")
            print("请运行: pip install pygame")
            sys.exit(1)
        
        self.pygame.init()
        self.pygame.display.set_caption(f"触觉传感器可视化 - {'左手' if receiver.hand == 0 else '右手'}")
        
        # 创建窗口
        self.screen = self.pygame.display.set_mode(
            (VisualConfig.WINDOW_WIDTH, VisualConfig.WINDOW_HEIGHT)
        )
        self.clock = self.pygame.time.Clock()
        
        # 字体
        self.pygame.font.init()
        try:
            # 尝试使用支持中文的字体
            font_names = ['Microsoft YaHei', 'SimHei', 'STHeiti', 'Arial Unicode MS', 'Arial']
            self.font = None
            for font_name in font_names:
                try:
                    self.font = self.pygame.font.SysFont(font_name, 20)
                    self.font_small = self.pygame.font.SysFont(font_name, 14)
                    self.font_large = self.pygame.font.SysFont(font_name, 28)
                    break
                except:
                    continue
            if self.font is None:
                self.font = self.pygame.font.Font(None, 20)
                self.font_small = self.pygame.font.Font(None, 14)
                self.font_large = self.pygame.font.Font(None, 28)
        except:
            self.font = self.pygame.font.Font(None, 20)
            self.font_small = self.pygame.font.Font(None, 14)
            self.font_large = self.pygame.font.Font(None, 28)
        
        # 生成颜色映射表 (256色)
        self._generate_colormap()
        
        # 计算布局
        self._calculate_layout()
        
        # 预渲染表面(用于双缓冲优化)
        # 旋转后：宽度=6(原行数)，高度=12(原列数)
        self._finger_surfaces = [
            self.pygame.Surface((
                CANConfig.MATRIX_ROWS * (VisualConfig.CELL_SIZE + VisualConfig.CELL_MARGIN),
                CANConfig.MATRIX_COLS * (VisualConfig.CELL_SIZE + VisualConfig.CELL_MARGIN)
            ))
            for _ in range(CANConfig.NUM_FINGERS)
        ]
        
        # FPS统计
        self._fps_history = deque(maxlen=60)
        self._last_update_counts = [0] * 5
        
        self._running = True
    
    def _generate_colormap(self):
        """
        @brief 生成256级颜色映射表
        @details 从深蓝(冷)到红色(热)的渐变
        """
        self.colormap = []
        
        # 定义关键颜色点
        key_colors = [
            (0,   (20, 20, 50)),      # 深蓝灰
            (50,  (30, 60, 150)),     # 深蓝
            (100, (50, 150, 200)),    # 青蓝
            (150, (100, 200, 100)),   # 绿色
            (200, (255, 255, 50)),    # 黄色
            (230, (255, 150, 30)),    # 橙色
            (255, (255, 50, 30)),     # 红色
        ]
        
        for i in range(256):
            # 找到i所在的区间
            color = None
            for j in range(len(key_colors) - 1):
                v1, c1 = key_colors[j]
                v2, c2 = key_colors[j + 1]
                if v1 <= i <= v2:
                    # 线性插值
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
        @brief 计算热力图布局位置
        @details 矩阵顺时针旋转90度后，原6行×12列变为12行×6列
                 显示宽度=6列，显示高度=12行
        """
        # 旋转后的热力图尺寸 (原6×12 -> 旋转后显示为12行×6列)
        # 宽度对应6列，高度对应12行
        heatmap_width = CANConfig.MATRIX_ROWS * (VisualConfig.CELL_SIZE + VisualConfig.CELL_MARGIN)
        heatmap_height = CANConfig.MATRIX_COLS * (VisualConfig.CELL_SIZE + VisualConfig.CELL_MARGIN)
        
        # 计算5个热力图的总宽度
        total_width = 5 * heatmap_width + 4 * VisualConfig.FINGER_SPACING
        
        # 起始X位置(居中)
        start_x = (VisualConfig.WINDOW_WIDTH - total_width) // 2
        
        # Y位置(留出顶部空间显示标题和统计)
        start_y = 120
        
        # 计算每个手指的位置
        self.finger_positions = []
        for i in range(5):
            x = start_x + i * (heatmap_width + VisualConfig.FINGER_SPACING)
            self.finger_positions.append((x, start_y))
        
        self.heatmap_width = heatmap_width
        self.heatmap_height = heatmap_height
    
    def _draw_heatmap(self, surface, matrix: np.ndarray):
        """
        @brief 在指定surface上绘制热力图
        @param surface pygame Surface
        @param matrix 6×12数据矩阵
        @details 将矩阵顺时针旋转90度后再左右镜像
                 原矩阵6行×12列 -> 旋转后12行×6列 -> 左右镜像
        """
        surface.fill(VisualConfig.BG_COLOR)
        
        # 顺时针旋转90度: np.rot90(matrix, k=-1) 或 k=3
        rotated = np.rot90(matrix, k=-1)
        # 左右镜像
        rotated = np.fliplr(rotated)
        
        cell_size = VisualConfig.CELL_SIZE
        margin = VisualConfig.CELL_MARGIN
        
        # 旋转后的矩阵尺寸: 12行 × 6列
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
        @brief 绘制手指标签和数值
        """
        fingers_data = self.receiver.get_all_fingers_data()
        
        for i, (pos, data) in enumerate(zip(self.finger_positions, fingers_data)):
            x, y = pos
            
            # 手指名称
            name = CANConfig.FINGER_NAMES[i]
            name_en = CANConfig.FINGER_NAMES_EN[i]
            
            # 标题
            text = self.font.render(f"{name} ({name_en})", True, VisualConfig.TEXT_COLOR)
            text_rect = text.get_rect(centerx=x + self.heatmap_width // 2, bottom=y - 5)
            self.screen.blit(text, text_rect)
            
            # 底部统计信息
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
    
    def _draw_colorbar(self):
        """
        @brief 绘制颜色条
        """
        bar_width = 25
        bar_height = self.heatmap_height  # 与热力图等高
        x = VisualConfig.WINDOW_WIDTH - 80
        y = self.finger_positions[0][1]   # 与热力图顶部对齐
        
        # 绘制颜色条
        for i in range(bar_height):
            value = int(255 * (bar_height - 1 - i) / (bar_height - 1))
            color = self.colormap[value]
            self.pygame.draw.line(self.screen, color, (x, y + i), (x + bar_width, y + i))
        
        # 边框
        self.pygame.draw.rect(self.screen, VisualConfig.BORDER_COLOR, 
                             (x - 1, y - 1, bar_width + 2, bar_height + 2), 1)
        
        # 标签
        labels = ['255', '128', '0']
        positions = [y, y + bar_height // 2, y + bar_height]
        
        for label, pos in zip(labels, positions):
            text = self.font_small.render(label, True, VisualConfig.TEXT_COLOR)
            self.screen.blit(text, (x + bar_width + 5, pos - 7))
    
    def _draw_stats(self):
        """
        @brief 绘制统计信息
        """
        stats = self.receiver.get_stats()
        
        # 计算实际FPS
        current_fps = self.clock.get_fps()
        self._fps_history.append(current_fps)
        avg_fps = sum(self._fps_history) / len(self._fps_history) if self._fps_history else 0
        
        # 计算数据更新率
        data_rate = stats['update_rate'] / 5 if stats['update_rate'] > 0 else 0  # 每指更新率
        
        hand_name = "左手" if self.receiver.hand == 0 else "右手"
        
        # 标题
        title = f"五指触觉传感器实时可视化 - {hand_name}"
        title_surface = self.font_large.render(title, True, VisualConfig.TEXT_COLOR)
        title_rect = title_surface.get_rect(centerx=VisualConfig.WINDOW_WIDTH // 2, y=15)
        self.screen.blit(title_surface, title_rect)
        
        # 统计行
        stats_text = (
            f"显示帧率: {avg_fps:.1f} FPS | "
            f"数据更新率: {data_rate:.1f} Hz/指 | "
            f"CAN帧: {stats['total_frames']} | "
            f"运行时间: {stats['elapsed']:.1f}s"
        )
        stats_surface = self.font.render(stats_text, True, VisualConfig.TEXT_COLOR)
        stats_rect = stats_surface.get_rect(centerx=VisualConfig.WINDOW_WIDTH // 2, y=55)
        self.screen.blit(stats_surface, stats_rect)
        
        # 底部说明
        help_text = "按 Q 或 ESC 退出 | 按 R 重置统计"
        help_surface = self.font_small.render(help_text, True, (150, 150, 150))
        help_rect = help_surface.get_rect(centerx=VisualConfig.WINDOW_WIDTH // 2, 
                                          y=VisualConfig.WINDOW_HEIGHT - 30)
        self.screen.blit(help_surface, help_rect)
    
    def update(self):
        """
        @brief 更新显示(每帧调用)
        @return 是否继续运行
        """
        # 处理事件
        for event in self.pygame.event.get():
            if event.type == self.pygame.QUIT:
                self._running = False
            elif event.type == self.pygame.KEYDOWN:
                if event.key in (self.pygame.K_q, self.pygame.K_ESCAPE):
                    self._running = False
                elif event.key == self.pygame.K_r:
                    # 重置统计
                    self.receiver._stats['start_time'] = time.time()
                    self.receiver._stats['total_frames'] = 0
                    self.receiver._stats['finger_updates'] = [0] * 5
        
        if not self._running:
            return False
        
        # 清屏
        self.screen.fill(VisualConfig.BG_COLOR)
        
        # 获取所有矩阵数据(高性能批量获取)
        matrices = self.receiver.get_all_matrices()
        
        # 更新每个手指的热力图
        for i, (surface, matrix, pos) in enumerate(zip(
            self._finger_surfaces, matrices, self.finger_positions
        )):
            self._draw_heatmap(surface, matrix)
            self.screen.blit(surface, pos)
            
            # 绘制边框
            self.pygame.draw.rect(
                self.screen, VisualConfig.BORDER_COLOR,
                (pos[0] - 1, pos[1] - 1, self.heatmap_width + 2, self.heatmap_height + 2),
                1
            )
        
        # 绘制标签
        self._draw_finger_labels()
        
        # 绘制颜色条
        self._draw_colorbar()
        
        # 绘制统计信息
        self._draw_stats()
        
        # 更新显示
        self.pygame.display.flip()
        
        # 控制帧率
        self.clock.tick(VisualConfig.FPS)
        
        return True
    
    def run(self):
        """
        @brief 运行主循环
        """
        print(f"启动可视化 (目标帧率: {VisualConfig.FPS} FPS)")
        print("按 Q 或 ESC 退出...")
        
        while self.update():
            pass
        
        self.close()
    
    def close(self):
        """
        @brief 关闭可视化器
        """
        self.pygame.quit()


# ============================================================================
# 模拟数据生成器(用于测试)
# ============================================================================

class SimulatedReceiver:
    """
    @brief 模拟数据接收器(用于无CAN硬件时测试)
    """
    
    def __init__(self, hand: int = 0):
        self.hand = hand
        self._lock = threading.Lock()
        self._running = False
        self._thread = None
        
        self._fingers = [FingerData(finger_id=i) for i in range(5)]
        self._stats = {
            'total_frames': 0,
            'finger_updates': [0] * 5,
            'start_time': 0.0
        }
        
        self._time_offset = 0
    
    def _simulate_loop(self):
        """模拟数据生成循环"""
        while self._running:
            with self._lock:
                self._time_offset += 0.016  # ~60Hz
                
                for i, finger in enumerate(self._fingers):
                    # 生成模拟数据(波浪效果)
                    for row in range(CANConfig.MATRIX_ROWS):
                        for col in range(CANConfig.MATRIX_COLS):
                            # 使用正弦波生成动态数据
                            phase = self._time_offset * 2 + i * 0.5
                            wave1 = np.sin(col * 0.5 + phase) * 0.5 + 0.5
                            wave2 = np.cos(row * 0.8 + phase * 0.7) * 0.5 + 0.5
                            
                            # 添加中心高亮效果
                            center_col, center_row = 5.5, 2.5
                            dist = np.sqrt((col - center_col)**2 + (row - center_row)**2)
                            center_weight = max(0, 1 - dist / 6) * np.sin(phase) * 0.5 + 0.5
                            
                            value = int((wave1 * wave2 * 0.5 + center_weight * 0.5) * 200 + 30)
                            finger.matrix[row, col] = min(255, max(0, value))
                    
                    finger.valid = True
                    finger.timestamp = time.time()
                    finger.update_count += 1
                    self._stats['finger_updates'][i] += 1
                
                self._stats['total_frames'] += 45  # 模拟45帧/周期
            
            time.sleep(0.016)  # ~60Hz
    
    def connect(self) -> bool:
        print("使用模拟数据模式 (无CAN硬件)")
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
    
    def get_finger_data(self, finger_id: int) -> FingerData:
        with self._lock:
            src = self._fingers[finger_id]
            return FingerData(
                finger_id=src.finger_id,
                matrix=src.matrix.copy(),
                valid=src.valid,
                timestamp=src.timestamp,
                update_count=src.update_count
            )
    
    def get_all_fingers_data(self) -> List[FingerData]:
        return [self.get_finger_data(i) for i in range(5)]
    
    def get_all_matrices(self) -> List[np.ndarray]:
        with self._lock:
            return [f.matrix.copy() for f in self._fingers]
    
    def get_stats(self) -> dict:
        elapsed = time.time() - self._stats['start_time'] if self._stats['start_time'] > 0 else 0
        total_updates = sum(self._stats['finger_updates'])
        return {
            'total_frames': self._stats['total_frames'],
            'finger_updates': self._stats['finger_updates'].copy(),
            'total_updates': total_updates,
            'elapsed': elapsed,
            'frame_rate': self._stats['total_frames'] / elapsed if elapsed > 0 else 0,
            'update_rate': total_updates / elapsed if elapsed > 0 else 0
        }


# ============================================================================
# 主程序
# ============================================================================

def main():
    """
    @brief 主函数
    """
    import argparse
    
    parser = argparse.ArgumentParser(description='五指触觉传感器可视化')
    parser.add_argument('--hand', type=int, default=0, choices=[0, 1],
                        help='选择手: 0=左手, 1=右手 (默认: 0)')
    parser.add_argument('--channel', type=str, default=None,
                        help='CAN通道 (默认: PCAN_USBBUS1)')
    parser.add_argument('--interface', type=str, default=None,
                        help='CAN接口 (默认: pcan)')
    parser.add_argument('--simulate', action='store_true',
                        help='使用模拟数据(无需CAN硬件)')
    parser.add_argument('--fps', type=int, default=60,
                        help='目标帧率 (默认: 60)')
    
    args = parser.parse_args()
    
    # 更新帧率配置
    VisualConfig.FPS = args.fps
    
    hand_name = "左手" if args.hand == 0 else "右手"
    print("=" * 60)
    print(f" 五指触觉传感器可视化程序 - {hand_name}")
    print("=" * 60)
    
    # 创建接收器
    if args.simulate:
        receiver = SimulatedReceiver(hand=args.hand)
    else:
        receiver = TactileCANReceiver(
            channel=args.channel,
            interface=args.interface,
            hand=args.hand
        )
    
    try:
        # 连接
        if not receiver.connect():
            print("连接失败，切换到模拟模式...")
            receiver = SimulatedReceiver(hand=args.hand)
            receiver.connect()
        
        # 开始接收
        receiver.start()
        
        # 创建并运行可视化器
        visualizer = TactileVisualizer(receiver)
        visualizer.run()
        
    except KeyboardInterrupt:
        print("\n用户中断")
    finally:
        receiver.disconnect()
        
        # 打印最终统计
        stats = receiver.get_stats()
        print("\n" + "=" * 60)
        print(" 最终统计")
        print("=" * 60)
        print(f"  总CAN帧数: {stats['total_frames']}")
        print(f"  每指更新次数: {stats['finger_updates']}")
        print(f"  总更新次数: {stats['total_updates']}")
        print(f"  运行时间: {stats['elapsed']:.2f}s")
        print(f"  平均帧率: {stats['frame_rate']:.1f} frames/s")
        print(f"  数据更新率: {stats['update_rate']:.1f} updates/s")
        print("=" * 60)


if __name__ == "__main__":
    main()
