#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
控制器到机械手映射程序

通过USB串口读取控制器数据，映射到机械手关节位置并通过CAN总线发送

@file    controller_to_robot_hand.py
@brief   USB控制器到机械手的数据映射和控制
@date    2025-01-07
"""

import serial
import struct
import time
import sys
from typing import List, Tuple, Optional
from can_robot_hand_sdk import create_sdk, CANProtocol


# ==================== 串口配置 ====================
SERIAL_PORT = 'COM61'              # Windows: 'COM3', Linux: '/dev/ttyUSB0'
SERIAL_BAUDRATE = 115200          # 串口波特率
SERIAL_TIMEOUT = 0.1              # 串口读取超时（秒）

# ==================== CAN配置 ====================
CAN_CHANNEL = 'PCAN_USBBUS1'      # CAN通道
CAN_BUSTYPE = 'pcan'              # CAN总线类型
CAN_BITRATE = 1000000             # CAN波特率

# ==================== 数据包配置 ====================
PACKET_SIZE = 28                  # 数据包大小：6*4字节float + 4字节帧尾
FRAME_TAIL = b'\x00\x00\x80\x7f'  # 帧尾标识

# ==================== 关节角度范围配置 ====================
# 每个关节的角度范围 [最小角度, 最大角度]
# 注意：如果最小角度 > 最大角度，则表示跨越0度的环绕情况
JOINT_ANGLE_RANGES = [
    (177.0, 236.0),   # 关节0
    (84.0, 190.0),    # 关节1
    (183.0, 264.0),   # 关节2
    (188.0, 265.0),   # 关节3
    (178.0, 258.0),   # 关节4
    (190.0, 267.0),   # 关节5
]

# 关节反向配置（True=反向映射，False=正向映射）
# 反向映射：角度最小值→位置1023，角度最大值→位置0
# 正向映射：角度最小值→位置0，角度最大值→位置1023
JOINT_REVERSED = [
    False,   # 关节0：正向
    True,    # 关节1：反向
    True,   # 关节2：正向
    True,    # 关节3：反向
    True,    # 关节4：反向
    True,    # 关节5：反向
]

# 输出关节位置范围
JOINT_POS_MIN = 0                 # 关节位置最小值
JOINT_POS_MAX = 1023              # 关节位置最大值（10bit）

# ==================== 控制参数 ====================
SEND_FREQUENCY = 100              # 发送频率（Hz）
SEND_PERIOD = 1.0 / SEND_FREQUENCY  # 发送周期（秒）

# 数据平滑参数（低通滤波）
ENABLE_FILTER = True              # 是否启用滤波
FILTER_ALPHA = 0.3                # 滤波系数（0-1），越小越平滑但响应越慢


# ==================== 角度映射函数 ====================

def normalize_angle(angle: float) -> float:
    """
    @brief 将原始角度限制在[-360°, 720°]范围内
    
    @details 控制器当前输出的原始角度工作空间为[-360°, 720°]。
             本函数只做“裁剪”，不再按360度做周期性折返，
             后续的关节映射逻辑直接在这个连续角度空间内工作。
    
    @param angle 输入角度
    @return 裁剪后的角度（仍在[-360°, 720°]空间内）
    """
    # 限制在物理给定的原始角度范围，避免异常值
    if angle < -360.0:
        angle = -360.0
    elif angle > 720.0:
        angle = 720.0
    return angle


def map_angle_to_position(angle: float, angle_min: float, angle_max: float, reversed: bool = False) -> int:
    """
    @brief 将角度值线性映射到关节位置（0-1023）
    
    @details 在统一的连续角度空间[-360°, 720°]内进行线性映射：
             - 先用 normalize_angle 将输入角度裁剪到[-360°, 720°]
             - 再在 [angle_min, angle_max] 上做线性插值
             - angle_min/angle_max 同样可以取在[-360°, 720°]范围内
             - 如果角度超出[min, max]，则在两端钳位
             反向映射：
             - 正向(reversed=False): angle_min→0, angle_max→1023
             - 反向(reversed=True):  angle_min→1023, angle_max→0
    
    @param angle 当前角度（0-360）
    @param angle_min 角度范围最小值
    @param angle_max 角度范围最大值
    @param reversed 是否反向映射
    @return 映射后的关节位置（0-1023）
    """
    # 将输入角度裁剪到统一工作空间
    angle = normalize_angle(angle)
    # 允许配置的最小/最大角度也在该空间内
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
    
    # 如果需要反向，则反转比例
    if reversed:
        ratio = 1.0 - ratio
    
    # 映射到0-1023
    position = int(ratio * (JOINT_POS_MAX - JOINT_POS_MIN) + JOINT_POS_MIN)
    
    # 确保在有效范围内
    position = max(JOINT_POS_MIN, min(JOINT_POS_MAX, position))
    
    return position


def map_angles_to_positions(angles: List[float]) -> List[int]:
    """
    @brief 将6个角度值映射到6个关节位置
    
    根据JOINT_REVERSED配置应用正向或反向映射
    
    @param angles 6个角度值列表
    @return 6个关节位置列表（0-1023）
    """
    if len(angles) != 6:
        raise ValueError(f"需要6个角度值，实际收到{len(angles)}个")
    
    positions = []
    for i in range(6):
        angle_min, angle_max = JOINT_ANGLE_RANGES[i]
        reversed_flag = JOINT_REVERSED[i]
        position = map_angle_to_position(angles[i], angle_min, angle_max, reversed_flag)
        positions.append(position)
    
    return positions


# ==================== 串口数据读取 ====================

class ControllerReader:
    """
    @brief 控制器数据读取类
    """
    
    def __init__(self, port: str, baudrate: int = SERIAL_BAUDRATE, timeout: float = SERIAL_TIMEOUT):
        """
        @brief 初始化串口连接
        
        @param port 串口端口号
        @param baudrate 波特率
        @param timeout 超时时间
        """
        self.port = port
        self.baudrate = baudrate
        self.timeout = timeout
        self.serial: Optional[serial.Serial] = None
        self.buffer = bytearray()
        
        # 滤波器状态
        self.filtered_angles = [0.0] * 6
        self.is_first_read = True
        
        # 统计信息
        self.packet_count = 0
        self.error_count = 0
        self.last_angles = [0.0] * 6
    
    def connect(self) -> bool:
        """
        @brief 连接串口
        
        @return True=成功, False=失败
        """
        try:
            self.serial = serial.Serial(
                port=self.port,
                baudrate=self.baudrate,
                timeout=self.timeout,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE
            )
            print(f"✓ 串口连接成功: {self.port} @ {self.baudrate}bps")
            return True
        except Exception as e:
            print(f"✗ 串口连接失败: {e}")
            return False
    
    def disconnect(self):
        """
        @brief 断开串口连接
        """
        if self.serial and self.serial.is_open:
            self.serial.close()
            print("✓ 串口已断开")
    
    def _find_frame_start(self) -> int:
        """
        @brief 在缓冲区中查找帧尾标识
        
        @return 帧尾位置，如果未找到返回-1
        """
        return self.buffer.find(FRAME_TAIL)
    
    def read_packet(self) -> Optional[List[float]]:
        """
        @brief 读取一个完整的数据包
        
        @return 6个浮点数列表，如果失败返回None
        """
        if not self.serial or not self.serial.is_open:
            return None
        
        try:
            # 读取可用数据
            if self.serial.in_waiting > 0:
                data = self.serial.read(self.serial.in_waiting)
                self.buffer.extend(data)
            
            # 查找帧尾
            frame_end = self._find_frame_start()
            if frame_end == -1:
                # 未找到完整帧，缓冲区过大则清空
                if len(self.buffer) > PACKET_SIZE * 2:
                    self.buffer.clear()
                return None
            
            # 检查是否有完整的数据包
            if frame_end < PACKET_SIZE - len(FRAME_TAIL):
                # 数据不足，继续等待
                return None
            
            # 提取数据包（从帧尾往前推）
            packet_start = frame_end - (PACKET_SIZE - len(FRAME_TAIL))
            if packet_start < 0:
                # 数据不完整
                self.buffer = self.buffer[frame_end + len(FRAME_TAIL):]
                return None
            
            packet = self.buffer[packet_start:frame_end]
            
            # 移除已处理的数据
            self.buffer = self.buffer[frame_end + len(FRAME_TAIL):]
            
            # 解析6个float
            if len(packet) != PACKET_SIZE - len(FRAME_TAIL):
                self.error_count += 1
                return None
            
            # 解包为6个float（小端序）
            angles = list(struct.unpack('<6f', packet))
            
            # 应用滤波
            if ENABLE_FILTER:
                if self.is_first_read:
                    self.filtered_angles = angles.copy()
                    self.is_first_read = False
                else:
                    for i in range(6):
                        self.filtered_angles[i] = (FILTER_ALPHA * angles[i] + 
                                                   (1 - FILTER_ALPHA) * self.filtered_angles[i])
                angles = self.filtered_angles.copy()
            
            self.packet_count += 1
            self.last_angles = angles
            
            return angles
            
        except Exception as e:
            print(f"✗ 读取数据包错误: {e}")
            self.error_count += 1
            return None
    
    def get_statistics(self) -> dict:
        """
        @brief 获取统计信息
        
        @return 统计信息字典
        """
        return {
            'packet_count': self.packet_count,
            'error_count': self.error_count,
            'buffer_size': len(self.buffer),
            'last_angles': self.last_angles
        }


# ==================== 主程序 ====================

def print_config():
    """
    @brief 打印配置信息
    """
    print("\n" + "="*70)
    print("  USB控制器到机械手映射程序")
    print("="*70)
    print(f"\n【串口配置】")
    print(f"  端口: {SERIAL_PORT}")
    print(f"  波特率: {SERIAL_BAUDRATE}bps")
    print(f"  超时: {SERIAL_TIMEOUT}s")
    
    print(f"\n【CAN配置】")
    print(f"  通道: {CAN_CHANNEL}")
    print(f"  总线类型: {CAN_BUSTYPE}")
    print(f"  波特率: {CAN_BITRATE}bps")
    
    print(f"\n【控制参数】")
    print(f"  发送频率: {SEND_FREQUENCY}Hz")
    print(f"  数据滤波: {'启用' if ENABLE_FILTER else '禁用'}")
    if ENABLE_FILTER:
        print(f"  滤波系数: {FILTER_ALPHA}")
    
    print(f"\n【角度映射配置】")
    for i, (angle_min, angle_max) in enumerate(JOINT_ANGLE_RANGES):
        reversed_flag = JOINT_REVERSED[i]
        reversed_str = "反向" if reversed_flag else "正向"
        
        if angle_min > angle_max:
            # 环绕情况
            if reversed_flag:
                print(f"  关节{i}: {angle_min:.1f}° -> 360° -> 0° -> {angle_max:.1f}° "
                      f"映射到 1023 -> 0 (环绕+反向)")
            else:
                print(f"  关节{i}: {angle_min:.1f}° -> 360° -> 0° -> {angle_max:.1f}° "
                      f"映射到 0 -> 1023 (环绕)")
        else:
            # 正常情况
            if reversed_flag:
                print(f"  关节{i}: {angle_min:.1f}° - {angle_max:.1f}° "
                      f"映射到 1023 -> 0 (反向)")
            else:
                print(f"  关节{i}: {angle_min:.1f}° - {angle_max:.1f}° "
                      f"映射到 0 -> 1023 (正向)")
    print()


def main():
    """
    @brief 主函数
    """
    print_config()
    
    # 创建控制器读取器
    controller = ControllerReader(SERIAL_PORT, SERIAL_BAUDRATE, SERIAL_TIMEOUT)
    
    # 连接串口
    if not controller.connect():
        print("\n请检查：")
        print("1. 控制器是否已连接")
        print("2. 串口号是否正确")
        print("3. 是否有权限访问串口（Linux需要sudo或加入dialout组）")
        return
    
    # 连接CAN总线
    print(f"\n正在连接CAN总线...")
    robot = create_sdk(
        channel=CAN_CHANNEL,
        bustype=CAN_BUSTYPE,
        bitrate=CAN_BITRATE,
        auto_connect=True
    )
    
    if not robot.is_connected():
        print("\n请检查：")
        print("1. CAN适配器是否已连接")
        print("2. CAN通道名称是否正确")
        print("3. CAN驱动是否已安装")
        controller.disconnect()
        return
    
    print("\n✓ 系统初始化完成！")
    print("开始读取控制器数据并控制机械手...")
    print("按 Ctrl+C 退出\n")
    
    try:
        cycle = 0
        last_send_time = time.time()
        last_print_time = time.time()
        
        while True:
            cycle += 1
            current_time = time.time()
            
            # 读取控制器数据
            angles = controller.read_packet()
            
            if angles is not None:
                # 映射到关节位置
                try:
                    positions = map_angles_to_positions(angles)
                    
                    # 发送到机械手（控制发送频率）
                    if (current_time - last_send_time) >= SEND_PERIOD:
                        robot.send_left_control(position=positions)
                        last_send_time = current_time
                    
                    # 每秒打印一次状态
                    if (current_time - last_print_time) >= 1.0:
                        stats = controller.get_statistics()
                        print(f"[状态] 周期:{cycle} 数据包:{stats['packet_count']} "
                              f"错误:{stats['error_count']}")
                        print(f"  角度: [{angles[0]:.1f}, {angles[1]:.1f}, {angles[2]:.1f}, "
                              f"{angles[3]:.1f}, {angles[4]:.1f}, {angles[5]:.1f}]")
                        print(f"  位置: [{positions[0]}, {positions[1]}, {positions[2]}, "
                              f"{positions[3]}, {positions[4]}, {positions[5]}]")
                        
                        # 打印CAN统计
                        can_stats = robot.get_statistics()
                        print(f"  CAN: TX={can_stats['tx_count']} RX={can_stats['rx_count']} "
                              f"Err={can_stats['tx_error_count']}\n")
                        
                        last_print_time = current_time
                
                except Exception as e:
                    print(f"✗ 映射错误: {e}")
            
            # 短暂延时，避免CPU占用过高
            time.sleep(0.001)
    
    except KeyboardInterrupt:
        print("\n\n用户中断，正在退出...")
    
    except Exception as e:
        print(f"\n✗ 程序错误: {e}")
        import traceback
        traceback.print_exc()
    
    finally:
        # 清理资源
        print("\n正在清理资源...")
        controller.disconnect()
        robot.disconnect()
        print("✓ 程序已退出\n")


if __name__ == "__main__":
    main()
