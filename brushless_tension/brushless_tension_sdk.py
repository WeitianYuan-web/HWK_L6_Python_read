#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
@file    brushless_tension_sdk.py
@brief   无刷拉力控制器 CAN 通信 SDK

本 SDK 实现了题述的无刷拉力控制器 CAN 协议：
- 每个拉力模块的 CAN ID 为 0x2n（高 4 位固定为 0x2，低 4 位为模块序号 n），共 5 个模块
- 上位机发送的控制帧（控制 5 个拉力模块）：
    - 拉力帧：长度 5 字节，5 个字节分别为 5 个模块的拉力设定值（0~255）
    - 位置帧：长度 5 字节，5 个字节分别为 5 个模块的位置设定值（0~255）
    - 本实现中：
        - 拉力命令 CAN ID = 0x200
        - 位置命令 CAN ID = 0x201
- 每个模块的反馈帧：
    - CAN ID = 0x2n，对应模块 n（0~4）
- 控制器反馈帧：
    - 帧长度 2 字节
    - 第 0 字节为反馈类型（0x01=位置反馈，0x02=拉力反馈）
    - 第 1 字节为数据（8bit 原始值，位置代表多圈位置 rad 的量化值，拉力代表 Vq 的量化值，用户可按需要转换）

依赖库：
    pip install python-can

@code
from brushless_tension_sdk import BrushlessTensionSDK

sdk = BrushlessTensionSDK(channel='PCAN_USBBUS1', bustype='pcan', bitrate=1000000)

if sdk.connect():
    sdk.start_receive()

    # 发送 5 模块拉力命令（原始 0~255）
    sdk.send_tension_frame([10, 20, 30, 40, 50])

    # 读取模块 0 的反馈
    state = sdk.get_module_state(0)
    print("模块0 位置(raw):", state.position_raw, "拉力(raw):", state.tension_raw)

    sdk.disconnect()
@endcode
"""

import threading
from dataclasses import dataclass
from enum import IntEnum
from typing import Dict, List, Optional, Callable

import can


__version__ = "0.1.0"
__author__ = "Your Name"
__all__ = [
    "BrushlessTensionSDK",
    "FeedbackType",
    "CommandType",
    "TensionModuleState",
]


class CommandType(IntEnum):
    """
    @brief 上位机发送命令帧类型
    """

    TENSION = 0x01  # 拉力命令
    POSITION = 0x02  # 位置命令


class FeedbackType(IntEnum):
    """
    @brief 控制器反馈数据类型
    """

    POSITION = 0x01  # 位置反馈，原始 0~255，可映射到多圈位置 rad
    TENSION = 0x02  # 拉力反馈，原始 0~255，可映射到 Vq


@dataclass
class TensionModuleState:
    """
    @brief 单个拉力模块的当前状态

    仅保存最近一次收到的 8bit 原始值，用户可通过转换函数映射成物理量。
    """

    module_index: int
    can_id: int
    position_raw: int = 0
    tension_raw: int = 0

    def position_to_physical(self, max_position: float = 35.0) -> float:
        """
        @brief 将位置原始值映射为物理位置

        默认按照 0~255 对应 0~35（题述），单位由用户自行解释（例如 mm 或 rad）。

        @param max_position 位置上限，对应原始值 255
        @return 映射后的物理位置值
        """
        return (self.position_raw / 255.0) * max_position

    def tension_to_physical(self, i_min_ratio: float = 0.04, i_max_ratio: float = 0.12) -> float:
        """
        @brief 将拉力原始值映射为电流占比或其他物理量

        题述为 0~255 对应最大电流的 0.04~0.12，本函数假设线性映射：
        ratio = i_min_ratio + (raw/255) * (i_max_ratio - i_min_ratio)

        @param i_min_ratio 最小比例
        @param i_max_ratio 最大比例
        @return 映射后的比例值（例如额定电流的倍数）
        """
        return i_min_ratio + (self.tension_raw / 255.0) * (i_max_ratio - i_min_ratio)


class BrushlessTensionSDK:
    """
    @brief 无刷拉力控制器 CAN 通信 SDK 主类

    提供以下功能：
    - 发送 5 模块拉力/位置控制命令帧（每模块 8bit）
    - 接收并解析各模块位置/拉力反馈帧
    - 维护每个模块最近一次的反馈状态
    """

    # 协议常量
    MODULE_COUNT = 5  ##< 模块数量
    FB_ID_BASE = 0x20  ##< 反馈基地址，每个模块反馈 ID = 0x2n (n=0~4)

    # 这里假设上位机发送控制帧使用固定 ID
    CTRL_ID_TENSION = 0x200  ##< 拉力命令 CAN ID
    CTRL_ID_POSITION = 0x201  ##< 位置命令 CAN ID

    CMD_FRAME_DLC = 5  ##< 控制帧长度（5 个模块）
    FB_FRAME_DLC = 2  ##< 反馈帧长度

    def __init__(
        self,
        channel: str = "can0",
        bustype: str = "socketcan",
        bitrate: int = 1_000_000,
        receive_timeout: float = 0.01,
    ):
        """
        @brief 初始化 SDK

        @param channel CAN 通道名称
        @param bustype CAN 适配器类型（'socketcan', 'pcan', 'vector', 'kvaser' 等）
        @param bitrate 波特率，默认 1Mbps
        @param receive_timeout 接收超时时间（秒）
        """
        self.channel = channel
        self.bustype = bustype
        self.bitrate = bitrate
        self.receive_timeout = receive_timeout

        self.bus: Optional[can.Bus] = None

        # 模块状态表：key 为模块序号 n（0~4），value 为 TensionModuleState
        self._modules: Dict[int, TensionModuleState] = {}

        # 接收线程
        self._rx_thread: Optional[threading.Thread] = None
        self._running = False
        self._lock = threading.Lock()

        # 回调
        self._feedback_callbacks: List[Callable[[TensionModuleState, FeedbackType], None]] = []
        self._error_callback: Optional[Callable[[Exception], None]] = None

    # ========= 连接管理 =========

    def connect(self) -> bool:
        """
        @brief 连接到 CAN 总线

        @return True=成功, False=失败
        """
        try:
            self.bus = can.interface.Bus(
                channel=self.channel,
                bustype=self.bustype,
                bitrate=self.bitrate,
            )
            print(f"✓ CAN 总线连接成功: {self.channel} ({self.bustype}) @ {self.bitrate}bps")
            return True
        except Exception as e:
            print(f"✗ CAN 总线连接失败: {e}")
            if self._error_callback:
                self._error_callback(e)
            return False

    def disconnect(self):
        """
        @brief 断开 CAN 总线连接
        """
        self.stop_receive()
        if self.bus is not None:
            self.bus.shutdown()
            self.bus = None
            print("✓ CAN 总线已断开")

    def is_connected(self) -> bool:
        """
        @brief 判断 CAN 是否已连接

        @return True=已连接, False=未连接
        """
        return self.bus is not None

    # ========= 命令发送 =========

    def _build_frame_data(self, values: List[int]) -> bytes:
        """
        @brief 将 5 个模块的设定值打包成 5 字节数据

        @param values 5 个模块的原始值列表 (0~255)，长度不足会补 0，多余会截断
        @return 5 字节的 CAN 数据
        """
        vals = list(values[: self.MODULE_COUNT])
        if len(vals) < self.MODULE_COUNT:
            vals += [0] * (self.MODULE_COUNT - len(vals))
        vals = [max(0, min(255, int(v))) for v in vals]
        return bytes(vals)

    def send_tension_frame(self, tensions: List[int]) -> bool:
        """
        @brief 发送拉力命令帧（同时控制 5 个模块）

        @param tensions 5 个模块的拉力原始值列表 (0~255)
        @return True=发送成功, False=发送失败
        """
        if not self.is_connected():
            print("✗ CAN 未连接")
            return False

        data = self._build_frame_data(tensions)
        try:
            msg = can.Message(
                arbitration_id=self.CTRL_ID_TENSION,
                data=data,
                is_extended_id=False,
            )
            self.bus.send(msg)
            return True
        except Exception as e:
            print(f"✗ 发送拉力命令失败 (ID=0x{self.CTRL_ID_TENSION:03X}): {e}")
            if self._error_callback:
                self._error_callback(e)
            return False

    def send_position_frame(self, positions: List[int]) -> bool:
        """
        @brief 发送位置命令帧（同时控制 5 个模块）

        @param positions 5 个模块的位置原始值列表 (0~255)，可按 0~35 映射物理位置
        @return True=发送成功, False=发送失败
        """
        if not self.is_connected():
            print("✗ CAN 未连接")
            return False

        data = self._build_frame_data(positions)
        try:
            msg = can.Message(
                arbitration_id=self.CTRL_ID_POSITION,
                data=data,
                is_extended_id=False,
            )
            self.bus.send(msg)
            return True
        except Exception as e:
            print(f"✗ 发送位置命令失败 (ID=0x{self.CTRL_ID_POSITION:03X}): {e}")
            if self._error_callback:
                self._error_callback(e)
            return False

    # ========= 接收与解析 =========

    def _ensure_module_state(self, module_index: int) -> TensionModuleState:
        """
        @brief 确保模块状态对象存在

        @param module_index 模块序号
        @return 对应的 TensionModuleState 对象
        """
        if module_index not in self._modules:
            can_id = self.FB_ID_BASE + module_index
            self._modules[module_index] = TensionModuleState(
                module_index=module_index,
                can_id=can_id,
            )
        return self._modules[module_index]

    def _process_feedback_message(self, msg: can.Message):
        """
        @brief 解析反馈帧

        仅处理 ID 匹配 0x2n 且 DLC=2 的帧：
        - data[0] 为反馈类型（0x01=位置, 0x02=拉力）
        - data[1] 为 8bit 原始数据
        """
        if msg.dlc != self.FB_FRAME_DLC:
            return
        can_id = msg.arbitration_id

        # 检查是否在 5 个模块的反馈 ID 范围内
        if not (self.FB_ID_BASE <= can_id < self.FB_ID_BASE + self.MODULE_COUNT):
            return

        module_index = can_id - self.FB_ID_BASE
        data = bytes(msg.data)
        fb_type_val = data[0]
        raw_val = data[1]

        try:
            fb_type = FeedbackType(fb_type_val)
        except ValueError:
            # 未知类型，丢弃
            return

        with self._lock:
            state = self._ensure_module_state(module_index)
            if fb_type == FeedbackType.POSITION:
                state.position_raw = raw_val
            elif fb_type == FeedbackType.TENSION:
                state.tension_raw = raw_val

        # 回调在锁外执行，以避免阻塞
        for cb in self._feedback_callbacks:
            try:
                cb(state, fb_type)
            except Exception as e:
                print(f"✗ 反馈回调执行错误: {e}")

    def _receive_loop(self):
        """
        @brief 接收线程主循环
        """
        print("✓ 无刷拉力控制器接收线程已启动")
        while self._running and self.bus is not None:
            try:
                msg = self.bus.recv(timeout=self.receive_timeout)
                if msg is not None:
                    self._process_feedback_message(msg)
            except Exception as e:
                if self._running and self._error_callback:
                    self._error_callback(e)
        print("✓ 无刷拉力控制器接收线程已停止")

    def start_receive(self) -> bool:
        """
        @brief 启动接收线程

        @return True=启动成功, False=失败
        """
        if not self.is_connected():
            print("✗ CAN 未连接，无法启动接收线程")
            return False
        if self._running:
            return True

        self._running = True
        self._rx_thread = threading.Thread(target=self._receive_loop, daemon=True)
        self._rx_thread.start()
        return True

    def stop_receive(self):
        """
        @brief 停止接收线程
        """
        if self._running:
            self._running = False
            if self._rx_thread is not None:
                self._rx_thread.join(timeout=1.0)
                self._rx_thread = None

    # ========= 状态访问与回调 =========

    def get_module_state(self, module_index: int) -> TensionModuleState:
        """
        @brief 获取指定模块最近一次反馈状态

        @param module_index 模块序号 n（0~4）
        @return 对应模块的 TensionModuleState 对象（若无反馈则为默认值）
        """
        with self._lock:
            return self._ensure_module_state(module_index)

    def add_feedback_callback(self, callback: Callable[[TensionModuleState, FeedbackType], None]):
        """
        @brief 添加反馈回调函数

        @param callback 回调函数，原型为 callback(state: TensionModuleState, fb_type: FeedbackType)
        """
        if callback not in self._feedback_callbacks:
            self._feedback_callbacks.append(callback)

    def remove_feedback_callback(self, callback: Callable[[TensionModuleState, FeedbackType], None]):
        """
        @brief 移除反馈回调函数

        @param callback 要移除的回调函数
        """
        if callback in self._feedback_callbacks:
            self._feedback_callbacks.remove(callback)

    def set_error_callback(self, callback: Optional[Callable[[Exception], None]]):
        """
        @brief 设置错误回调函数

        @param callback 错误回调函数，参数为 Exception
        """
        self._error_callback = callback


if __name__ == "__main__":
    """
    @brief 简单自测入口
    """
    print(f"无刷拉力控制器 CAN SDK v{__version__}")
    print("示例：from brushless_tension_sdk import BrushlessTensionSDK")

