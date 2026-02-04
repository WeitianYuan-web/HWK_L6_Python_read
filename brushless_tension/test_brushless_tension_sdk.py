#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
@file    test_brushless_tension_sdk.py
@brief   无刷拉力控制器 SDK 测试脚本

@details
以 60Hz 的频率循环：
- 向 5 个拉力模块发送拉力命令帧（简单正弦波/锯齿波）
- 读取 5 个模块的反馈数据并打印

依赖：
    pip install python-can

用法示例：
    python test_brushless_tension_sdk.py
    python test_brushless_tension_sdk.py --channel PCAN_USBBUS1 --bustype pcan --duration 10
"""

import argparse
import math
import signal
import sys
import time

from brushless_tension_sdk import BrushlessTensionSDK, FeedbackType


_running = True


def _signal_handler(sig, frame):
    """
    @brief 信号处理函数，用于优雅退出
    """
    global _running
    print("\n收到退出信号，准备退出 ...")
    _running = False


def parse_args() -> argparse.Namespace:
    """
    @brief 解析命令行参数
    """
    parser = argparse.ArgumentParser(description="无刷拉力控制器 SDK 测试脚本 (60Hz)")
    parser.add_argument(
        "--channel",
        type=str,
        default="PCAN_USBBUS1",
        help="CAN 通道名称，示例：can0 / PCAN_USBBUS1 (默认: can0)",
    )
    parser.add_argument(
        "--bustype",
        type=str,
        default="pcan",
        help="CAN 适配器类型，示例：socketcan / pcan / vector / kvaser (默认: socketcan)",
    )
    parser.add_argument(
        "--bitrate",
        type=int,
        default=1_000_000,
        help="CAN 波特率，默认 1000000 (1Mbps)",
    )
    parser.add_argument(
        "--duration",
        type=float,
        default=0.0,
        help="测试持续时间（秒），<=0 表示一直运行直到 Ctrl+C (默认: 0)",
    )
    return parser.parse_args()


def main() -> int:
    """
    @brief 主函数：以 60Hz 频率发送命令并打印反馈
    """
    args = parse_args()

    # 注册信号处理（Ctrl+C 退出）
    signal.signal(signal.SIGINT, _signal_handler)
    if hasattr(signal, "SIGTERM"):
        signal.signal(signal.SIGTERM, _signal_handler)

    sdk = BrushlessTensionSDK(
        channel=args.channel,
        bustype=args.bustype,
        bitrate=args.bitrate,
        receive_timeout=0.005,
    )

    if not sdk.connect():
        print("连接 CAN 失败，退出。")
        return 1

    if not sdk.start_receive():
        print("启动接收线程失败，退出。")
        sdk.disconnect()
        return 1

    print("开始测试 5 个模块，频率 60Hz")
    print("按 Ctrl+C 结束测试。\n")

    target_hz = 60.0
    dt = 1.0 / target_hz
    t0 = time.perf_counter()
    last_print_time = 0.0

    try:
        while _running:
            now = time.perf_counter()
            elapsed = now - t0

            # 若设置了持续时间且已超时，则退出
            if args.duration > 0.0 and elapsed >= args.duration:
                print("\n达到设定持续时间，结束测试。")
                break

            # 生成一个简单的 5 模块拉力正弦/锯齿波（0~255）
            # 模块 0: 正弦
            # 模块 1: 相位偏移的正弦
            # 模块 2: 锯齿波
            # 模块 3: 常值 128
            # 模块 4: 递增 / 递减方波
            phase = 2.0 * math.pi * 0.1 * elapsed  # 0.1Hz 正弦
            ch0 = int((math.sin(phase) * 0.5 + 0.5) * 255)
            ch1 = int((math.sin(phase + math.pi / 2) * 0.5 + 0.5) * 255)
            saw = (elapsed * 0.2) % 1.0  # 0.2Hz 锯齿
            ch2 = int(saw * 255)
            ch3 = 128
            ch4 = 255 if int(elapsed * 0.5) % 2 == 0 else 0  # 0.5Hz 方波
            tensions = [ch0, ch1, ch2, ch3, ch4]

            # 发送拉力命令帧（5 模块）
            ok = sdk.send_tension_frame(tensions)
            if not ok:
                print("发送拉力命令失败。")

            # 读取 5 个模块反馈
            states = [sdk.get_module_state(i) for i in range(5)]

            # 控制打印频率（此处和控制频率一致，也可以设置为更低）
            if now - last_print_time >= 0.5:
                last_print_time = now

                fb_strs = []
                for idx, st in enumerate(states):
                    pos_raw = st.position_raw
                    ten_raw = st.tension_raw
                    pos_phys = st.position_to_physical()
                    ten_ratio = st.tension_to_physical()
                    fb_strs.append(
                        f"M{idx}:pos_raw={pos_raw:3d},pos={pos_phys:5.2f},"
                        f"ten_raw={ten_raw:3d},ten={ten_ratio:5.3f}"
                    )

                print(f"t={elapsed:6.3f}s | Cmd={tensions} | " + " | ".join(fb_strs))

            # 精准一点的 60Hz 控制：睡眠到下一个周期
            next_time = now + dt
            sleep_time = next_time - time.perf_counter()
            if sleep_time > 0:
                time.sleep(sleep_time)

    finally:
        sdk.stop_receive()
        sdk.disconnect()

    print("测试结束。")
    return 0


if __name__ == "__main__":
    sys.exit(main())

