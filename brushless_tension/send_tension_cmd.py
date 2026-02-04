#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
@file    send_tension_cmd.py
@brief   交互式控制台：拉力和位置设定 (0~255)

命令说明：
    t [模块] <值>        发送拉力。例: t 128  /  t 0 100  /  t 10,20,30,40,50
    p [模块] <值>        发送位置。例: p 200  /  p 2 50   /  p 0,50,100,150,200
    s / status           查看 5 个模块当前反馈（若已连接）
    h / help             显示帮助
    q / quit / exit      退出

用法：
    python send_tension_cmd.py
    python send_tension_cmd.py --channel PCAN_USBBUS1 --bustype pcan
"""

import argparse
import sys

from brushless_tension_sdk import BrushlessTensionSDK

MODULE_COUNT = 5


def parse_value_list(s: str) -> list:
    """
    @brief 解析逗号分隔的数值列表，长度须为 1 或 MODULE_COUNT
    """
    parts = [p.strip() for p in s.split(",") if p.strip()]
    if not parts:
        raise ValueError("至少需要 1 个数值")
    vals = []
    for p in parts:
        try:
            v = int(p)
        except ValueError:
            raise ValueError(f"无效数值: {p}")
        if v < 0 or v > 255:
            raise ValueError(f"数值须在 0~255 之间: {v}")
        vals.append(v)
    if len(vals) != 1 and len(vals) != MODULE_COUNT:
        raise ValueError(
            f"须为 1 个数值或 {MODULE_COUNT} 个逗号分隔数值，当前为 {len(vals)} 个"
        )
    return vals


def build_frame_values(val_list: list, module_index: int | None) -> list:
    """
    @brief 根据解析出的数值和可选的模块号生成 5 个模块的设定列表
    """
    if len(val_list) == MODULE_COUNT:
        return list(val_list[:MODULE_COUNT])
    val = val_list[0]
    if module_index is not None:
        arr = [0] * MODULE_COUNT
        if 0 <= module_index < MODULE_COUNT:
            arr[module_index] = val
        return arr
    return [val] * MODULE_COUNT


def print_help():
    """打印交互命令帮助"""
    print()
    print("  命令:")
    print("    t [模块] <值>    发送拉力 (0~255)")
    print("                      例: t 128     → 5 个模块都为 128")
    print("                          t 0 100   → 仅模块 0 为 100")
    print("                          t 10,20,30,40,50")
    print("    p [模块] <值>    发送位置 (0~255)")
    print("                      例: p 200  /  p 2 50  /  p 0,50,100,150,200")
    print("    s / status       查看 5 个模块当前反馈")
    print("    h / help         显示本帮助")
    print("    q / quit / exit  退出")
    print()


def run_interactive(sdk: BrushlessTensionSDK) -> None:
    """
    @brief 交互式命令循环
    """
    print_help()
    while True:
        try:
            line = input("> ").strip()
        except EOFError:
            print()
            break
        if not line:
            continue
        parts = line.split(maxsplit=2)
        cmd = (parts[0] or "").lower()
        rest = (parts[1] + " " + parts[2]).strip() if len(parts) >= 3 else (parts[1] if len(parts) >= 2 else "")

        if cmd in ("q", "quit", "exit"):
            break
        if cmd in ("h", "help"):
            print_help()
            continue
        if cmd in ("s", "status"):
            for i in range(MODULE_COUNT):
                st = sdk.get_module_state(i)
                pos_phys = st.position_to_physical()
                ten_ratio = st.tension_to_physical()
                print(f"  模块{i}: 位置 raw={st.position_raw:3d} ({pos_phys:5.2f})  "
                      f"拉力 raw={st.tension_raw:3d} (比例={ten_ratio:.3f})")
            continue

        if cmd in ("t", "tension"):
            if not rest:
                print("  用法: t [模块] <值>  或  t 值0,值1,值2,值3,值4")
                continue
            # 解析：可能是 "128" 或 "0 100" 或 "10,20,30,40,50"
            toks = rest.split(maxsplit=1)
            module_index = None
            value_str = rest
            if len(toks) == 2:
                try:
                    first = int(toks[0])
                    if 0 <= first <= 255 and ("," not in toks[0]):
                        # 可能是 "0 100" 形式
                        second = toks[1].strip()
                        if second.isdigit() and "," not in toks[1]:
                            module_index = first
                            value_str = second
                except ValueError:
                    pass
            try:
                val_list = parse_value_list(value_str)
                values = build_frame_values(val_list, module_index)
                if sdk.send_tension_frame(values):
                    print(f"  已发送拉力: {values}")
                else:
                    print("  发送拉力失败")
            except ValueError as e:
                print(f"  {e}")
            continue

        if cmd in ("p", "position"):
            if not rest:
                print("  用法: p [模块] <值>  或  p 值0,值1,值2,值3,值4")
                continue
            toks = rest.split(maxsplit=1)
            module_index = None
            value_str = rest
            if len(toks) == 2:
                try:
                    first = int(toks[0])
                    if 0 <= first <= 255 and ("," not in toks[0]):
                        second = toks[1].strip()
                        if second.isdigit() and "," not in toks[1]:
                            module_index = first
                            value_str = second
                except ValueError:
                    pass
            try:
                val_list = parse_value_list(value_str)
                values = build_frame_values(val_list, module_index)
                if sdk.send_position_frame(values):
                    print(f"  已发送位置: {values}")
                else:
                    print("  发送位置失败")
            except ValueError as e:
                print(f"  {e}")
            continue

        print("  未知命令，输入 h 或 help 查看帮助")
    print("已退出。")


def main() -> int:
    parser = argparse.ArgumentParser(
        description="无刷拉力控制器 - 交互式拉力和位置控制"
    )
    parser.add_argument(
        "--channel",
        type=str,
        default="PCAN_USBBUS1",
        help="CAN 通道 (默认: PCAN_USBBUS1)",
    )
    parser.add_argument(
        "--bustype",
        type=str,
        default="pcan",
        help="CAN 适配器类型 (默认: pcan)",
    )
    parser.add_argument(
        "--bitrate",
        type=int,
        default=1_000_000,
        help="波特率 (默认: 1000000)",
    )
    args = parser.parse_args()

    sdk = BrushlessTensionSDK(
        channel=args.channel,
        bustype=args.bustype,
        bitrate=args.bitrate,
    )
    if not sdk.connect():
        return 1

    sdk.start_receive()
    print("已连接，进入交互式控制。输入 h 查看命令。")
    try:
        run_interactive(sdk)
    finally:
        sdk.stop_receive()
        sdk.disconnect()
    return 0


if __name__ == "__main__":
    sys.exit(main())
