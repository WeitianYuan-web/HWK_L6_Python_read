#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
简单的CAN测试程序
用于快速测试CAN通信功能
"""

import can
import struct
import time

# CAN配置
CHANNEL = 'PCAN_USBBUS1'        # Windows PCAN: 'PCAN_USBBUS1', Linux: 'can0'
BUSTYPE = 'pcan'   # Windows PCAN: 'pcan', Linux: 'socketcan'
BITRATE = 1000000

# CAN ID定义
CAN_ID_LEFT_CTRL_POS = 0x30     # 左手位置控制
CAN_ID_LEFT_FB_POS = 0x40       # 左手位置反馈
CAN_ID_RIGHT_CTRL_POS = 0x50    # 右手位置控制
CAN_ID_RIGHT_FB_POS = 0x60      # 右手位置反馈


def pack_joints(joints):
    """将6个关节数据打包为8字节"""
    packed = 0
    for i in range(6):
        packed |= (joints[i] & 0x3FF) << (i * 10)
    return struct.pack('<Q', packed)


def unpack_joints(data):
    """从8字节解包为6个关节数据"""
    packed = struct.unpack('<Q', data)[0]
    return [(packed >> (i * 10)) & 0x3FF for i in range(6)]


def main():
    print("="*60)
    print("  CAN通信简单测试程序")
    print("="*60)
    
    # 连接CAN总线
    try:
        bus = can.interface.Bus(
            channel=CHANNEL,
            bustype=BUSTYPE,
            bitrate=BITRATE
        )
        print(f"✓ CAN连接成功: {CHANNEL} @ {BITRATE}bps\n")
    except Exception as e:
        print(f"✗ CAN连接失败: {e}\n")
        print("Linux用户请先配置CAN接口：")
        print("  sudo ip link set can0 type can bitrate 500000")
        print("  sudo ip link set can0 up\n")
        return
    
    tx_count = 0
    rx_count = 0
    
    print("开始测试... (按Ctrl+C退出)\n")
    
    try:
        while True:
            # 发送测试数据
            test_joints = [100, 200, 300, 400, 500, 600]
            data = pack_joints(test_joints)
            
            msg = can.Message(
                arbitration_id=CAN_ID_LEFT_CTRL_POS,
                data=data,
                is_extended_id=False
            )
            
            bus.send(msg)
            tx_count += 1
            
            # 接收数据（非阻塞）
            msg = bus.recv(timeout=0.001)
            if msg:
                rx_count += 1
                joints = unpack_joints(msg.data)
                
                print(f"[RX] ID:0x{msg.arbitration_id:02X} "
                      f"Joints:{joints} (Total RX:{rx_count})")
            
            # 每100次打印一次发送状态
            if tx_count % 100 == 0:
                print(f"[TX] 已发送 {tx_count} 条消息")
            
            time.sleep(0.01)  # 100Hz
            
    except KeyboardInterrupt:
        print(f"\n\n统计信息：")
        print(f"  发送: {tx_count} 条")
        print(f"  接收: {rx_count} 条")
        print("\n✓ 测试结束\n")
    
    finally:
        bus.shutdown()


if __name__ == "__main__":
    main()
