#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
双手机械手CAN通信SDK使用示例

@file    can_sdk_example.py
@brief   展示如何使用can_robot_hand_sdk进行CAN通信
@date    2025-01-07
"""

import time
import math
from can_robot_hand_sdk import (
    RobotHandSDK, 
    create_sdk,
    CANProtocol,
    DataType
)


def example1_basic_usage():
    """
    @brief 示例1：基本使用方法
    """
    print("\n" + "="*70)
    print("  示例1：基本使用方法")
    print("="*70)
    
    # 创建SDK实例
    sdk = RobotHandSDK(
        channel='PCAN_USBBUS1',  # Windows PCAN
        bustype='pcan',
        bitrate=1000000
    )
    
    # 连接CAN总线
    if not sdk.connect():
        print("无法连接到CAN总线")
        return
    
    # 启动接收线程
    sdk.start_receive()
    
    try:
        # 发送左手位置控制命令
        left_position = [512, 512, 512, 512, 512, 512]
        sdk.send_left_control(position=left_position)
        
        # 等待接收数据
        time.sleep(0.5)
        
        # 读取左手反馈
        left_fb = sdk.get_left_feedback()
        print(f"\n左手位置反馈: {left_fb.position}")
        print(f"左手速度反馈: {left_fb.velocity}")
        print(f"左手扭矩反馈: {left_fb.torque}")
        
        # 打印统计信息
        sdk.print_status()
        
    finally:
        # 断开连接
        sdk.disconnect()


def example2_context_manager():
    """
    @brief 示例2：使用上下文管理器
    """
    print("\n" + "="*70)
    print("  示例2：使用上下文管理器")
    print("="*70)
    
    # 使用with语句自动管理连接
    with RobotHandSDK(channel='PCAN_USBBUS1', bustype='pcan', bitrate=1000000) as sdk:
        # 发送控制命令
        sdk.send_left_control(position=[512] * 6)
        sdk.send_right_control(position=[512] * 6)
        
        time.sleep(0.5)
        
        # 读取反馈
        left_fb = sdk.get_left_feedback()
        right_fb = sdk.get_right_feedback()
        
        print(f"\n左手位置: {left_fb.position}")
        print(f"右手位置: {right_fb.position}")


def example3_continuous_control():
    """
    @brief 示例3：连续控制（正弦波运动）
    """
    print("\n" + "="*70)
    print("  示例3：连续控制（正弦波运动）")
    print("="*70)
    
    sdk = create_sdk(
        channel='PCAN_USBBUS1',
        bustype='pcan',
        bitrate=1000000,
        auto_connect=True
    )
    
    print("\n开始发送正弦波控制命令...")
    print("按 Ctrl+C 停止\n")
    
    try:
        cycle = 0
        while True:
            cycle += 1
            
            # 生成正弦波位置数据
            angle = cycle * 0.1
            left_pos = [
                int(512 + 200 * math.sin(angle + i * 0.5))
                for i in range(CANProtocol.JOINT_COUNT)
            ]
            
            # 发送左手位置控制
            sdk.send_left_control(position=left_pos)
            
            # 每100个周期打印一次状态
            if cycle % 100 == 0:
                sdk.print_status()
            
            # 10ms控制周期（100Hz）
            time.sleep(0.01)
            
    except KeyboardInterrupt:
        print("\n\n用户中断")
    finally:
        sdk.disconnect()


def example4_callback():
    """
    @brief 示例4：使用回调函数
    """
    print("\n" + "="*70)
    print("  示例4：使用回调函数")
    print("="*70)
    
    # 定义消息回调函数
    def on_message(msg):
        """处理接收到的CAN消息"""
        print(f"收到消息: ID=0x{msg.arbitration_id:02X}, "
              f"数据={' '.join(f'{b:02X}' for b in msg.data)}")
    
    # 定义错误回调函数
    def on_error(error):
        """处理错误"""
        print(f"发生错误: {error}")
    
    sdk = RobotHandSDK(channel='PCAN_USBBUS1', bustype='pcan', bitrate=1000000)
    
    # 注册回调函数
    sdk.add_message_callback(on_message)
    sdk.set_error_callback(on_error)
    
    if sdk.connect():
        sdk.start_receive()
        
        # 发送一些测试数据
        sdk.send_left_control(position=[512] * 6)
        
        # 等待接收
        time.sleep(2)
        
        sdk.disconnect()


def example5_dual_hand_control():
    """
    @brief 示例5：双手协同控制
    """
    print("\n" + "="*70)
    print("  示例5：双手协同控制")
    print("="*70)
    
    with create_sdk(channel='PCAN_USBBUS1', bustype='pcan', bitrate=1000000) as sdk:
        print("\n开始双手协同运动...")
        
        try:
            for i in range(100):
                # 左手和右手做相反的运动
                angle = i * 0.1
                
                left_pos = [int(512 + 200 * math.sin(angle)) for _ in range(6)]
                right_pos = [int(512 - 200 * math.sin(angle)) for _ in range(6)]
                
                # 同时发送双手控制命令
                sdk.send_left_control(position=left_pos)
                sdk.send_right_control(position=right_pos)
                
                if i % 20 == 0:
                    left_fb = sdk.get_left_feedback()
                    right_fb = sdk.get_right_feedback()
                    print(f"\n周期 {i}:")
                    print(f"  左手位置: {left_fb.position[0]}")
                    print(f"  右手位置: {right_fb.position[0]}")
                
                time.sleep(0.01)
                
        except KeyboardInterrupt:
            print("\n用户中断")


def example6_statistics():
    """
    @brief 示例6：统计信息监控
    """
    print("\n" + "="*70)
    print("  示例6：统计信息监控")
    print("="*70)
    
    with create_sdk(channel='PCAN_USBBUS1', bustype='pcan', bitrate=1000000) as sdk:
        # 重置统计信息
        sdk.reset_statistics()
        
        print("\n开始发送数据...")
        
        # 发送100条消息
        for i in range(100):
            sdk.send_left_control(position=[512] * 6)
            time.sleep(0.01)
        
        # 获取统计信息
        stats = sdk.get_statistics()
        
        print(f"\n统计信息:")
        print(f"  发送消息数: {stats['tx_count']}")
        print(f"  接收消息数: {stats['rx_count']}")
        print(f"  发送错误数: {stats['tx_error_count']}")
        print(f"\n各ID接收统计:")
        for can_id, count in sorted(stats['rx_count_by_id'].items()):
            print(f"  0x{can_id:02X}: {count} 条")


def main():
    """
    @brief 主函数 - 选择运行哪个示例
    """
    print("\n双手机械手CAN通信SDK - 使用示例")
    print("="*70)
    print("\n可用示例:")
    print("  1. 基本使用方法")
    print("  2. 使用上下文管理器")
    print("  3. 连续控制（正弦波运动）")
    print("  4. 使用回调函数")
    print("  5. 双手协同控制")
    print("  6. 统计信息监控")
    print("  0. 退出")
    
    while True:
        try:
            choice = input("\n请选择示例 (0-6): ").strip()
            
            if choice == '1':
                example1_basic_usage()
            elif choice == '2':
                example2_context_manager()
            elif choice == '3':
                example3_continuous_control()
            elif choice == '4':
                example4_callback()
            elif choice == '5':
                example5_dual_hand_control()
            elif choice == '6':
                example6_statistics()
            elif choice == '0':
                print("\n退出程序")
                break
            else:
                print("无效选择，请重试")
                
        except KeyboardInterrupt:
            print("\n\n用户中断")
            break
        except Exception as e:
            print(f"\n发生错误: {e}")


if __name__ == "__main__":
    main()
