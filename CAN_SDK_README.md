# 双手机械手CAN通信SDK

## 概述

本SDK提供了与双手机械手进行CAN总线通信的完整协议实现，支持双手独立控制和反馈读取。

## 特性

- ✅ 完整的CAN通信协议实现
- ✅ 支持左右手独立控制
- ✅ 位置、速度、扭矩三种数据类型
- ✅ 线程安全的数据读写
- ✅ 自动接收和数据解析
- ✅ 统计信息监控
- ✅ 回调函数支持
- ✅ 上下文管理器支持
- ✅ Doxygen风格注释

## 文件说明

```
├── can_robot_hand_sdk.py    # SDK核心文件
├── can_sdk_example.py       # 使用示例
├── CAN_SDK_README.md        # 本文档
└── can_host.py              # 原始上位机程序（已被SDK替代）
```

## 安装依赖

```bash
pip install python-can
```

## 协议说明

### CAN ID分配

| 手臂 | 类型 | ID范围 | 基地址 |
|------|------|--------|--------|
| 左手 | 控制 | 0x30-0x3F | 0x30 |
| 左手 | 反馈 | 0x40-0x4F | 0x40 |
| 右手 | 控制 | 0x50-0x5F | 0x50 |
| 右手 | 反馈 | 0x60-0x6F | 0x60 |

### 数据类型偏移

| 数据类型 | 偏移量 | 说明 |
|----------|--------|------|
| 位置 | n=0 | Position |
| 速度 | n=1 | Velocity |
| 扭矩/电流 | n=2 | Torque/Current |

### 数据格式

- **帧长度**: 8字节
- **关节数量**: 6个
- **数据位宽**: 每个关节10bit (0-1023)
- **字节序**: 小端序

**打包格式**:
```
Joint0: bit 0-9
Joint1: bit 10-19
Joint2: bit 20-29
Joint3: bit 30-39
Joint4: bit 40-49
Joint5: bit 50-59
Padding: bit 60-63 (填充0)
```

## 快速开始

### 示例1：基本使用

```python
from can_robot_hand_sdk import RobotHandSDK

# 创建SDK实例
sdk = RobotHandSDK(
    channel='PCAN_USBBUS1',  # CAN通道
    bustype='pcan',          # 总线类型
    bitrate=1000000          # 波特率
)

# 连接CAN总线
if sdk.connect():
    # 启动接收
    sdk.start_receive()
    
    # 发送左手位置命令
    sdk.send_left_control(position=[512, 512, 512, 512, 512, 512])
    
    # 等待接收反馈
    import time
    time.sleep(0.5)
    
    # 读取左手反馈
    left_fb = sdk.get_left_feedback()
    print(f"左手位置: {left_fb.position}")
    
    # 断开连接
    sdk.disconnect()
```

### 示例2：使用上下文管理器

```python
from can_robot_hand_sdk import RobotHandSDK

# 使用with语句自动管理连接
with RobotHandSDK(channel='PCAN_USBBUS1', bustype='pcan', bitrate=1000000) as sdk:
    # 发送控制命令
    sdk.send_left_control(position=[512] * 6)
    
    # 读取反馈
    left_fb = sdk.get_left_feedback()
    print(left_fb.position)
# 自动断开连接
```

### 示例3：便捷创建函数

```python
from can_robot_hand_sdk import create_sdk

# 自动连接并启动接收
sdk = create_sdk(
    channel='PCAN_USBBUS1',
    bustype='pcan',
    bitrate=1000000,
    auto_connect=True
)

# 直接使用
sdk.send_left_control(position=[512] * 6)
```

## API参考

### 类：RobotHandSDK

#### 初始化

```python
def __init__(self, 
             channel: str = 'can0', 
             bustype: str = 'socketcan', 
             bitrate: int = 500000,
             receive_timeout: float = 0.1)
```

**参数**:
- `channel`: CAN通道名称
  - Linux SocketCAN: `'can0'`, `'can1'`
  - Windows PCAN: `'PCAN_USBBUS1'`, `'PCAN_USBBUS2'`
  - Vector: `0`, `1`
- `bustype`: 总线类型
  - `'socketcan'`: Linux SocketCAN
  - `'pcan'`: PEAK PCAN
  - `'vector'`: Vector CANalyzer/CANoe
  - `'kvaser'`: Kvaser
- `bitrate`: 波特率（bps）
- `receive_timeout`: 接收超时时间（秒）

#### 连接管理

```python
def connect() -> bool
def disconnect()
def is_connected() -> bool
```

#### 数据发送

```python
def send_left_control(position=None, velocity=None, torque=None) -> bool
def send_right_control(position=None, velocity=None, torque=None) -> bool
def send_joint_data(base_id, data_type, joint_data) -> bool
```

**参数**:
- `position`: 位置数据列表，6个元素，范围0-1023
- `velocity`: 速度数据列表，6个元素，范围0-1023
- `torque`: 扭矩数据列表，6个元素，范围0-1023

**返回**: `True`=成功, `False`=失败

#### 数据接收

```python
def start_receive() -> bool
def stop_receive()
```

#### 数据读取

```python
def get_left_feedback() -> JointData
def get_right_feedback() -> JointData
def get_left_control() -> JointData
def get_right_control() -> JointData
```

**返回**: `JointData`对象，包含`position`、`velocity`、`torque`三个列表

#### 统计信息

```python
def get_statistics() -> Dict
def reset_statistics()
def print_status()
```

#### 回调函数

```python
def add_message_callback(callback: Callable[[can.Message], None])
def remove_message_callback(callback: Callable[[can.Message], None])
def set_error_callback(callback: Callable[[Exception], None])
```

### 类：JointData

```python
@dataclass
class JointData:
    position: List[int]  # 位置数据 (0-1023)
    velocity: List[int]  # 速度数据 (0-1023)
    torque: List[int]    # 扭矩数据 (0-1023)
```

### 枚举：DataType

```python
class DataType(IntEnum):
    POSITION = 0  # 位置数据
    VELOCITY = 1  # 速度数据
    TORQUE = 2    # 扭矩/电流数据
```

### 枚举：HandType

```python
class HandType(IntEnum):
    LEFT = 0   # 左手
    RIGHT = 1  # 右手
```

### 工具函数

```python
def pack_joint_data(joint_data: List[int]) -> bytes
def unpack_joint_data(can_data: bytes) -> List[int]
def create_sdk(...) -> RobotHandSDK
```

## 高级用法

### 使用回调函数

```python
from can_robot_hand_sdk import RobotHandSDK

def on_message(msg):
    """处理每个接收到的CAN消息"""
    print(f"收到消息: ID=0x{msg.arbitration_id:02X}")

def on_error(error):
    """处理错误"""
    print(f"错误: {error}")

sdk = RobotHandSDK(channel='PCAN_USBBUS1', bustype='pcan', bitrate=1000000)
sdk.add_message_callback(on_message)
sdk.set_error_callback(on_error)

if sdk.connect():
    sdk.start_receive()
    # ... 进行通信 ...
    sdk.disconnect()
```

### 双手协同控制

```python
from can_robot_hand_sdk import create_sdk
import math

with create_sdk(channel='PCAN_USBBUS1', bustype='pcan', bitrate=1000000) as sdk:
    for i in range(100):
        angle = i * 0.1
        
        # 左手和右手做相反的运动
        left_pos = [int(512 + 200 * math.sin(angle)) for _ in range(6)]
        right_pos = [int(512 - 200 * math.sin(angle)) for _ in range(6)]
        
        sdk.send_left_control(position=left_pos)
        sdk.send_right_control(position=right_pos)
        
        import time
        time.sleep(0.01)
```

### 监控统计信息

```python
from can_robot_hand_sdk import create_sdk

with create_sdk(channel='PCAN_USBBUS1', bustype='pcan', bitrate=1000000) as sdk:
    # 发送一些数据
    for _ in range(100):
        sdk.send_left_control(position=[512] * 6)
    
    # 获取统计信息
    stats = sdk.get_statistics()
    print(f"发送: {stats['tx_count']}")
    print(f"接收: {stats['rx_count']}")
    print(f"错误: {stats['tx_error_count']}")
    
    # 或者直接打印状态
    sdk.print_status()
```

## CAN适配器配置

### Linux SocketCAN

```bash
# 设置波特率并启动
sudo ip link set can0 type can bitrate 1000000
sudo ip link set can0 up

# 查看状态
ip -details link show can0

# 停止
sudo ip link set can0 down
```

### Windows PCAN

1. 安装PCAN驱动: https://www.peak-system.com/
2. 连接PCAN USB适配器
3. 使用SDK时指定`bustype='pcan'`, `channel='PCAN_USBBUS1'`

### Vector

1. 安装Vector驱动
2. 使用SDK时指定`bustype='vector'`, `channel=0`

## 示例程序

查看`can_sdk_example.py`获取更多使用示例：

```bash
python can_sdk_example.py
```

包含以下示例：
1. 基本使用方法
2. 使用上下文管理器
3. 连续控制（正弦波运动）
4. 使用回调函数
5. 双手协同控制
6. 统计信息监控

## 注意事项

1. **线程安全**: SDK内部使用锁保证线程安全，可以在多线程环境中使用
2. **数据范围**: 关节数据范围为0-1023（10bit），超出范围会自动截断
3. **接收延迟**: 调用`get_*_feedback()`返回的是最近一次接收到的数据
4. **连接管理**: 使用完毕后务必调用`disconnect()`或使用上下文管理器
5. **错误处理**: 建议设置错误回调函数以便及时处理错误

## 性能参考

- **最大通信频率**: 取决于CAN波特率和消息数量
  - 500kbps: 约400Hz（每只手3个消息）
  - 1000kbps: 约800Hz（每只手3个消息）
- **延迟**: 通常<1ms（取决于CAN适配器和系统负载）

## 协议版本

- **SDK版本**: 1.0.0
- **协议ID范围**: 0x30-0x6F
- **兼容STM32固件**: 参见`Core/Src/main.c`

## 许可证

根据项目许可证

## 联系方式

如有问题或建议，请联系项目维护者。
