# 无刷拉力模块集成使用说明

## 概述

本文档说明如何在 `tactile_control_visualizer.py` 中使用集成的无刷拉力模块控制功能。

## 系统架构

系统使用 **两个独立的CAN总线**：

### CAN总线1 - 机械手和触觉传感器
- **默认通道**: `PCAN_USBBUS1`
- **功能**:
  - 触觉传感器数据接收 (CAN ID: 0x300-0x448)
  - 机械手控制命令发送 (CAN ID: 0x30-0x5F)
  - 机械手反馈数据接收 (CAN ID: 0x40-0x6F)

### CAN总线2 - 无刷拉力模块
- **默认通道**: `PCAN_USBBUS2`
- **功能**:
  - 拉力命令发送 (CAN ID: 0x200)
  - 位置命令发送 (CAN ID: 0x201)
  - 模块反馈接收 (CAN ID: 0x20-0x24)

## 控制算法

### 拉力命令计算

```
拉力命令 = (触觉归一化 × 0.8 + 电流归一化 × 0.2) × 255
```

- **触觉传感器归一化**: 
  - 每个手指取触觉矩阵的最大值
  - 最大值255归一化为1.0

- **关节电流归一化**:
  - 512 → 0.0 (零电流)
  - 0 → 1.0 (最大电流)
  - 线性插值

### 位置命令计算

```
位置命令 = 关节位置归一化 × (255 - 50) + 50
```

- **关节位置归一化**:
  - 0 → 0.0
  - 1023 → 1.0
  
- **映射范围**: 50 ~ 255

## 使用方法

### 基本用法（不启用拉力控制）

```bash
python tactile_control_visualizer.py
```

### 启用无刷拉力模块控制

```bash
python tactile_control_visualizer.py --enable-tension
```

### 自定义CAN通道

```bash
python tactile_control_visualizer.py \
    --enable-tension \
    --channel PCAN_USBBUS1 \
    --tension-channel PCAN_USBBUS2
```

### 完整参数示例

```bash
python tactile_control_visualizer.py \
    --hand 1 \
    --channel PCAN_USBBUS1 \
    --tension-channel PCAN_USBBUS2 \
    --serial-port COM69 \
    --enable-tension \
    --fps 60
```

## 命令行参数

| 参数 | 说明 | 默认值 |
|------|------|--------|
| `--hand` | 选择手 (0=左手, 1=右手) | 1 |
| `--channel` | 机械手和触觉传感器CAN通道 | PCAN_USBBUS1 |
| `--tension-channel` | 无刷拉力模块CAN通道 | PCAN_USBBUS2 |
| `--interface` | CAN接口类型 | pcan |
| `--tension-interface` | 拉力模块CAN接口类型 | pcan |
| `--serial-port` | 控制器串口 | COM69 |
| `--enable-tension` | 启用无刷拉力模块控制 | 关闭 |
| `--no-control` | 禁用控制器 | 关闭 |
| `--simulate` | 使用模拟数据 | 关闭 |
| `--fps` | 目标帧率 | 60 |

## 可视化界面

启用拉力控制后，界面会额外显示：

1. **拉力命令** (橙色): 显示5个手指的拉力命令值 (0-255)
2. **拉力位置** (蓝色): 显示5个手指的位置命令值 (50-255)

示例输出：
```
拉力: F0:125 | F1:98 | F2:156 | F3:88 | F4:203
拉力位置: F0:120 | F1:95 | F2:150 | F3:85 | F4:200
```

## 配置调整

如需调整控制参数，修改 `ControlConfig` 类：

```python
class ControlConfig:
    # 无刷拉力模块配置
    TENSION_TACTILE_WEIGHT = 0.8      # 触觉传感器权重
    TENSION_CURRENT_WEIGHT = 0.2      # 关节电流权重
    TENSION_CURRENT_ZERO = 512        # 关节电流零点
    TENSION_CURRENT_MAX = 0           # 关节电流最大值
    TENSION_POS_MIN = 50              # 位置命令最小值
    TENSION_POS_MAX = 255             # 位置命令最大值
```

## 注意事项

1. **硬件连接**: 确保两个PCAN适配器正确连接到对应的CAN总线
2. **通道配置**: 确认PCAN_USBBUS1和PCAN_USBBUS2分别对应正确的硬件
3. **波特率**: 默认都使用1Mbps，确保硬件设备支持此波特率
4. **控制频率**: 系统以200Hz频率发送控制命令
5. **数据同步**: 拉力控制基于实时的触觉传感器和关节反馈数据

## 故障排除

### 无法连接无刷拉力模块

```
✗ 无刷拉力模块连接失败，将禁用拉力控制
```

**解决方法**:
1. 检查PCAN_USBBUS2是否正确连接
2. 确认设备管理器中显示PCAN设备
3. 尝试指定不同的通道: `--tension-channel PCAN_USBBUS3`

### 拉力命令不更新

**可能原因**:
1. 触觉传感器数据未接收到
2. 机械手反馈数据未接收到
3. 控制器未启动 (检查 `--no-control` 参数)

**解决方法**:
- 检查界面上"触觉传感器"和"机械手反馈"是否有数据更新
- 按 `R` 键重置统计信息
- 按 `C` 键切换控制开关

## 示例场景

### 场景1: 测试拉力控制（模拟模式）

```bash
python tactile_control_visualizer.py --simulate --enable-tension
```

### 场景2: 实际应用（右手）

```bash
python tactile_control_visualizer.py \
    --hand 1 \
    --enable-tension \
    --channel PCAN_USBBUS1 \
    --tension-channel PCAN_USBBUS2
```

### 场景3: 仅查看数据不控制

```bash
python tactile_control_visualizer.py \
    --no-control \
    --enable-tension
```

## 技术支持

如有问题，请检查：
1. 系统输出日志
2. CAN连接状态
3. 数据更新频率（界面右上角显示）
