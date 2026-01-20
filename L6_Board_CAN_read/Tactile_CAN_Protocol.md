# 触觉传感器 CAN 通讯协议

**版本**: 1.0  
**日期**: 2026-01-20  
**适用设备**: 灵巧手触觉传感器采集模块

---

## 1. 概述

本协议定义了灵巧手触觉传感器数据通过CAN总线传输的格式。每只手包含5个手指，每个手指配备一个12×6的触觉传感器矩阵。

### 1.1 系统参数

| 参数 | 数值 | 说明 |
|------|------|------|
| CAN 波特率 | 1 Mbps | 标准CAN 2.0B |
| 手指数量 | 5 | 大拇指、食指、中指、无名指、小指 |
| 矩阵尺寸 | 12列 × 6行 | 每个传感器72个触点 |
| 数据位深 | 8位 | 0-255 |
| 每帧数据 | 8字节 | 标准CAN帧 |
| 每手指帧数 | 9帧 | 72字节 ÷ 8字节 = 9帧 |
| 每手总帧数 | 45帧 | 5手指 × 9帧 |
| 典型帧率 | 50-200 FPS | 取决于系统配置 |

---

## 2. CAN ID 分配

### 2.1 ID 格式

```
CAN ID = 0xHmn

H = 手臂标识 (3=左手, 4=右手)
m = 手指ID (0-4)
n = 帧序号 (0-8)
```

### 2.2 左手 CAN ID 范围 (0x300 - 0x348)

| 手指 | 手指ID | CAN ID 范围 | 帧序号 |
|------|--------|-------------|--------|
| 大拇指 (Thumb) | 0 | 0x300 - 0x308 | 0-8 |
| 食指 (Index) | 1 | 0x310 - 0x318 | 0-8 |
| 中指 (Middle) | 2 | 0x320 - 0x328 | 0-8 |
| 无名指 (Ring) | 3 | 0x330 - 0x338 | 0-8 |
| 小指 (Pinky) | 4 | 0x340 - 0x348 | 0-8 |

### 2.3 右手 CAN ID 范围 (0x400 - 0x448)

| 手指 | 手指ID | CAN ID 范围 | 帧序号 |
|------|--------|-------------|--------|
| 大拇指 (Thumb) | 0 | 0x400 - 0x408 | 0-8 |
| 食指 (Index) | 1 | 0x410 - 0x418 | 0-8 |
| 中指 (Middle) | 2 | 0x420 - 0x428 | 0-8 |
| 无名指 (Ring) | 3 | 0x430 - 0x438 | 0-8 |
| 小指 (Pinky) | 4 | 0x440 - 0x448 | 0-8 |

### 2.4 CAN ID 解析算法

```c
/**
 * @brief 解析CAN ID
 * @param can_id CAN标识符
 * @param hand   输出: 0=左手, 1=右手
 * @param finger 输出: 手指ID (0-4)
 * @param frame  输出: 帧序号 (0-8)
 * @return true=有效, false=无效
 */
bool parse_tactile_can_id(uint32_t can_id, uint8_t *hand, uint8_t *finger, uint8_t *frame)
{
    uint8_t base = (can_id >> 8) & 0x0F;
    
    if (base == 0x03) {
        *hand = 0;  // 左手
    } else if (base == 0x04) {
        *hand = 1;  // 右手
    } else {
        return false;  // 非触觉传感器帧
    }
    
    *finger = (can_id >> 4) & 0x0F;
    *frame = can_id & 0x0F;
    
    return (*finger <= 4 && *frame <= 8);
}
```

---

## 3. 数据帧格式

### 3.1 单帧结构

| 字段 | 位数 | 说明 |
|------|------|------|
| CAN ID | 11位 | 标准帧标识符 |
| DLC | 4位 | 数据长度 = 8 |
| Data[0-7] | 64位 | 8字节矩阵数据 |

### 3.2 数据分帧

每个传感器72字节数据分为9帧传输：

| 帧序号 | 字节偏移 | 包含数据 |
|--------|----------|----------|
| 0 | 0-7 | 矩阵字节 0-7 |
| 1 | 8-15 | 矩阵字节 8-15 |
| 2 | 16-23 | 矩阵字节 16-23 |
| 3 | 24-31 | 矩阵字节 24-31 |
| 4 | 32-39 | 矩阵字节 32-39 |
| 5 | 40-47 | 矩阵字节 40-47 |
| 6 | 48-55 | 矩阵字节 48-55 |
| 7 | 56-63 | 矩阵字节 56-63 |
| 8 | 64-71 | 矩阵字节 64-71 |

---

## 4. 矩阵数据格式

### 4.1 矩阵规格

- **尺寸**: 12列 × 6行 = 72个触点
- **数据类型**: uint8_t (0-255)
- **排列方式**: **列优先 (Column-Major)**

### 4.2 数据排列顺序

```
字节索引:  0    1    2    3    4    5    6    7   ...  66   67   68   69   70   71
矩阵位置: C0R0 C0R1 C0R2 C0R3 C0R4 C0R5 C1R0 C1R1 ... C11R0 C11R1 C11R2 C11R3 C11R4 C11R5
          |-------- 列0 --------| |-------- 列1 --------|   |----------- 列11 -----------|
```

### 4.3 矩阵布局图

```
        列0   列1   列2   列3   列4   列5   列6   列7   列8   列9   列10  列11
      +-----+-----+-----+-----+-----+-----+-----+-----+-----+-----+-----+-----+
行0   | [0] | [6] | [12]| [18]| [24]| [30]| [36]| [42]| [48]| [54]| [60]| [66]|
      +-----+-----+-----+-----+-----+-----+-----+-----+-----+-----+-----+-----+
行1   | [1] | [7] | [13]| [19]| [25]| [31]| [37]| [43]| [49]| [55]| [61]| [67]|
      +-----+-----+-----+-----+-----+-----+-----+-----+-----+-----+-----+-----+
行2   | [2] | [8] | [14]| [20]| [26]| [32]| [38]| [44]| [50]| [56]| [62]| [68]|
      +-----+-----+-----+-----+-----+-----+-----+-----+-----+-----+-----+-----+
行3   | [3] | [9] | [15]| [21]| [27]| [33]| [39]| [45]| [51]| [57]| [63]| [69]|
      +-----+-----+-----+-----+-----+-----+-----+-----+-----+-----+-----+-----+
行4   | [4] | [10]| [16]| [22]| [28]| [34]| [40]| [46]| [52]| [58]| [64]| [70]|
      +-----+-----+-----+-----+-----+-----+-----+-----+-----+-----+-----+-----+
行5   | [5] | [11]| [17]| [23]| [29]| [35]| [41]| [47]| [53]| [59]| [65]| [71]|
      +-----+-----+-----+-----+-----+-----+-----+-----+-----+-----+-----+-----+

[n] = 原始数据中的字节索引
```

### 4.4 坐标转换公式

```c
// 列优先索引 -> 行列坐标
col = byte_index / 6;
row = byte_index % 6;

// 行列坐标 -> 列优先索引
byte_index = col * 6 + row;

// 列优先转行优先 (推荐在接收端转换)
for (col = 0; col < 12; col++) {
    for (row = 0; row < 6; row++) {
        matrix[row][col] = raw_data[col * 6 + row];
    }
}
```

---

## 5. 传输时序

### 5.1 单手指传输

```
时间轴 →
|----帧0----|----帧1----|----帧2----| ... |----帧8----|
   ~130μs      ~130μs      ~130μs           ~130μs

单手指传输时间 ≈ 9帧 × 130μs ≈ 1.17ms
```

### 5.2 单手完整传输

```
|--大拇指(9帧)--|--食指(9帧)--|--中指(9帧)--|--无名指(9帧)--|--小指(9帧)--|
    1.17ms         1.17ms        1.17ms         1.17ms          1.17ms

单手传输时间 ≈ 45帧 × 130μs ≈ 5.85ms
理论最大帧率 ≈ 170 FPS (仅考虑CAN传输)
```

### 5.3 双手传输（需要两个CAN节点）

- 左手: CAN ID 0x300-0x348
- 右手: CAN ID 0x400-0x448
- 两手可并行传输，无ID冲突

---

## 6. 数据完整性

### 6.1 帧完整性检测

接收端需要检测是否收到完整的9帧数据：

```c
typedef struct {
    uint8_t raw_data[72];    // 原始数据缓冲
    uint16_t frame_mask;     // 已接收帧位掩码
    bool complete;           // 数据完整标志
} SensorBuffer_t;

// 处理接收到的CAN帧
void process_frame(SensorBuffer_t *buf, uint8_t frame_idx, uint8_t *data)
{
    // 存储数据
    memcpy(&buf->raw_data[frame_idx * 8], data, 8);
    
    // 标记帧已接收
    buf->frame_mask |= (1 << frame_idx);
    
    // 检查完整性 (0x1FF = 0b111111111)
    buf->complete = (buf->frame_mask == 0x1FF);
}
```

### 6.2 超时处理

建议设置帧组超时：

| 参数 | 推荐值 | 说明 |
|------|--------|------|
| 单帧超时 | 10ms | 单帧未到达视为丢失 |
| 帧组超时 | 50ms | 未收齐9帧重置状态 |

---

## 7. 示例数据

### 7.1 左手食指完整数据示例

```
CAN帧序列 (左手食指, 手指ID=1):

帧0: ID=0x310, DLC=8, Data=[0x12, 0x34, 0x56, 0x78, 0x9A, 0xBC, 0xDE, 0xF0]
帧1: ID=0x311, DLC=8, Data=[0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77, 0x88]
帧2: ID=0x312, DLC=8, Data=[0xAA, 0xBB, 0xCC, 0xDD, 0xEE, 0xFF, 0x00, 0x11]
帧3: ID=0x313, DLC=8, Data=[...]
帧4: ID=0x314, DLC=8, Data=[...]
帧5: ID=0x315, DLC=8, Data=[...]
帧6: ID=0x316, DLC=8, Data=[...]
帧7: ID=0x317, DLC=8, Data=[...]
帧8: ID=0x318, DLC=8, Data=[...]
```

### 7.2 数据重组后的矩阵

```
接收到的72字节重组为12×6矩阵:

        C0    C1    C2    C3    C4    C5    C6    C7    C8    C9   C10   C11
   +------+------+------+------+------+------+------+------+------+------+------+------+
R0 | 0x12 | 0xDE | 0x33 | 0xAA | ...  | ...  | ...  | ...  | ...  | ...  | ...  | ...  |
R1 | 0x34 | 0xF0 | 0x44 | 0xBB | ...  | ...  | ...  | ...  | ...  | ...  | ...  | ...  |
R2 | 0x56 | 0x11 | 0x55 | 0xCC | ...  | ...  | ...  | ...  | ...  | ...  | ...  | ...  |
R3 | 0x78 | 0x22 | 0x66 | 0xDD | ...  | ...  | ...  | ...  | ...  | ...  | ...  | ...  |
R4 | 0x9A | 0x33 | 0x77 | 0xEE | ...  | ...  | ...  | ...  | ...  | ...  | ...  | ...  |
R5 | 0xBC | 0x44 | 0x88 | 0xFF | ...  | ...  | ...  | ...  | ...  | ...  | ...  | ...  |
   +------+------+------+------+------+------+------+------+------+------+------+------+
```

---

## 8. 上位机实现参考

### 8.1 Python 示例

```python
import can
import numpy as np

class TactileReceiver:
    """触觉传感器CAN接收器"""
    
    def __init__(self, channel='can0', bustype='socketcan'):
        self.bus = can.interface.Bus(channel=channel, bustype=bustype)
        self.buffers = {}  # {(hand, finger): SensorBuffer}
    
    def parse_can_id(self, can_id):
        """解析CAN ID"""
        base = (can_id >> 8) & 0x0F
        if base == 0x03:
            hand = 0  # 左手
        elif base == 0x04:
            hand = 1  # 右手
        else:
            return None
        
        finger = (can_id >> 4) & 0x0F
        frame = can_id & 0x0F
        
        if finger > 4 or frame > 8:
            return None
        
        return (hand, finger, frame)
    
    def process_message(self, msg):
        """处理CAN消息"""
        result = self.parse_can_id(msg.arbitration_id)
        if result is None:
            return None
        
        hand, finger, frame = result
        key = (hand, finger)
        
        # 初始化缓冲区
        if key not in self.buffers:
            self.buffers[key] = {
                'data': bytearray(72),
                'mask': 0
            }
        
        buf = self.buffers[key]
        
        # 存储数据
        offset = frame * 8
        buf['data'][offset:offset+8] = msg.data[:8]
        buf['mask'] |= (1 << frame)
        
        # 检查完整性
        if buf['mask'] == 0x1FF:
            matrix = self._convert_to_matrix(buf['data'])
            buf['mask'] = 0  # 重置
            return (hand, finger, matrix)
        
        return None
    
    def _convert_to_matrix(self, raw_data):
        """列优先转行优先矩阵"""
        matrix = np.zeros((6, 12), dtype=np.uint8)
        idx = 0
        for col in range(12):
            for row in range(6):
                matrix[row, col] = raw_data[idx]
                idx += 1
        return matrix
    
    def run(self):
        """主循环"""
        hand_names = ['左手', '右手']
        finger_names = ['大拇指', '食指', '中指', '无名指', '小指']
        
        while True:
            msg = self.bus.recv(timeout=1.0)
            if msg:
                result = self.process_message(msg)
                if result:
                    hand, finger, matrix = result
                    print(f"\n{hand_names[hand]} {finger_names[finger]}:")
                    print(matrix)
```

### 8.2 C++ 示例

```cpp
#include <cstdint>
#include <cstring>
#include <array>

class TactileSensor {
public:
    static constexpr int ROWS = 6;
    static constexpr int COLS = 12;
    static constexpr int DATA_SIZE = 72;
    static constexpr int FRAMES = 9;
    
    struct SensorData {
        uint8_t hand;                           // 0=左手, 1=右手
        uint8_t finger;                         // 0-4
        std::array<std::array<uint8_t, COLS>, ROWS> matrix;
        bool valid;
    };
    
    /**
     * @brief 解析CAN帧
     * @return 如果数据完整返回true
     */
    bool processFrame(uint32_t canId, const uint8_t* data) {
        uint8_t hand, finger, frame;
        if (!parseCanId(canId, hand, finger, frame)) {
            return false;
        }
        
        int bufIdx = hand * 5 + finger;
        auto& buf = buffers_[bufIdx];
        
        // 存储数据
        std::memcpy(&buf.rawData[frame * 8], data, 8);
        buf.frameMask |= (1 << frame);
        
        // 检查完整性
        if (buf.frameMask == 0x1FF) {
            convertToMatrix(buf.rawData, lastData_.matrix);
            lastData_.hand = hand;
            lastData_.finger = finger;
            lastData_.valid = true;
            buf.frameMask = 0;
            return true;
        }
        
        return false;
    }
    
    const SensorData& getLastData() const { return lastData_; }
    
private:
    struct Buffer {
        uint8_t rawData[DATA_SIZE] = {0};
        uint16_t frameMask = 0;
    };
    
    std::array<Buffer, 10> buffers_;  // 2手 × 5手指
    SensorData lastData_;
    
    bool parseCanId(uint32_t id, uint8_t& hand, uint8_t& finger, uint8_t& frame) {
        uint8_t base = (id >> 8) & 0x0F;
        if (base == 0x03) hand = 0;
        else if (base == 0x04) hand = 1;
        else return false;
        
        finger = (id >> 4) & 0x0F;
        frame = id & 0x0F;
        
        return (finger <= 4 && frame <= 8);
    }
    
    void convertToMatrix(const uint8_t* raw, 
                         std::array<std::array<uint8_t, COLS>, ROWS>& matrix) {
        int idx = 0;
        for (int col = 0; col < COLS; col++) {
            for (int row = 0; row < ROWS; row++) {
                matrix[row][col] = raw[idx++];
            }
        }
    }
};
```

---

## 9. 故障排查

| 现象 | 可能原因 | 解决方法 |
|------|----------|----------|
| 无CAN数据 | CAN未启动/接线错误 | 检查CAN初始化和硬件连接 |
| 数据不完整 | 帧丢失 | 增加超时重传机制 |
| CAN ID不在范围 | 左右手配置错误 | 检查USE_LEFT_HAND宏定义 |
| 矩阵数据错位 | 列优先/行优先混淆 | 确认使用正确的转换算法 |
| 帧率低 | CAN总线拥堵 | 检查总线负载率 |

---

## 10. 版本历史

| 版本 | 日期 | 修改内容 |
|------|------|----------|
| 1.0 | 2026-01-20 | 初始版本，支持左右手选择 |

---

## 附录 A: CAN ID 快速参考表

### 左手 (0x3xx)

| 帧序号 | 大拇指 | 食指 | 中指 | 无名指 | 小指 |
|--------|--------|------|------|--------|------|
| 0 | 0x300 | 0x310 | 0x320 | 0x330 | 0x340 |
| 1 | 0x301 | 0x311 | 0x321 | 0x331 | 0x341 |
| 2 | 0x302 | 0x312 | 0x322 | 0x332 | 0x342 |
| 3 | 0x303 | 0x313 | 0x323 | 0x333 | 0x343 |
| 4 | 0x304 | 0x314 | 0x324 | 0x334 | 0x344 |
| 5 | 0x305 | 0x315 | 0x325 | 0x335 | 0x345 |
| 6 | 0x306 | 0x316 | 0x326 | 0x336 | 0x346 |
| 7 | 0x307 | 0x317 | 0x327 | 0x337 | 0x347 |
| 8 | 0x308 | 0x318 | 0x328 | 0x338 | 0x348 |

### 右手 (0x4xx)

| 帧序号 | 大拇指 | 食指 | 中指 | 无名指 | 小指 |
|--------|--------|------|------|--------|------|
| 0 | 0x400 | 0x410 | 0x420 | 0x430 | 0x440 |
| 1 | 0x401 | 0x411 | 0x421 | 0x431 | 0x441 |
| 2 | 0x402 | 0x412 | 0x422 | 0x432 | 0x442 |
| 3 | 0x403 | 0x413 | 0x423 | 0x433 | 0x443 |
| 4 | 0x404 | 0x414 | 0x424 | 0x434 | 0x444 |
| 5 | 0x405 | 0x415 | 0x425 | 0x435 | 0x445 |
| 6 | 0x406 | 0x416 | 0x426 | 0x436 | 0x446 |
| 7 | 0x407 | 0x417 | 0x427 | 0x437 | 0x447 |
| 8 | 0x408 | 0x418 | 0x428 | 0x438 | 0x448 |
