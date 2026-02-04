"""
矩阵传感器 Python SDK

本模块提供矩阵传感器的上位机通信接口，支持：
- 设备连接与管理
- 数据请求与接收
- 矩阵数据解析
- 实时数据可视化

协议版本: v1.1
波特率: 1152000 bps
矩阵规格: 12列 × 6行
"""

import serial
import serial.tools.list_ports
import time
import threading
import numpy as np
from typing import Optional, List, Tuple, Callable
from dataclasses import dataclass
from enum import IntEnum


class Command(IntEnum):
    """协议命令字定义"""
    REQUEST = 0x01   # 请求命令
    RESPONSE = 0x02  # 响应命令


class FrameConfig:
    """帧配置常量"""
    HEADER = bytes([0xAA, 0x55])  # 帧头
    REQUEST_LEN = 5               # 请求帧长度
    RESPONSE_LEN = 78             # 响应帧长度
    MATRIX_DATA_LEN = 72          # 矩阵数据长度
    MATRIX_COLS = 12              # 矩阵列数
    MATRIX_ROWS = 6               # 矩阵行数


@dataclass
class SensorResponse:
    """
    传感器响应数据结构
    
    Attributes:
        device_id: 设备ID
        raw_data: 原始72字节矩阵数据
        matrix: 12x6 NumPy矩阵 (列优先转换后)
        timestamp: 接收时间戳
        valid: 数据是否有效
    """
    device_id: int
    raw_data: bytes
    matrix: np.ndarray
    timestamp: float
    valid: bool


class MatrixSensor:
    """
    矩阵传感器通信类
    
    提供与矩阵传感器的串口通信功能，支持同步和异步数据读取。
    
    Example:
        >>> sensor = MatrixSensor('COM3')
        >>> sensor.connect()
        >>> response = sensor.read_matrix(device_id=0x01)
        >>> if response.valid:
        ...     print(response.matrix)
        >>> sensor.disconnect()
    """
    
    def __init__(self, port: str, baudrate: int = 1152000, timeout: float = 0.01):
        """
        初始化传感器实例
        
        Args:
            port: 串口端口号 (如 'COM3' 或 '/dev/ttyUSB0')
            baudrate: 波特率，默认 1152000
            timeout: 读取超时时间(秒)，默认 10ms
        """
        self.port = port
        self.baudrate = baudrate
        self.timeout = timeout
        self._serial: Optional[serial.Serial] = None
        self._lock = threading.Lock()
        self._is_streaming = False
        self._stream_thread: Optional[threading.Thread] = None
        
    @staticmethod
    def list_ports() -> List[str]:
        """
        列出所有可用的串口
        
        Returns:
            可用串口端口列表
        """
        ports = serial.tools.list_ports.comports()
        return [port.device for port in ports]
    
    @staticmethod
    def find_sensor_port() -> Optional[str]:
        """
        自动查找传感器端口 (尝试发送请求并检测响应)
        
        Returns:
            找到的端口号，未找到返回 None
        """
        for port in MatrixSensor.list_ports():
            try:
                sensor = MatrixSensor(port, timeout=0.5)
                sensor.connect()
                response = sensor.read_matrix(device_id=0x01)
                sensor.disconnect()
                if response.valid:
                    return port
            except Exception:
                pass
        return None
    
    @property
    def is_connected(self) -> bool:
        """检查是否已连接"""
        return self._serial is not None and self._serial.is_open
    
    def connect(self) -> bool:
        """
        连接传感器
        
        Returns:
            连接是否成功
            
        Raises:
            serial.SerialException: 端口打开失败
        """
        if self.is_connected:
            return True
            
        try:
            self._serial = serial.Serial(
                port=self.port,
                baudrate=self.baudrate,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                timeout=self.timeout
            )
            # 清空缓冲区
            self._serial.reset_input_buffer()
            self._serial.reset_output_buffer()
            return True
        except serial.SerialException as e:
            print(f"连接失败: {e}")
            return False
    
    def disconnect(self) -> None:
        """断开连接"""
        self.stop_streaming()
        if self._serial is not None:
            self._serial.close()
            self._serial = None
    
    @staticmethod
    def _calculate_checksum(data: bytes) -> int:
        """
        计算校验和 (字节累加 & 0xFF)
        
        Args:
            data: 待校验数据
            
        Returns:
            校验和值
        """
        return sum(data) & 0xFF
    
    def _build_request_frame(self, device_id: int) -> bytes:
        """
        构建请求帧
        
        Args:
            device_id: 目标设备ID (0x01-0xFF)
            
        Returns:
            5字节请求帧
        """
        frame = bytearray([0xAA, 0x55, Command.REQUEST, device_id])
        checksum = self._calculate_checksum(frame)
        frame.append(checksum)
        return bytes(frame)
    
    def _parse_response(self, data: bytes) -> SensorResponse:
        """
        解析响应帧
        
        Args:
            data: 接收到的原始数据
            
        Returns:
            SensorResponse 对象
        """
        timestamp = time.time()
        
        # 长度检查
        if len(data) < FrameConfig.RESPONSE_LEN:
            return SensorResponse(
                device_id=0,
                raw_data=b'',
                matrix=np.zeros((FrameConfig.MATRIX_COLS, FrameConfig.MATRIX_ROWS), dtype=np.uint8),
                timestamp=timestamp,
                valid=False
            )
        
        # 帧头检查
        if data[0] != 0xAA or data[1] != 0x55:
            return SensorResponse(
                device_id=0,
                raw_data=b'',
                matrix=np.zeros((FrameConfig.MATRIX_COLS, FrameConfig.MATRIX_ROWS), dtype=np.uint8),
                timestamp=timestamp,
                valid=False
            )
        
        # 命令字检查
        if data[2] != Command.RESPONSE:
            return SensorResponse(
                device_id=0,
                raw_data=b'',
                matrix=np.zeros((FrameConfig.MATRIX_COLS, FrameConfig.MATRIX_ROWS), dtype=np.uint8),
                timestamp=timestamp,
                valid=False
            )
        
        device_id = data[3]
        data_len = data[4]
        
        # 数据长度检查
        if data_len != FrameConfig.MATRIX_DATA_LEN:
            return SensorResponse(
                device_id=device_id,
                raw_data=b'',
                matrix=np.zeros((FrameConfig.MATRIX_COLS, FrameConfig.MATRIX_ROWS), dtype=np.uint8),
                timestamp=timestamp,
                valid=False
            )
        
        # 校验和验证
        checksum_recv = data[77]
        checksum_calc = self._calculate_checksum(data[:77])
        
        if checksum_recv != checksum_calc:
            return SensorResponse(
                device_id=device_id,
                raw_data=b'',
                matrix=np.zeros((FrameConfig.MATRIX_COLS, FrameConfig.MATRIX_ROWS), dtype=np.uint8),
                timestamp=timestamp,
                valid=False
            )
        
        # 提取矩阵数据
        matrix_data = data[5:77]
        
        # 转换为 12x6 矩阵 (列优先)
        matrix = np.zeros((FrameConfig.MATRIX_COLS, FrameConfig.MATRIX_ROWS), dtype=np.uint8)
        for col in range(FrameConfig.MATRIX_COLS):
            for row in range(FrameConfig.MATRIX_ROWS):
                matrix[col, row] = matrix_data[col * FrameConfig.MATRIX_ROWS + row]
        
        return SensorResponse(
            device_id=device_id,
            raw_data=bytes(matrix_data),
            matrix=matrix,
            timestamp=timestamp,
            valid=True
        )
    
    def read_matrix(self, device_id: int = 0x01) -> SensorResponse:
        """
        读取传感器矩阵数据 (同步方式)
        
        Args:
            device_id: 目标设备ID，默认 0x01
            
        Returns:
            SensorResponse 对象
            
        Raises:
            RuntimeError: 未连接设备
        """
        if not self.is_connected:
            raise RuntimeError("传感器未连接，请先调用 connect()")
        
        with self._lock:
            # 清空接收缓冲区
            self._serial.reset_input_buffer()
            
            # 发送请求帧
            request = self._build_request_frame(device_id)
            self._serial.write(request)
            
            # 接收响应
            response_data = self._serial.read(FrameConfig.RESPONSE_LEN)
            
            return self._parse_response(response_data)
    
    def read_matrix_batch(self, device_ids: List[int]) -> List[SensorResponse]:
        """
        批量读取多个设备的数据
        
        Args:
            device_ids: 设备ID列表
            
        Returns:
            SensorResponse 列表
        """
        responses = []
        for device_id in device_ids:
            response = self.read_matrix(device_id)
            responses.append(response)
            time.sleep(0.01)  # 设备间延迟
        return responses
    
    def start_streaming(
        self, 
        device_id: int = 0x01, 
        callback: Optional[Callable[[SensorResponse], None]] = None,
        interval: float = 0.05
    ) -> None:
        """
        启动数据流 (异步连续读取)
        
        Args:
            device_id: 目标设备ID
            callback: 数据回调函数，接收 SensorResponse 参数
            interval: 读取间隔(秒)，默认 0.05 (20Hz)
        """
        if self._is_streaming:
            return
            
        self._is_streaming = True
        
        def stream_loop():
            while self._is_streaming:
                try:
                    response = self.read_matrix(device_id)
                    if callback:
                        callback(response)
                except Exception as e:
                    print(f"流读取错误: {e}")
                time.sleep(interval)
        
        self._stream_thread = threading.Thread(target=stream_loop, daemon=True)
        self._stream_thread.start()
    
    def stop_streaming(self) -> None:
        """停止数据流"""
        self._is_streaming = False
        if self._stream_thread is not None:
            self._stream_thread.join(timeout=1.0)
            self._stream_thread = None
    
    def __enter__(self):
        """上下文管理器入口"""
        self.connect()
        return self
    
    def __exit__(self, exc_type, exc_val, exc_tb):
        """上下文管理器出口"""
        self.disconnect()
        return False


class MatrixVisualizer:
    """
    矩阵数据可视化工具
    
    提供控制台和图形界面的数据显示功能。
    """
    
    @staticmethod
    def print_matrix(matrix: np.ndarray, title: str = "Matrix Data") -> None:
        """
        在控制台打印矩阵
        
        Args:
            matrix: 12x6 NumPy 矩阵
            title: 标题文本
        """
        print(f"\n{'=' * 50}")
        print(f" {title}")
        print('=' * 50)
        
        # 打印列标题
        print("     ", end="")
        for row in range(FrameConfig.MATRIX_ROWS):
            print(f"  R{row:d}  ", end="")
        print()
        print("     " + "-" * 42)
        
        # 打印数据
        for col in range(FrameConfig.MATRIX_COLS):
            print(f"C{col:02d} |", end="")
            for row in range(FrameConfig.MATRIX_ROWS):
                print(f" {matrix[col, row]:3d} ", end="")
            print("|")
        
        print("     " + "-" * 42)
    
    @staticmethod
    def print_heatmap_ascii(matrix: np.ndarray) -> None:
        """
        用 ASCII 字符打印热力图
        
        Args:
            matrix: 12x6 NumPy 矩阵
        """
        # 灰度字符映射 (从亮到暗)
        chars = " ░▒▓█"
        
        print("\n热力图:")
        for col in range(FrameConfig.MATRIX_COLS):
            line = ""
            for row in range(FrameConfig.MATRIX_ROWS):
                val = matrix[col, row]
                idx = min(int(val / 255 * (len(chars) - 1)), len(chars) - 1)
                line += chars[idx] * 2
            print(f"  {line}")
    
    @staticmethod
    def create_matplotlib_visualizer():
        """
        创建 Matplotlib 实时可视化器
        
        Returns:
            更新函数和关闭函数的元组
        """
        try:
            import matplotlib.pyplot as plt
            from matplotlib.colors import LinearSegmentedColormap
            
            plt.ion()  # 交互模式
            # 调整图形尺寸：12列×6行，应该是竖着的长方形
            # 矩阵 shape=(12, 6)，在 imshow 中 Y轴对应12（高度），X轴对应6（宽度）
            fig, ax = plt.subplots(figsize=(6, 12))
            
            # 自定义颜色映射
            colors = ['#000033', '#0033AA', '#00AAFF', '#FFFF00', '#FF5500', '#FF0000']
            cmap = LinearSegmentedColormap.from_list('sensor', colors, N=256)
            
            # 初始化热力图
            # 矩阵 shape=(12列, 6行)，只翻转左右
            # imshow 中第一个维度(12)对应Y轴（高度），第二个维度(6)对应X轴（宽度）
            matrix = np.zeros((FrameConfig.MATRIX_COLS, FrameConfig.MATRIX_ROWS))
            # 只翻转左右（fliplr），不翻转上下
            matrix_flipped = np.fliplr(matrix)
            im = ax.imshow(matrix_flipped, cmap=cmap, vmin=0, vmax=255, aspect='equal', origin='lower')
            plt.colorbar(im, ax=ax, label='Pressure')
            
            ax.set_xlabel('Row (6)')
            ax.set_ylabel('Column (12)')
            ax.set_title('Matrix Sensor Real-time Data')
            
            # 设置刻度（翻转后的顺序）
            ax.set_xticks(range(FrameConfig.MATRIX_ROWS))
            ax.set_yticks(range(FrameConfig.MATRIX_COLS))
            ax.set_xticklabels([f'R{i}' for i in reversed(range(FrameConfig.MATRIX_ROWS))])
            ax.set_yticklabels([f'C{i}' for i in range(FrameConfig.MATRIX_COLS)])
            
            plt.tight_layout()
            
            def update(response: SensorResponse):
                """更新热力图"""
                if response.valid:
                    # 只翻转左右，shape=(12列, 6行)
                    matrix_flipped = np.fliplr(response.matrix)
                    im.set_data(matrix_flipped)
                    ax.set_title(f'Device {response.device_id:#04x} - {time.strftime("%H:%M:%S")}')
                    fig.canvas.draw()
                    fig.canvas.flush_events()
            
            def close():
                """关闭窗口"""
                plt.close(fig)
            
            return update, close
            
        except ImportError:
            print("警告: 未安装 matplotlib，无法使用图形可视化")
            return None, None
    
    @staticmethod
    def create_multi_sensor_visualizer(num_sensors: int = 5):
        """
        创建多传感器 Matplotlib 实时可视化器
        
        Args:
            num_sensors: 传感器数量，默认5个
            
        Returns:
            更新函数和关闭函数的元组
        """
        try:
            import matplotlib.pyplot as plt
            from matplotlib.colors import LinearSegmentedColormap
            
            plt.ion()  # 交互模式
            
            # 创建1行5列的子图布局
            fig, axes = plt.subplots(1, num_sensors, figsize=(4 * num_sensors, 8))
            if num_sensors == 1:
                axes = [axes]
            
            # 自定义颜色映射
            colors = ['#000033', '#0033AA', '#00AAFF', '#FFFF00', '#FF5500', '#FF0000']
            cmap = LinearSegmentedColormap.from_list('sensor', colors, N=256)
            
            # 初始化每个传感器的热力图
            images = []
            for i, ax in enumerate(axes):
                matrix = np.zeros((FrameConfig.MATRIX_COLS, FrameConfig.MATRIX_ROWS))
                matrix_flipped = np.fliplr(matrix)
                im = ax.imshow(matrix_flipped, cmap=cmap, vmin=0, vmax=255, aspect='equal', origin='lower')
                
                ax.set_xlabel('Row')
                ax.set_ylabel('Column')
                ax.set_title(f'Sensor {i+1} (ID=0x{i+1:02X})')
                
                # 设置刻度
                ax.set_xticks(range(FrameConfig.MATRIX_ROWS))
                ax.set_yticks(range(FrameConfig.MATRIX_COLS))
                ax.set_xticklabels([f'{i}' for i in reversed(range(FrameConfig.MATRIX_ROWS))])
                ax.set_yticklabels([f'{i}' for i in range(FrameConfig.MATRIX_COLS)])
                
                images.append(im)
            
            # 添加一个公共的colorbar
            fig.colorbar(images[0], ax=axes, label='Pressure', shrink=0.6)
            
            # 添加主标题和统计信息文本
            title = fig.suptitle('Multi-Sensor Real-time Data', fontsize=14)
            
            plt.tight_layout()
            plt.subplots_adjust(top=0.9)
            
            def update(sensor_index: int, response: SensorResponse):
                """
                更新指定传感器的热力图
                
                Args:
                    sensor_index: 传感器索引 (0-4)
                    response: 传感器响应数据
                """
                if sensor_index < len(images) and response.valid:
                    matrix_flipped = np.fliplr(response.matrix)
                    images[sensor_index].set_data(matrix_flipped)
                    axes[sensor_index].set_title(
                        f'Sensor {sensor_index+1} (ID=0x{response.device_id:02X})\n'
                        f'Max:{response.matrix.max()} Avg:{response.matrix.mean():.1f}'
                    )
            
            def refresh():
                """刷新显示"""
                fig.canvas.draw()
                fig.canvas.flush_events()
            
            def update_title(text: str):
                """更新主标题"""
                title.set_text(text)
            
            def close():
                """关闭窗口"""
                plt.close(fig)
            
            return update, refresh, update_title, close
            
        except ImportError:
            print("警告: 未安装 matplotlib，无法使用图形可视化")
            return None, None, None, None


def demo_console():
    """控制台演示程序 - 30Hz持续读取"""
    print("=" * 60)
    print(" 矩阵传感器 SDK 演示程序 (30Hz)")
    print("=" * 60)
    
    # 使用固定端口
    port = 'COM7'
    print(f"使用端口: {port}")
    print("开始以30Hz频率持续读取，按 Ctrl+C 退出...")
    
    # 创建传感器实例
    with MatrixSensor(port) as sensor:
        print("传感器已连接")
        
        read_count = 0
        try:
            while True:
                response = sensor.read_matrix(device_id=0x01)
                read_count += 1
                
                if response.valid:
                    # 每30次读取显示一次详细信息（约每秒一次）
                    if read_count % 30 == 0:
                        MatrixVisualizer.print_matrix(response.matrix, f"读取 #{read_count}")
                        MatrixVisualizer.print_heatmap_ascii(response.matrix)
                        
                        print(f"\n统计信息:")
                        print(f"  设备ID: {response.device_id:#04x}")
                        print(f"  最大值: {response.matrix.max()}")
                        print(f"  最小值: {response.matrix.min()}")
                        print(f"  平均值: {response.matrix.mean():.2f}")
                        print(f"  读取次数: {read_count}")
                    else:
                        # 其他时候只显示简要信息
                        print(f"[#{read_count}] 设备ID: {response.device_id:#04x}, "
                              f"最大值: {response.matrix.max()}, "
                              f"平均值: {response.matrix.mean():.2f}")
                else:
                    print(f"[#{read_count}] 读取失败")
                
                # 30Hz = 1/30 秒间隔
                time.sleep(1.0 / 30.0)
        except KeyboardInterrupt:
            print(f"\n已停止，总共读取 {read_count} 次")
    
    print("\n演示完成")


def demo_realtime():
    """实时可视化演示程序"""
    import sys
    
    # 使用固定端口
    port = 'COM7'
    print(f"使用端口: {port}")
    
    update_func, close_func = MatrixVisualizer.create_matplotlib_visualizer()
    
    if update_func is None:
        print("matplotlib 不可用，退回控制台模式")
        demo_console()
        return
    
    with MatrixSensor(port) as sensor:
        print("开始实时显示 (30Hz)，按 Ctrl+C 退出...")
        
        read_count = 0
        try:
            while True:
                response = sensor.read_matrix(device_id=0x03)
                read_count += 1
                update_func(response)
                # 30Hz = 1/30 秒间隔
                time.sleep(1.0 / 30.0)
        except KeyboardInterrupt:
            print(f"\n已停止，总共读取 {read_count} 次")
        finally:
            close_func()


def demo_multi_sensor():
    """
    多传感器实时可视化演示程序
    
    5个传感器轮流读取，无间隔，并同时显示5个热力图。
    每个传感器响应时间约2ms，5个传感器一轮约10ms，理论帧率约100Hz。
    """
    import sys
    
    # 使用固定端口
    port = 'COM7'
    print(f"使用端口: {port}")
    
    # 5个传感器的设备ID
    sensor_ids = [0x01, 0x02, 0x03, 0x04, 0x05]
    num_sensors = len(sensor_ids)
    
    print(f"传感器ID列表: {[f'0x{id:02X}' for id in sensor_ids]}")
    
    # 创建多传感器可视化器
    result = MatrixVisualizer.create_multi_sensor_visualizer(num_sensors)
    
    if result[0] is None:
        print("matplotlib 不可用，退回控制台模式")
        demo_console()
        return
    
    update_func, refresh_func, update_title_func, close_func = result
    
    with MatrixSensor(port) as sensor:
        print("传感器已连接")
        print(f"开始轮流读取 {num_sensors} 个传感器 (无间隔)，按 Ctrl+C 退出...")
        
        round_count = 0  # 轮次计数
        total_read_count = 0  # 总读取次数
        success_count = 0  # 成功次数
        start_time = time.time()
        
        try:
            while True:
                round_start = time.time()
                
                # 轮流读取每个传感器，无间隔
                for i, device_id in enumerate(sensor_ids):
                    response = sensor.read_matrix(device_id=device_id)
                    total_read_count += 1
                    
                    if response.valid:
                        success_count += 1
                        update_func(i, response)
                    # 无间隔，直接读取下一个传感器
                
                round_count += 1
                round_time = (time.time() - round_start) * 1000  # 毫秒
                
                # 计算帧率
                elapsed = time.time() - start_time
                fps = round_count / elapsed if elapsed > 0 else 0
                success_rate = (success_count / total_read_count * 100) if total_read_count > 0 else 0
                
                # 更新主标题显示统计信息
                update_title_func(
                    f'Multi-Sensor Real-time Data | '
                    f'Round: {round_count} | FPS: {fps:.1f} | '
                    f'Round Time: {round_time:.1f}ms | '
                    f'Success: {success_rate:.1f}%'
                )
                
                # 刷新显示
                refresh_func()
                
        except KeyboardInterrupt:
            elapsed = time.time() - start_time
            fps = round_count / elapsed if elapsed > 0 else 0
            print(f"\n已停止")
            print(f"  总轮次: {round_count}")
            print(f"  总读取次数: {total_read_count}")
            print(f"  成功次数: {success_count}")
            print(f"  成功率: {success_count / total_read_count * 100:.1f}%")
            print(f"  运行时间: {elapsed:.2f}s")
            print(f"  平均帧率: {fps:.1f} FPS")
        finally:
            close_func()


if __name__ == "__main__":
    import sys
    
    if len(sys.argv) > 1:
        if sys.argv[1] == "--realtime":
            demo_realtime()
        elif sys.argv[1] == "--multi":
            demo_multi_sensor()
        else:
            print("用法:")
            print("  python sensor_sdk.py           # 控制台模式 (单传感器)")
            print("  python sensor_sdk.py --realtime # 实时可视化 (单传感器)")
            print("  python sensor_sdk.py --multi    # 多传感器模式 (5个传感器)")
    else:
        demo_console()
