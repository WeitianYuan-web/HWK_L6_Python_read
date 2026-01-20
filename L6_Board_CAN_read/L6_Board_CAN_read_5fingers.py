#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
L6 Five-Finger Sensor CAN Data Reader and Visualizer

Read 5 finger sensor matrix data via CAN bus and visualize in real-time.

CAN Frame Protocol:
- CAN ID: 0x3mn, where m=finger_id(0-4), n=frame_index(0-8)
- 9 frames per finger, 8 bytes per frame, 72 bytes total
- Matrix size: 12 columns x 6 rows (column-major order)
"""

import can
import time
import threading
import numpy as np
from typing import Optional, List, Dict
from dataclasses import dataclass, field
from collections import defaultdict


class CANConfig:
    """CAN configuration constants"""
    CHANNEL = 'PCAN_USBBUS1'  # Windows PCAN: 'PCAN_USBBUS1', Linux: 'can0'
    INTERFACE = 'pcan'        # Windows PCAN: 'pcan', Linux: 'socketcan'
    BITRATE = 1000000         # 1Mbps
    
    # Frame configuration
    CAN_ID_BASE = 0x300       # CAN ID base address
    FRAMES_PER_FINGER = 9     # 9 frames per finger
    BYTES_PER_FRAME = 8       # 8 bytes per frame
    TOTAL_BYTES = 72          # 72 bytes per finger
    
    # Matrix configuration
    MATRIX_COLS = 12          # Matrix columns
    MATRIX_ROWS = 6           # Matrix rows
    
    # Finger configuration
    NUM_FINGERS = 5           # Number of fingers
    FINGER_NAMES = ['Thumb', 'Index', 'Middle', 'Ring', 'Pinky']


@dataclass
class FingerSensorData:
    """
    Finger sensor data structure
    
    Attributes:
        finger_id: Finger ID (0-4)
        raw_data: Raw 72-byte data
        matrix: 12x6 NumPy matrix
        timestamp: Data timestamp
        valid: Whether data is complete and valid
        frame_flags: Frame reception flags (9 bits)
    """
    finger_id: int = 0
    raw_data: bytearray = field(default_factory=lambda: bytearray(72))
    matrix: np.ndarray = field(default_factory=lambda: np.zeros((12, 6), dtype=np.uint8))
    timestamp: float = 0.0
    valid: bool = False
    frame_flags: int = 0  # 9-bit flags indicating which frames have been received


class CANFrameParser:
    """
    CAN frame parser
    
    Parses CAN frames, extracts finger ID and frame index, and reassembles data.
    """
    
    @staticmethod
    def parse_can_id(can_id: int) -> tuple:
        """
        Parse CAN ID to extract finger ID and frame index
        
        Args:
            can_id: CAN frame ID (0x3mn format)
            
        Returns:
            (finger_id, frame_index) tuple, returns (-1, -1) if invalid
        """
        # Check if valid sensor frame (0x300-0x348)
        if (can_id & 0xF00) != 0x300:
            return (-1, -1)
        
        # Extract finger ID: second hex digit (m)
        finger_id = (can_id >> 4) & 0x0F
        
        # Extract frame index: last hex digit (n)
        frame_index = can_id & 0x0F
        
        # Validate range
        if finger_id > 4 or frame_index > 8:
            return (-1, -1)
        
        return (finger_id, frame_index)
    
    @staticmethod
    def bytes_to_matrix(data: bytearray) -> np.ndarray:
        """
        Convert 72-byte data to 12x6 matrix (column-major order)
        
        Data layout:
        [C0R0, C0R1, C0R2, C0R3, C0R4, C0R5,   // Column 0
         C1R0, C1R1, C1R2, C1R3, C1R4, C1R5,   // Column 1
         ...
         C11R0, C11R1, C11R2, C11R3, C11R4, C11R5]  // Column 11
        
        Args:
            data: 72-byte raw data
            
        Returns:
            12x6 NumPy matrix (columns, rows)
        """
        matrix = np.zeros((CANConfig.MATRIX_COLS, CANConfig.MATRIX_ROWS), dtype=np.uint8)
        
        for col in range(CANConfig.MATRIX_COLS):
            for row in range(CANConfig.MATRIX_ROWS):
                idx = col * CANConfig.MATRIX_ROWS + row
                if idx < len(data):
                    matrix[col, row] = data[idx]
        
        return matrix


class FiveFingerSensorCAN:
    """
    Five-finger sensor CAN communication class
    
    Manages CAN connection, receives and parses 5 finger sensor data.
    
    Example:
        >>> sensor = FiveFingerSensorCAN()
        >>> sensor.connect()
        >>> sensor.start_receiving()
        >>> data = sensor.get_all_fingers_data()
        >>> sensor.stop_receiving()
        >>> sensor.disconnect()
    """
    
    def __init__(self, channel: str = None, interface: str = None, bitrate: int = None):
        """
        Initialize five-finger sensor CAN instance
        
        Args:
            channel: CAN channel, defaults to CANConfig.CHANNEL
            interface: CAN interface type, defaults to CANConfig.INTERFACE
            bitrate: CAN bitrate, defaults to CANConfig.BITRATE
        """
        self.channel = channel or CANConfig.CHANNEL
        self.interface = interface or CANConfig.INTERFACE
        self.bitrate = bitrate or CANConfig.BITRATE
        
        self.bus: Optional[can.Bus] = None
        self._lock = threading.Lock()
        self._is_receiving = False
        self._receive_thread: Optional[threading.Thread] = None
        
        # Initialize 5 finger data buffers
        self._finger_data: List[FingerSensorData] = [
            FingerSensorData(finger_id=i) for i in range(CANConfig.NUM_FINGERS)
        ]
        
        # Statistics
        self._stats = {
            'total_frames': 0,           # Total CAN frames received
            'finger_complete': [0] * 5,  # Complete count per finger
            'complete_rounds': 0,        # Complete rounds (all 5 fingers)
            'start_time': 0.0
        }
        self._last_round_flags = 0  # Track which fingers completed in current round
    
    @property
    def is_connected(self) -> bool:
        """Check if connected"""
        return self.bus is not None
    
    def connect(self) -> bool:
        """
        Connect to CAN bus
        
        Returns:
            Whether connection succeeded
        """
        if self.is_connected:
            return True
        
        try:
            self.bus = can.interface.Bus(
                channel=self.channel,
                interface=self.interface,
                bitrate=self.bitrate
            )
            print(f"CAN Connected: {self.channel} @ {self.bitrate}bps")
            return True
        except Exception as e:
            print(f"CAN Connection Failed: {e}")
            print("Please check CAN adapter connection and configuration")
            return False
    
    def disconnect(self) -> None:
        """Disconnect from CAN bus"""
        self.stop_receiving()
        if self.bus is not None:
            self.bus.shutdown()
            self.bus = None
            print("CAN Disconnected")
    
    def _process_frame(self, msg: can.Message) -> None:
        """
        Process received CAN frame
        
        Args:
            msg: CAN message
        """
        finger_id, frame_idx = CANFrameParser.parse_can_id(msg.arbitration_id)
        
        if finger_id < 0:
            return  # Invalid frame
        
        self._stats['total_frames'] += 1
        
        with self._lock:
            finger_data = self._finger_data[finger_id]
            
            # Calculate data offset and write
            offset = frame_idx * CANConfig.BYTES_PER_FRAME
            for i, byte in enumerate(msg.data):
                if offset + i < CANConfig.TOTAL_BYTES:
                    finger_data.raw_data[offset + i] = byte
            
            # Set frame reception flag
            finger_data.frame_flags |= (1 << frame_idx)
            finger_data.timestamp = time.time()
            
            # Check if all 9 frames received for this finger
            if finger_data.frame_flags == 0x1FF:  # All 9 bits set = 0x1FF
                finger_data.matrix = CANFrameParser.bytes_to_matrix(finger_data.raw_data)
                finger_data.valid = True
                finger_data.frame_flags = 0  # Reset for next round
                self._stats['finger_complete'][finger_id] += 1
                
                # Track complete rounds (all 5 fingers)
                self._last_round_flags |= (1 << finger_id)
                if self._last_round_flags == 0x1F:  # All 5 fingers complete
                    self._stats['complete_rounds'] += 1
                    self._last_round_flags = 0
    
    def _receive_loop(self) -> None:
        """CAN receive loop"""
        while self._is_receiving:
            try:
                msg = self.bus.recv(timeout=0.001)
                if msg:
                    self._process_frame(msg)
            except Exception as e:
                if self._is_receiving:
                    print(f"CAN Receive Error: {e}")
                break
    
    def start_receiving(self) -> None:
        """Start CAN data reception"""
        if not self.is_connected:
            print("Please connect to CAN bus first")
            return
        
        if self._is_receiving:
            return
        
        self._is_receiving = True
        self._stats['start_time'] = time.time()
        self._receive_thread = threading.Thread(target=self._receive_loop, daemon=True)
        self._receive_thread.start()
        print("Started receiving CAN data...")
    
    def stop_receiving(self) -> None:
        """Stop CAN data reception"""
        self._is_receiving = False
        if self._receive_thread is not None:
            self._receive_thread.join(timeout=1.0)
            self._receive_thread = None
    
    def get_finger_data(self, finger_id: int) -> FingerSensorData:
        """
        Get sensor data for specified finger
        
        Args:
            finger_id: Finger ID (0-4)
            
        Returns:
            FingerSensorData object
        """
        with self._lock:
            return FingerSensorData(
                finger_id=self._finger_data[finger_id].finger_id,
                raw_data=bytearray(self._finger_data[finger_id].raw_data),
                matrix=self._finger_data[finger_id].matrix.copy(),
                timestamp=self._finger_data[finger_id].timestamp,
                valid=self._finger_data[finger_id].valid,
                frame_flags=self._finger_data[finger_id].frame_flags
            )
    
    def get_all_fingers_data(self) -> List[FingerSensorData]:
        """
        Get sensor data for all fingers
        
        Returns:
            List of 5 FingerSensorData objects
        """
        return [self.get_finger_data(i) for i in range(CANConfig.NUM_FINGERS)]
    
    def get_stats(self) -> dict:
        """Get statistics"""
        elapsed = time.time() - self._stats['start_time'] if self._stats['start_time'] > 0 else 0
        total_finger_complete = sum(self._stats['finger_complete'])
        return {
            'total_frames': self._stats['total_frames'],
            'finger_complete': self._stats['finger_complete'].copy(),
            'total_complete': total_finger_complete,
            'complete_rounds': self._stats['complete_rounds'],
            'elapsed_time': elapsed,
            'frame_rate': self._stats['total_frames'] / elapsed if elapsed > 0 else 0,
            'round_rate': self._stats['complete_rounds'] / elapsed if elapsed > 0 else 0
        }
    
    def __enter__(self):
        """Context manager entry"""
        self.connect()
        return self
    
    def __exit__(self, exc_type, exc_val, exc_tb):
        """Context manager exit"""
        self.disconnect()
        return False


class FiveFingerVisualizer:
    """
    Five-finger sensor data visualization tool
    
    Provides console and matplotlib graphical visualization.
    """
    
    @staticmethod
    def print_matrix(matrix: np.ndarray, title: str = "Matrix Data") -> None:
        """
        Print matrix to console
        
        Args:
            matrix: 12x6 NumPy matrix
            title: Title text
        """
        print(f"\n{'=' * 50}")
        print(f" {title}")
        print('=' * 50)
        
        # Print column headers
        print("     ", end="")
        for row in range(CANConfig.MATRIX_ROWS):
            print(f"  R{row:d}  ", end="")
        print()
        print("     " + "-" * 42)
        
        # Print data
        for col in range(CANConfig.MATRIX_COLS):
            print(f"C{col:02d} |", end="")
            for row in range(CANConfig.MATRIX_ROWS):
                print(f" {matrix[col, row]:3d} ", end="")
            print("|")
        
        print("     " + "-" * 42)
    
    @staticmethod
    def print_all_fingers_summary(finger_data_list: List[FingerSensorData]) -> None:
        """
        Print summary of all fingers
        
        Args:
            finger_data_list: List of 5 finger sensor data
        """
        print("\n" + "=" * 70)
        print(" Five-Finger Sensor Data Summary")
        print("=" * 70)
        print(f"{'Finger':<10} {'Status':<10} {'Max':<8} {'Avg':<10} {'Timestamp':<20}")
        print("-" * 70)
        
        for i, data in enumerate(finger_data_list):
            status = "Valid" if data.valid else "Waiting"
            max_val = data.matrix.max() if data.valid else "-"
            avg_val = f"{data.matrix.mean():.1f}" if data.valid else "-"
            timestamp = time.strftime("%H:%M:%S", time.localtime(data.timestamp)) if data.timestamp > 0 else "-"
            
            print(f"{CANConfig.FINGER_NAMES[i]:<10} {status:<10} {str(max_val):<8} {avg_val:<10} {timestamp:<20}")
        
        print("-" * 70)
    
    @staticmethod
    def create_matplotlib_visualizer():
        """
        Create Matplotlib five-finger real-time visualizer
        
        Returns:
            (update_func, refresh_func, update_title_func, close_func) tuple
        """
        try:
            import matplotlib.pyplot as plt
            from matplotlib.colors import LinearSegmentedColormap
            
            plt.ion()  # Interactive mode
            
            # Create 1x5 subplot layout
            fig, axes = plt.subplots(1, CANConfig.NUM_FINGERS, figsize=(4 * CANConfig.NUM_FINGERS, 8))
            
            # Custom colormap (dark blue to red)
            colors = ['#000033', '#0033AA', '#00AAFF', '#FFFF00', '#FF5500', '#FF0000']
            cmap = LinearSegmentedColormap.from_list('sensor', colors, N=256)
            
            # Initialize heatmaps for each finger
            images = []
            for i, ax in enumerate(axes):
                matrix = np.zeros((CANConfig.MATRIX_COLS, CANConfig.MATRIX_ROWS))
                matrix_flipped = np.fliplr(matrix)
                im = ax.imshow(matrix_flipped, cmap=cmap, vmin=0, vmax=255, 
                              aspect='equal', origin='lower')
                
                ax.set_xlabel('Row')
                ax.set_ylabel('Column')
                ax.set_title(f'{CANConfig.FINGER_NAMES[i]} (ID={i})')
                
                # Set ticks
                ax.set_xticks(range(CANConfig.MATRIX_ROWS))
                ax.set_yticks(range(CANConfig.MATRIX_COLS))
                ax.set_xticklabels([f'{j}' for j in reversed(range(CANConfig.MATRIX_ROWS))])
                ax.set_yticklabels([f'{j}' for j in range(CANConfig.MATRIX_COLS)])
                
                images.append(im)
            
            # Add shared colorbar
            fig.colorbar(images[0], ax=axes, label='Pressure', shrink=0.6)
            
            # Add main title
            title = fig.suptitle('Five-Finger Sensor Real-time Data', fontsize=14)
            
            plt.tight_layout()
            plt.subplots_adjust(top=0.9)
            
            def update(finger_index: int, finger_data: FingerSensorData):
                """
                Update specified finger's heatmap
                
                Args:
                    finger_index: Finger index (0-4)
                    finger_data: Finger sensor data
                """
                if finger_index < len(images) and finger_data.valid:
                    matrix_flipped = np.fliplr(finger_data.matrix)
                    images[finger_index].set_data(matrix_flipped)
                    axes[finger_index].set_title(
                        f'{CANConfig.FINGER_NAMES[finger_index]} (ID={finger_index})\n'
                        f'Max:{finger_data.matrix.max()} Avg:{finger_data.matrix.mean():.1f}'
                    )
            
            def refresh():
                """Refresh display"""
                fig.canvas.draw()
                fig.canvas.flush_events()
            
            def update_title(text: str):
                """Update main title"""
                title.set_text(text)
            
            def close():
                """Close window"""
                plt.close(fig)
            
            return update, refresh, update_title, close
            
        except ImportError:
            print("Warning: matplotlib not installed, cannot use graphical visualization")
            print("Please run: pip install matplotlib")
            return None, None, None, None


def demo_console():
    """Console demo program"""
    print("=" * 60)
    print(" Five-Finger Sensor CAN Reader Demo (Console Mode)")
    print("=" * 60)
    
    with FiveFingerSensorCAN() as sensor:
        sensor.start_receiving()
        
        try:
            while True:
                finger_data_list = sensor.get_all_fingers_data()
                FiveFingerVisualizer.print_all_fingers_summary(finger_data_list)
                
                # Print detailed matrix data (once per second)
                for i, data in enumerate(finger_data_list):
                    if data.valid:
                        FiveFingerVisualizer.print_matrix(
                            data.matrix, 
                            f"{CANConfig.FINGER_NAMES[i]} (ID={i})"
                        )
                
                stats = sensor.get_stats()
                print(f"\nStats: Frames={stats['total_frames']}, "
                      f"Rounds={stats['complete_rounds']}, "
                      f"Frame Rate={stats['frame_rate']:.1f}/s, "
                      f"Round Rate={stats['round_rate']:.1f}/s")
                
                time.sleep(1.0)
                
        except KeyboardInterrupt:
            print("\nStopped")
            stats = sensor.get_stats()
            print(f"Final Stats: Frames={stats['total_frames']}, "
                  f"Rounds={stats['complete_rounds']}, "
                  f"Time={stats['elapsed_time']:.2f}s")


def demo_realtime():
    """Real-time visualization demo program"""
    print("=" * 60)
    print(" Five-Finger Sensor CAN Reader Demo (Real-time Mode)")
    print("=" * 60)
    
    # Create visualizer
    result = FiveFingerVisualizer.create_matplotlib_visualizer()
    
    if result[0] is None:
        print("matplotlib unavailable, falling back to console mode")
        demo_console()
        return
    
    update_func, refresh_func, update_title_func, close_func = result
    
    with FiveFingerSensorCAN() as sensor:
        sensor.start_receiving()
        
        print("Starting real-time display, press Ctrl+C to exit...")
        
        try:
            while True:
                # Get all finger data
                finger_data_list = sensor.get_all_fingers_data()
                
                # Update each finger's visualization
                for i, data in enumerate(finger_data_list):
                    update_func(i, data)
                
                # Update title with statistics
                stats = sensor.get_stats()
                valid_count = sum(1 for d in finger_data_list if d.valid)
                update_title_func(
                    f'Five-Finger Sensor Real-time Data | '
                    f'Valid: {valid_count}/{CANConfig.NUM_FINGERS} | '
                    f'Frames: {stats["total_frames"]} | '
                    f'Rounds: {stats["complete_rounds"]} | '
                    f'Rate: {stats["round_rate"]:.1f} rps'
                )
                
                # Refresh display
                refresh_func()
                
                time.sleep(0.033)  # ~30Hz refresh rate
                
        except KeyboardInterrupt:
            print("\nStopped")
            stats = sensor.get_stats()
            print(f"Final Statistics:")
            print(f"  Total Frames: {stats['total_frames']}")
            print(f"  Complete Rounds: {stats['complete_rounds']}")
            print(f"  Per-Finger Complete: {stats['finger_complete']}")
            print(f"  Elapsed Time: {stats['elapsed_time']:.2f}s")
            print(f"  Frame Rate: {stats['frame_rate']:.1f} frames/s")
            print(f"  Round Rate: {stats['round_rate']:.1f} rounds/s")
        finally:
            close_func()


def demo_test_parser():
    """Test CAN frame parser"""
    print("=" * 60)
    print(" CAN Frame Parser Test")
    print("=" * 60)
    
    # Test cases
    test_cases = [
        (0x300, "Thumb, Frame 0"),
        (0x308, "Thumb, Frame 8"),
        (0x315, "Index, Frame 5"),
        (0x320, "Middle, Frame 0"),
        (0x328, "Middle, Frame 8"),
        (0x332, "Ring, Frame 2"),
        (0x348, "Pinky, Frame 8"),
        (0x200, "Invalid (not 0x3xx)"),
        (0x350, "Invalid (finger_id > 4)"),
        (0x309, "Invalid (frame_idx > 8)"),
    ]
    
    print(f"{'CAN ID':<10} {'Result':<25} {'Expected':<25}")
    print("-" * 60)
    
    for can_id, expected in test_cases:
        finger_id, frame_idx = CANFrameParser.parse_can_id(can_id)
        if finger_id >= 0:
            result = f"Finger {finger_id} ({CANConfig.FINGER_NAMES[finger_id]}), Frame {frame_idx}"
        else:
            result = "Invalid frame"
        print(f"0x{can_id:03X}      {result:<25} {expected:<25}")


if __name__ == "__main__":
    import sys
    
    if len(sys.argv) > 1:
        if sys.argv[1] == "--realtime":
            demo_realtime()
        elif sys.argv[1] == "--console":
            demo_console()
        elif sys.argv[1] == "--test":
            demo_test_parser()
        else:
            print("Usage:")
            print("  python L6_Board_CAN_read_5fingers.py            # Real-time visualization (default)")
            print("  python L6_Board_CAN_read_5fingers.py --realtime # Real-time visualization")
            print("  python L6_Board_CAN_read_5fingers.py --console  # Console mode")
            print("  python L6_Board_CAN_read_5fingers.py --test     # Test parser")
    else:
        demo_realtime()  # Default: start real-time visualization
