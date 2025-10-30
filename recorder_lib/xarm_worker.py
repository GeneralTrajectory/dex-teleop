"""
xArm recording worker that captures joint positions during teleoperation.
"""

import time
import threading
from typing import List, Dict, TYPE_CHECKING

if TYPE_CHECKING:
    import sys
    from pathlib import Path
    sys.path.insert(0, str(Path(__file__).parent.parent))
    from vive_teleop_xarm import ViveToXArmMapper


class XArmRecorder:
    """
    Records xArm joint positions and TCP poses during teleoperation.
    
    Runs in a background thread at 100Hz, capturing:
    - Monotonic timestamp
    - Joint angles (7 DOF)
    - TCP pose (x, y, z, roll, pitch, yaw)
    """
    
    def __init__(self, mapper: 'ViveToXArmMapper', label: str, io_client=None):
        """
        Initialize xArm recorder.
        
        Args:
            mapper: ViveToXArmMapper instance (already running teleoperation)
            label: 'left' or 'right'
            io_client: Optional XArmIOClient for reading real joint positions
        """
        self.mapper = mapper
        self.label = label
        self.io_client = io_client  # XArmIOClient or None
        
        self._recording = False
        self._thread = None
        self._data_buffer: List[Dict] = []
        self._lock = threading.Lock()
        
        self.rate_hz = 100  # Target recording rate
    
    def start_recording(self):
        """Start recording in background thread."""
        if self._recording:
            return
        
        print(f"🎬 Starting xArm {self.label} recording at {self.rate_hz} Hz")
        
        self._recording = True
        self._data_buffer.clear()
        self._thread = threading.Thread(target=self._record_loop, daemon=True)
        self._thread.start()
    
    def stop_recording(self) -> List[Dict]:
        """
        Stop recording and return collected data.
        
        Returns:
            List of data dicts with keys: 't_mono', 'joint_pos', 'tcp_pose'
        """
        if not self._recording:
            return []
        
        self._recording = False
        if self._thread:
            self._thread.join(timeout=2.0)
        
        # Return copy of buffer
        with self._lock:
            data = list(self._data_buffer)
            self._data_buffer.clear()
        
        print(f"📊 xArm {self.label}: Recorded {len(data)} samples")
        return data
    
    def _record_loop(self):
        """Background thread that samples arm state at target rate."""
        dt = 1.0 / self.rate_hz
        
        while self._recording:
            loop_start = time.time()
            
            try:
                # Timestamp
                t_mono = time.monotonic()
                
                # If we have an I/O client, use real joint/TCP data
                if self.io_client:
                    io_data = self.io_client.get_latest()
                    if io_data:
                        joint_angles = io_data['joints']
                        tcp_pose = io_data['tcp_pose']
                    else:
                        # No data from I/O process yet; skip
                        time.sleep(dt)
                        continue
                else:
                    # Fallback: use commanded pose from mapper (no joint data)
                    pose = getattr(self.mapper, '_last_valid_pose', None)
                    if not pose:
                        # Nothing commanded yet; skip
                        time.sleep(dt)
                        continue
                    
                    tcp_pose = [
                        float(pose['x']), float(pose['y']), float(pose['z']),
                        float(pose['roll']), float(pose['pitch']), float(pose['yaw'])
                    ]
                    joint_angles = [0.0] * 7  # Placeholder
                
                sample = {
                    't_mono': t_mono,
                    'joint_pos': joint_angles,
                    'tcp_pose': tcp_pose,
                }
                
                with self._lock:
                    self._data_buffer.append(sample)
                
            except Exception as e:
                print(f"⚠️ xArm {self.label} recording error: {e}")
            
            # Rate limiting
            elapsed = time.time() - loop_start
            sleep_time = dt - elapsed
            if sleep_time > 0:
                time.sleep(sleep_time)
    
    def get_buffer_size(self) -> int:
        """Get current number of samples in buffer."""
        with self._lock:
            return len(self._data_buffer)


