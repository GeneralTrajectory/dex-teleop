"""
xArm I/O proxy that isolates xArm SDK calls in a separate process.

Main process interacts with a lightweight XArmIOClient that provides joint/TCP data.
This avoids segfaults in the main recorder process if the vendor SDK crashes.
"""

import multiprocessing as mp
import time
from typing import Optional, Dict, List
import sys


class XArmIOClient:
    """
    Lightweight client that reads xArm state from a multiprocessing.Queue.
    Provides methods compatible with recorder expectations.
    """
    def __init__(self, arm_ip: str, queue: mp.Queue):
        self._arm_ip = arm_ip
        self._queue = queue
        self._last_data: Optional[Dict] = None
        self._last_update = 0.0

    def get_latest(self) -> Optional[Dict]:
        """
        Non-blocking fetch of latest xArm state from proxy process.
        
        Returns:
            Dict with keys: 'joints' (7 DOF), 'tcp_pose' (6 DOF), 'timestamp'
            Returns None if no data available yet
        """
        try:
            # Drain queue to get most recent data
            while True:
                self._last_data = self._queue.get_nowait()
                self._last_update = time.time()
        except Exception:
            # Empty queue
            pass
        return self._last_data


def _xarm_io_process(queue: mp.Queue, arm_ip: str, poll_hz: float = 100.0):
    """
    Child process: initializes xArm SDK and pushes joint/TCP state to queue.
    
    Args:
        queue: Multiprocessing queue for IPC
        arm_ip: xArm IP address
        poll_hz: Polling rate in Hz
    """
    # Import xArm SDK in child process
    sys.path.insert(0, '/home/joshua/Documents/Sources/Papers/Cursor/AnyDexGrasp')
    
    try:
        from xarm.wrapper import XArmAPI
        
        # Connect to xArm
        arm = XArmAPI(arm_ip, is_radian=False)
        arm.motion_enable(True)
        arm.set_mode(1)  # Servo mode for streaming
        arm.set_state(0)
        
        dt = 1.0 / max(1.0, poll_hz)
        
        while True:
            loop_start = time.time()
            
            try:
                # Read joint angles
                code_joint, joint_angles = arm.get_servo_angle(is_radian=False)
                if code_joint != 0 or not joint_angles:
                    joint_angles = [0.0] * 7
                
                # Read TCP pose
                code_tcp, tcp_pose = arm.get_position(is_radian=False)
                if code_tcp != 0 or not tcp_pose:
                    tcp_pose = [0.0] * 6
                
                # Package data
                data = {
                    'joints': list(joint_angles[:7]),
                    'tcp_pose': list(tcp_pose[:6]),
                    'timestamp': time.monotonic()
                }
                
                # Put in queue (drop oldest if full to prevent blocking)
                try:
                    queue.put(data, block=False)
                except Exception:
                    # Queue full, drop oldest
                    try:
                        queue.get_nowait()
                        queue.put(data, block=False)
                    except Exception:
                        pass
                
            except Exception as e:
                # Continue on errors (SDK may have transient failures)
                pass
            
            # Rate limiting
            elapsed = time.time() - loop_start
            sleep_time = dt - elapsed
            if sleep_time > 0:
                time.sleep(sleep_time)
                
    except Exception as e:
        print(f"❌ xArm I/O process error ({arm_ip}): {e}", file=sys.stderr)


def start_xarm_io_proxy(arm_ip: str, poll_hz: float = 100.0) -> XArmIOClient:
    """
    Spawn an xArm I/O process and return a client connected to it.
    
    Args:
        arm_ip: xArm IP address
        poll_hz: Polling rate in Hz
        
    Returns:
        XArmIOClient instance
    """
    queue: mp.Queue = mp.Queue(maxsize=5)
    proc = mp.Process(target=_xarm_io_process, args=(queue, arm_ip, poll_hz), daemon=True)
    proc.start()
    return XArmIOClient(arm_ip, queue)




