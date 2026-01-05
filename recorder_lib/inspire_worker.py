"""
Inspire Hand recording worker that captures finger angles during teleoperation.
"""

import time
import threading
from typing import List, Dict, TYPE_CHECKING

if TYPE_CHECKING:
    from inspire_hand import InspireHand


class InspireRecorder:
    """
    Records Inspire Hand finger angles during teleoperation.
    
    Runs in a background thread at 60Hz, capturing:
    - Monotonic timestamp
    - 6 DOF angles (little, ring, middle, index, thumb_bend, thumb_rotate)
    """
    
    def __init__(self, hand: 'InspireHand', label: str, shared_state: dict = None):
        """
        Initialize Inspire Hand recorder.
        
        Args:
            hand: InspireHand instance (already connected)
            label: 'left' or 'right'
            shared_state: Dict to share commanded angles from teleop loop (avoids concurrent Modbus reads)
        """
        self.hand = hand
        self.label = label
        self.shared_state = shared_state or {}
        
        self._recording = False
        self._thread = None
        self._data_buffer: List[Dict] = []
        self._lock = threading.Lock()
        
        self.rate_hz = 60  # Target recording rate
    
    def start_recording(self):
        """Start recording in background thread."""
        if self._recording:
            return
        
        print(f"🎬 Starting Inspire {self.label} recording at {self.rate_hz} Hz")
        
        self._recording = True
        self._data_buffer.clear()
        self._thread = threading.Thread(target=self._record_loop, daemon=True)
        self._thread.start()
    
    def stop_recording(self) -> List[Dict]:
        """
        Stop recording and return collected data.
        
        Returns:
            List of data dicts with keys: 't_mono', 'angles'
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
        
        print(f"📊 Inspire {self.label}: Recorded {len(data)} samples")
        return data
    
    def _record_loop(self):
        """Background thread that samples hand state at target rate."""
        dt = 1.0 / self.rate_hz
        
        while self._recording:
            loop_start = time.time()
            
            try:
                # Capture timestamp
                t_mono = time.monotonic()
                
                # Read commanded angles from shared state (avoid concurrent Modbus access that causes segfaults)
                angles = self.shared_state.get('last_commanded')
                if not angles or len(angles) != 6:
                    # Nothing commanded yet; skip
                    time.sleep(dt)
                    continue
                
                # Store sample
                sample = {
                    't_mono': t_mono,
                    'angles': list(angles),  # 6 DOF
                }
                
                with self._lock:
                    self._data_buffer.append(sample)
                
            except Exception as e:
                print(f"⚠️ Inspire {self.label} recording error: {e}")
            
            # Rate limiting
            elapsed = time.time() - loop_start
            sleep_time = dt - elapsed
            if sleep_time > 0:
                time.sleep(sleep_time)
    
    def get_buffer_size(self) -> int:
        """Get current number of samples in buffer."""
        with self._lock:
            return len(self._data_buffer)


