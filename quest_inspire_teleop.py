#!/usr/bin/env python3
"""
Meta Quest Hand Tracking → Inspire Hand Teleoperation
Real-time bimanual dexterous hand control using Quest hand tracking.

Usage:
    # Ensure Quest app is running and sending UDP to port 9000
    python quest_inspire_teleop.py
    
    # Stop: Press 'b' (foot pedal) or Ctrl+C
"""

import sys
import time
import os
from pathlib import Path
from typing import Dict, Optional, List
from collections import deque
import threading

# Add inspire_hands to path (absolute path to avoid path resolution issues)
inspire_hands_path = '/home/joshua/Research/inspire_hands'
if inspire_hands_path not in sys.path:
    sys.path.insert(0, inspire_hands_path)
from inspire_hand import InspireHand

# Import Quest receiver and foot pedal from local modules
from quest_hand_receiver import QuestHandReceiver

try:
    import select
    import tty
    import termios
    KEYBOARD_AVAILABLE = True
except ImportError:
    KEYBOARD_AVAILABLE = False


class FootPedalMonitor:
    """Monitor foot pedal (sends 'b' key) for stop signal in a background thread."""
    
    def __init__(self):
        self.stop_requested = False
        self._thread = None
        self._running = False
        self._old_terminal_settings = None
        
        if KEYBOARD_AVAILABLE and sys.stdin.isatty():
            print("🦶 Foot pedal enabled - press 'b' to stop")
        else:
            print("⚠️ Foot pedal unavailable (stdin not a terminal)")
    
    def start(self):
        """Start monitoring foot pedal in background thread."""
        if not KEYBOARD_AVAILABLE or not sys.stdin.isatty():
            return
        
        # Save terminal settings and set to raw mode
        try:
            self._old_terminal_settings = termios.tcgetattr(sys.stdin)
            tty.setraw(sys.stdin.fileno())
        except Exception as e:
            print(f"⚠️ Could not set terminal to raw mode: {e}")
            return
        
        import threading
        self._running = True
        self._thread = threading.Thread(target=self._monitor_loop, daemon=True)
        self._thread.start()
    
    def _monitor_loop(self):
        """Background thread that monitors for 'b' key press."""
        while self._running:
            if select.select([sys.stdin], [], [], 0.1)[0]:
                try:
                    char = sys.stdin.read(1)
                    if char == 'b':
                        print("\n\n🦶 Foot pedal pressed - stopping!")
                        self.stop_requested = True
                        break
                except Exception:
                    pass
    
    def stop(self):
        """Stop monitoring and restore terminal settings."""
        self._running = False
        if self._thread:
            self._thread.join(timeout=1.0)
        
        if self._old_terminal_settings:
            try:
                termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self._old_terminal_settings)
            except Exception:
                pass


class HandTrackingMapper:
    """Maps Quest hand tracking data to Inspire Hand control commands."""
    
    @staticmethod
    def _normalize(value: Optional[float], vmin: float, vmax: float) -> float:
        if value is None:
            return 0.0
        if vmax <= vmin:
            return 0.0
        x = (value - vmin) / (vmax - vmin)
        if x < 0.0:
            x = 0.0
        elif x > 1.0:
            x = 1.0
        return x

    @staticmethod
    def curl_to_angle(curl: Optional[float]) -> int:
        """
        Convert Quest finger curl to Inspire Hand angle.
        
        Args:
            curl: Curl metric 0.0 (open) to ~3.0 (fully curled), or None
            
        Returns:
            Inspire angle 0 (closed) to 1000 (open)
        """
        # Use calibrated range (defaults derived from user observation)
        curl_min = float(os.environ.get('QUEST_CURL_MIN', '0.2'))
        curl_max = float(os.environ.get('QUEST_CURL_MAX', '1.0'))
        norm = HandTrackingMapper._normalize(curl, curl_min, curl_max)
        # Invert: more curl (1.0) => more closed (0)
        angle = int(1000 - norm * 1000)
        return max(0, min(1000, angle))
    
    @staticmethod
    def flexion_to_angle(flexion_deg: Optional[float]) -> int:
        """
        Convert thumb flexion angle to Inspire Hand thumb bend angle.
        
        Args:
            flexion_deg: Flexion angle ~0° (open) to ~180° (closed), or None
            
        Returns:
            Inspire angle 0 (closed) to 1000 (open)
        """
        # Use calibrated range (defaults derived from user observation)
        fmin = float(os.environ.get('QUEST_TFLEX_MIN', '30.0'))
        fmax = float(os.environ.get('QUEST_TFLEX_MAX', '100.0'))
        norm = HandTrackingMapper._normalize(flexion_deg, fmin, fmax)
        angle = int(1000 - norm * 1000)
        return max(0, min(1000, angle))
    
    @staticmethod
    def opposition_to_angle(opposition_deg: Optional[float]) -> int:
        """
        Convert thumb opposition angle to Inspire Hand thumb rotate angle.
        
        Args:
            opposition_deg: Opposition angle ~90° (opposed) to ~10° (parallel), or None
            
        Returns:
            Inspire angle 0 (parallel) to 1000 (opposed)
        """
        # Use calibrated range (defaults derived from user observation)
        omin = float(os.environ.get('QUEST_TOPP_MIN', '90.0'))
        omax = float(os.environ.get('QUEST_TOPP_MAX', '100.0'))
        norm = HandTrackingMapper._normalize(opposition_deg, omin, omax)
        invert = os.environ.get('QUEST_TOPP_INVERT', '1') not in ('0', 'false', 'False')
        if invert:
            # Increase in opposition angle should reduce Inspire angle (flip direction)
            angle = int((1.0 - norm) * 1000)
        else:
            angle = int(norm * 1000)
        return max(0, min(1000, angle))
    
    @staticmethod
    def map_hand_data(hand_data: Dict) -> list:
        """
        Convert complete hand tracking data to Inspire Hand angles.
        
        Args:
            hand_data: Dict from QuestHandReceiver.receive()
            
        Returns:
            List of 6 angles [little, ring, middle, index, thumb_bend, thumb_rotate]
        """
        curls = hand_data['curls']
        
        angles = [
            HandTrackingMapper.curl_to_angle(curls['Little']),
            HandTrackingMapper.curl_to_angle(curls['Ring']),
            HandTrackingMapper.curl_to_angle(curls['Middle']),
            HandTrackingMapper.curl_to_angle(curls['Index']),
            HandTrackingMapper.flexion_to_angle(hand_data['thumb_flexion']),
            HandTrackingMapper.opposition_to_angle(hand_data['thumb_opposition']),
        ]
        
        return angles


class AngleEMAFilter:
    """Adaptive EMA: bypass for large changes, smooth only micro-jitter."""
    
    def __init__(self, alpha_slow: float = 0.5, bypass_threshold: int = 8):
        self.alpha_slow = alpha_slow  # For small changes (more smoothing)
        self.bypass_threshold = bypass_threshold  # Pass through if delta >= this
        self._last: Optional[List[int]] = None
    
    def apply(self, angles: List[int]) -> List[int]:
        if self._last is None:
            # First sample initializes the filter
            self._last = list(angles)
            return list(angles)
        smoothed: List[int] = []
        for prev, new in zip(self._last, angles):
            delta = abs(new - prev)
            # Bypass filter entirely for large changes (instant response)
            if delta >= self.bypass_threshold:
                val = int(new)
            else:
                # Only smooth micro-jitter
                val = int(self.alpha_slow * new + (1.0 - self.alpha_slow) * prev)
            # Clamp to valid range
            if val < 0:
                val = 0
            elif val > 1000:
                val = 1000
            smoothed.append(val)
        self._last = smoothed
        return smoothed


class AngleMedianFilter:
    """Median filter on 6-angle vectors to reject outliers."""
    def __init__(self, window_size: int = 3):
        self.window_size = max(1, window_size)
        self.buffers: List[deque] = [deque(maxlen=self.window_size) for _ in range(6)]

    def apply(self, angles: List[int]) -> List[int]:
        out: List[int] = []
        for i, val in enumerate(angles):
            self.buffers[i].append(int(val))
            buf = list(self.buffers[i])
            buf.sort()
            mid = buf[len(buf)//2]
            out.append(mid)
        return out

class QuestReceiverThread:
    """Background receiver that continuously keeps the latest data per hand."""
    
    def __init__(self, port: int = 9000):
        self.receiver = QuestHandReceiver(port=port)
        self._lock = threading.Lock()
        self._latest_by_hand: Dict[str, Dict] = {}
        self._running = False
        self._thread: Optional[threading.Thread] = None
    
    def start(self):
        self._running = True
        self._thread = threading.Thread(target=self._loop, daemon=True)
        self._thread.start()
    
    def stop(self):
        self._running = False
        if self._thread:
            self._thread.join(timeout=1.0)
        try:
            self.receiver.close()
        except Exception:
            pass
    
    def _loop(self):
        # Drain UDP as fast as needed; keep only the latest per hand
        while self._running:
            data = self.receiver.receive()
            if data is not None and isinstance(data, dict):
                hand = data.get('hand')
                if hand in ('left', 'right'):
                    with self._lock:
                        self._latest_by_hand[hand] = data
                # Immediately try to fetch more without sleeping to drain bursts
                continue
            # No data available this moment; brief nap to avoid busy spin
            time.sleep(0.002)
    
    def get_latest(self, hand: str) -> Optional[Dict]:
        with self._lock:
            return self._latest_by_hand.get(hand)


class HandSender:
    """Async writer that sends angles to a specific Inspire hand on its own thread.
    This parallelizes serial I/O across left/right ports to reduce end-to-end latency.
    """
    def __init__(self, hand: InspireHand):
        self.hand = hand
        self._latest: Optional[List[int]] = None
        self._lock = threading.Lock()
        self._event = threading.Event()
        self._running = False
        self._thread: Optional[threading.Thread] = None
        self.sent_count = 0

    def start(self):
        self._running = True
        self._thread = threading.Thread(target=self._loop, daemon=True)
        self._thread.start()

    def stop(self):
        self._running = False
        self._event.set()
        if self._thread:
            self._thread.join(timeout=1.0)

    def send(self, angles: List[int]):
        with self._lock:
            self._latest = list(angles)
        self._event.set()

    def _loop(self):
        while self._running:
            # Wait for a new command or timeout to exit responsively
            triggered = self._event.wait(timeout=0.1)
            if not self._running:
                break
            if not triggered:
                continue
            self._event.clear()
            with self._lock:
                payload = self._latest
            if payload is None:
                continue
            try:
                self.hand.modbus.write_multiple_registers(1486, payload)
                self.sent_count += 1
            except Exception as e:
                print(f"\n⚠️ [sender] Command error: {e}")


def run_quest_hand_teleop():
    """Main control loop for Quest hand tracking teleoperation."""

    print("Get into a ready position...")
    time.sleep(6)
    
    print("="*60)
    print("🎮 Quest Hand Tracking → Inspire Hands")
    print("="*60)
    
    # Initialize Quest background receiver
    rx = QuestReceiverThread(port=9000)
    rx.start()
    
    # Initialize Inspire Hands
    hands = {}
    
    try:
        # Left hand (port /dev/ttyUSB1, slave_id 1)
        print("\n🤖 Connecting to left Inspire Hand...")
        hands['left'] = InspireHand(port='/dev/ttyUSB1', slave_id=1, debug=False)
        hands['left'].open()
        print("✅ Connected to left Inspire Hand (/dev/ttyUSB1)")
        
        # Right hand (port /dev/ttyUSB0, slave_id 1)
        print("\n🤖 Connecting to right Inspire Hand...")
        hands['right'] = InspireHand(port='/dev/ttyUSB0', slave_id=1, debug=False)
        hands['right'].open()
        print("✅ Connected to right Inspire Hand (/dev/ttyUSB0)")
        
        # Open all fingers to start
        print("\n🖐️ Opening all fingers...")
        speed_default = int(os.environ.get('INSPIRE_SPEED', '1000'))
        for hand in hands.values():
            hand.open_all_fingers()
            # Balance responsiveness and stability
            hand.set_all_finger_speeds(speed_default)
        time.sleep(0.5)
        
    except Exception as e:
        print(f"❌ Failed to initialize Inspire Hands: {e}")
        return 1
    
    # Start foot pedal monitor
    foot_pedal = FootPedalMonitor()
    foot_pedal.start()

    # Per-hand async senders (parallelize serial I/O)
    senders = {
        'left': HandSender(hands['left']),
        'right': HandSender(hands['right']),
    }
    senders['left'].start()
    senders['right'].start()
    
    # Main control loop
    print("\n✅ Bimanual hand control active at ~30 Hz")
    print("   Press 'b' (foot pedal) or Ctrl+C to stop")
    print("-"*60)
    
    loop_count = 0
    # Tunables via env

    # EMA tunables
    ema_slow = float(os.environ.get('QUEST_EMA_SLOW', '0.5'))   # alpha for small changes
    ema_bypass = int(os.environ.get('QUEST_EMA_BYPASS', '8'))   # bypass threshold (counts)

    # Filters per hand
    ema_filters = {
        'left': AngleEMAFilter(alpha_slow=ema_slow, bypass_threshold=ema_bypass),
        'right': AngleEMAFilter(alpha_slow=ema_slow, bypass_threshold=ema_bypass),
    }
    median_filters = None

    # Last sent angles per hand
    last_sent = {
        'left': None,
        'right': None,
    }
    # Control rate (Hz)
    RATE_HZ = int(os.environ.get('QUEST_RATE_HZ', '60'))
    if RATE_HZ < 10:
        RATE_HZ = 10
    # Optional quantization to suppress tiny changes
    QUANT = int(os.environ.get('QUEST_QUANT', '4'))
    
    # Aggregate status printing once per second
    last_status_time = time.time()
    last_status = {'left': None, 'right': None}

    try:
        while True:
            # Check for foot pedal stop
            if foot_pedal.stop_requested:
                print("\n🛑 Stop requested by foot pedal")
                break
            
            loop_start = time.time()
            
            # Pull latest for both hands and send at fixed rate
            for hand_label in ('left', 'right'):
                latest = rx.get_latest(hand_label)
                if latest is None and last_sent[hand_label] is None:
                    # Nothing to send yet for this hand
                    continue
                # Map to angles (use last sent if no new latest)
                if latest is not None:
                    mapped = HandTrackingMapper.map_hand_data(latest)
                else:
                    mapped = last_sent[hand_label]
                # Optional median filter (disable for max speed)
                if median_filters:
                    mapped = median_filters[hand_label].apply(mapped)
                # EMA smoothing with bypass for large changes
                mapped = ema_filters[hand_label].apply(mapped)
                # Quantize to suppress micro-jitter
                to_send = mapped
                if QUANT > 1:
                    to_send = [int(round(v / QUANT) * QUANT) for v in to_send]
                # Async send to avoid blocking other hand
                senders[hand_label].send(to_send)
                last_sent[hand_label] = to_send
                last_status[hand_label] = latest
            
            loop_count += 1

            # Print compact status once per second
            now = time.time()
            if now - last_status_time >= 1.0:
                left = last_status.get('left')
                right = last_status.get('right')
                parts = []
                if left:
                    lc = left['curls']
                    parts.append(f"[left ] L:{lc['Little']:.1f} R:{lc['Ring']:.1f} M:{lc['Middle']:.1f} I:{lc['Index']:.1f} T:{lc['Thumb']:.1f}")
                if right:
                    rc = right['curls']
                    parts.append(f"[right] L:{rc['Little']:.1f} R:{rc['Ring']:.1f} M:{rc['Middle']:.1f} I:{rc['Index']:.1f} T:{rc['Thumb']:.1f}")
                if parts:
                    print("\r" + "  |  ".join(parts), end='', flush=True)
                last_status_time = now
            
            # Rate limiting
            elapsed = time.time() - loop_start
            sleep_time = (1.0 / RATE_HZ) - elapsed
            if sleep_time > 0:
                time.sleep(sleep_time)
    
    except KeyboardInterrupt:
        print("\n\n⚠️ Stopped by user (Ctrl+C)")
    
    except Exception as e:
        print(f"\n❌ Fatal error: {e}")
        import traceback
        traceback.print_exc()
    
    finally:
        # Clean shutdown
        foot_pedal.stop()
        
        print("\n\n🛑 Shutting down...")
        
        # Open all fingers before disconnect (safe state)
        try:
            for hand in hands.values():
                hand.open_all_fingers()
            time.sleep(0.5)
        except Exception:
            pass
        
        # Disconnect
        for label, hand in hands.items():
            try:
                hand.close()
                print(f"✅ Disconnected from {label} hand")
            except Exception:
                pass
        
        rx.stop()
        # Stop senders
        try:
            senders['left'].stop()
            senders['right'].stop()
        except Exception:
            pass
        
        print(f"\n📊 Session statistics:")
        print(f"   Total frames: {loop_count}")
    
    return 0


def main():
    """Entry point."""
    return run_quest_hand_teleop()


if __name__ == '__main__':
    sys.exit(main())

