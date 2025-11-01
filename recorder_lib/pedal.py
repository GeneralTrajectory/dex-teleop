"""
Recorder pedal with toggle logic and debouncing.
"""

import sys
import time
import threading
import os
from typing import Callable

try:
    import select
    import tty
    import termios
    KEYBOARD_AVAILABLE = True
except ImportError:
    KEYBOARD_AVAILABLE = False


class RecorderPedal:
    """
    Foot pedal monitor with toggle logic and debouncing.
    
    Extends FootPedalMonitor to call a callback on each press instead of
    setting a stop flag. Includes 150ms debouncing to prevent double-hits.
    """
    
    def __init__(self, on_toggle_callback: Callable[[], None]):
        """
        Initialize the recorder pedal.
        
        Args:
            on_toggle_callback: Function to call on each pedal press
        """
        self.on_toggle = on_toggle_callback
        self.last_press_time = 0.0
        # Debounce duration (seconds); can be overridden via env
        # Longer debounce mitigates OS key auto-repeat from foot pedal
        self.debounce_s = float(os.environ.get('RECORDER_PEDAL_DEBOUNCE_S', '0.50'))
        self._thread = None
        self._running = False
        self._old_terminal_settings = None
        
        if KEYBOARD_AVAILABLE and sys.stdin.isatty():
            print("🦶 Foot pedal enabled - press 'b' to toggle recording")
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
        
        self._running = True
        self._thread = threading.Thread(target=self._monitor_loop, daemon=True)
        self._thread.start()
    
    def _monitor_loop(self):
        """Background thread that monitors for 'b' key press with debouncing.
        Also handles Ctrl+C (ETX) by emitting a SIGINT to the main process.
        """
        import os
        import signal
        while self._running:
            if select.select([sys.stdin], [], [], 0.1)[0]:
                try:
                    char = sys.stdin.read(1)
                    # Ctrl+C in raw mode arrives as ETX (\x03)
                    if char == '\x03':
                        try:
                            # Restore terminal ASAP
                            if self._old_terminal_settings:
                                termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self._old_terminal_settings)
                        except Exception:
                            pass
                        # Forward SIGINT to main process
                        os.kill(os.getpid(), signal.SIGINT)
                        return
                    if char == 'b':
                        current_time = time.time()
                        
                        # Check debounce
                        if current_time - self.last_press_time < self.debounce_s:
                            # Too soon, ignore (debounce)
                            continue
                        
                        # Valid press
                        self.last_press_time = current_time
                        print(f"\n🦶 Pedal pressed at t={current_time:.3f}")
                        
                        # Call callback
                        try:
                            self.on_toggle()
                        except Exception as e:
                            print(f"❌ Pedal callback error: {e}")
                            import traceback
                            traceback.print_exc()
                        
                        # Flush any auto-repeated 'b' characters from stdin buffer
                        try:
                            termios.tcflush(sys.stdin, termios.TCIFLUSH)
                        except Exception:
                            pass
                        
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


