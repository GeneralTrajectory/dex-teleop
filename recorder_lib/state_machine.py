"""
Recorder orchestrator with state machine for coordinating teleoperation and recording.
"""

import os
import sys
import time
from enum import Enum
from pathlib import Path
from typing import Dict, Tuple, Optional

# Add parent path for imports
sys.path.insert(0, str(Path(__file__).parent.parent))

# Optional: Add inspire_hands path from environment
inspire_path = os.environ.get('INSPIRE_HANDS_PATH', '')
if inspire_path and inspire_path not in sys.path:
    sys.path.insert(0, inspire_path)

from .pedal import RecorderPedal
from .xarm_worker import XArmRecorder
from .inspire_worker import InspireRecorder
from .writer import HDF5Writer

# Lazy imports (avoid requiring hardware dependencies for basic operations)
def _import_vive():
    from vive_teleop_xarm import ViveToXArmMapper, ViveTrackerModule
    return ViveToXArmMapper, ViveTrackerModule

def _import_quest():
    from quest_inspire_teleop import QuestReceiverThread, HandTrackingMapper, HandSender, AngleEMAFilter
    return QuestReceiverThread, HandTrackingMapper, HandSender, AngleEMAFilter

def _import_inspire():
    from inspire_hand import InspireHand
    return InspireHand


class RecorderState(Enum):
    """Recording state machine states."""
    IDLE = "idle"
    HOMING = "homing"
    RECORDING = "recording"
    STOPPING = "stopping"
    SAVING = "saving"
    DISCARDING = "discarding"
    ERROR = "error"


class RecorderOrchestrator:
    """
    Orchestrates bimanual teleoperation with synchronized recording.
    
    Manages state machine, health checks, homing, and coordinates
    all recording workers.
    """
    
    HOME_POSE = [275, 0, 670, 180, -1, 0]  # Standard home pose
    
    def __init__(self, ephemeral: bool = False, subject: Optional[str] = None,
                 task: Optional[str] = None, notes: Optional[str] = None,
                 arms: str = 'both',
                 speed_scale: float = 1.0,
                 record_hands: bool = True,
                 skip_home: bool = False):
        """
        Initialize the recorder orchestrator.
        
        Args:
            ephemeral: If True, discard data on stop instead of saving
            subject: Subject ID
            task: Task name
            notes: Session notes
            arms: Which arm(s) to use ('left', 'right', or 'both')
            speed_scale: Teleoperation speed multiplier (default: 1.0, e.g., 0.5 = half speed)
            record_hands: If True, record Inspire hand data (default: True)
            skip_home: If True, skip moving to home position and use current position (default: False)
        """
        self.ephemeral = ephemeral
        self.subject = subject
        self.task = task
        self.notes = notes
        self.arms = arms  # 'left', 'right', or 'both'
        self.speed_scale = speed_scale
        self.record_hands = record_hands
        self.skip_home = skip_home
        
        # Determine active arms
        self.active_arms = []
        if arms == 'both':
            self.active_arms = ['left', 'right']
        else:
            self.active_arms = [arms]
        
        self.state = RecorderState.IDLE
        self.pedal = None
        
        # Devices
        self.xarm_mappers: Dict[str, ViveToXArmMapper] = {}
        self.inspire_hands: Dict[str, InspireHand] = {}
        self.quest_receiver: Optional[QuestReceiverThread] = None
        
        # Recording workers
        self.xarm_recorders: Dict[str, XArmRecorder] = {}
        self.inspire_recorders: Dict[str, InspireRecorder] = {}
        self.writer: Optional[HDF5Writer] = None
        
        # I/O proxies (for safe concurrent SDK access)
        self.xarm_io_clients: Dict[str, 'XArmIOClient'] = {}
        
        # Stats
        self.record_start_time = None
        self.trial_id = None
        
        # Teleoperation components (initialized when recording starts)
        self.hand_senders = None
        self.ema_filters = None
        self.last_inspire_sent = {'left': None, 'right': None}
    
    def run(self):
        """Main control loop."""
        print("="*60)
        print("🎬 Teleoperation Recorder")
        if self.ephemeral:
            print("⚠️  EPHEMERAL MODE - Data will NOT be saved")
        print(f"🤖 Arms: {self.arms.upper()}")
        print("="*60)
        print()
        
        # Initialize devices
        try:
            self._initialize_devices()
        except Exception as e:
            print(f"❌ Initialization failed: {e}")
            import traceback
            traceback.print_exc()
            return 1
        
        # Teleoperation components will be initialized when recording starts
        self.hand_senders = None
        self.ema_filters = None
        
        # Set up pedal
        self.pedal = RecorderPedal(on_toggle_callback=self._on_pedal_press)
        self.pedal.start()
        
        print("\n✅ System ready")
        print(f"   State: {self.state.value.upper()}")
        print(f"   Press 'b' (pedal) to start recording")
        print(f"   Press Ctrl+C to exit")
        print()
        
        # Main loop
        last_status_time = time.time()
        last_sent = {arm: None for arm in self.active_arms}
        
        try:
            while True:
                loop_start = time.time()
                
                # If recording, run teleoperation control loops
                if self.state == RecorderState.RECORDING:
                    # Run xArm teleoperation
                    self._run_xarm_teleop_step()
                    
                    # Run Inspire teleoperation (if enabled)
                    if self.record_hands:
                        self._run_inspire_teleop_step(last_sent)
                    
                    # Display status periodically
                    if time.time() - last_status_time >= 1.0:
                        self._display_recording_status()
                        last_status_time = time.time()
                else:
                    # Idle, just sleep
                    time.sleep(0.1)
                
                # Rate limiting for main loop
                # Use TELEOP_RATE_HZ if set, otherwise default to 100Hz (standard for xArm teleop)
                target_rate = int(os.environ.get('TELEOP_RATE_HZ', '100'))
                if self.state == RecorderState.RECORDING:
                    elapsed = time.time() - loop_start
                    sleep_time = (1.0 / target_rate) - elapsed
                    if sleep_time > 0:
                        time.sleep(sleep_time)
        
        except KeyboardInterrupt:
            print("\n\n⚠️ Interrupted by user (Ctrl+C)")
        
        finally:
            self._shutdown()
        
        return 0
    
    def _initialize_devices(self):
        """Initialize all devices (xArm, Inspire, Quest, Vive)."""
        print("🔧 Initializing devices...")
        
        # Lazy imports
        ViveToXArmMapper, ViveTrackerModule = _import_vive()
        from .tracker_proxy import start_tracker_proxy
        from .xarm_io_proxy import start_xarm_io_proxy
        QuestReceiverThread, HandTrackingMapper, HandSender, AngleEMAFilter = _import_quest()
        InspireHand = _import_inspire()
        
        # Get environment variables
        left_ip = os.environ.get('XARM_IP_LEFT', '192.168.1.111')
        right_ip = os.environ.get('XARM_IP_RIGHT', '192.168.1.214')
        
        # Initialize Vive Tracker system
        print(f"\n📡 Initializing Vive Tracker system for {self.arms} arm(s)...")
        use_proxy = os.environ.get('RECORDER_USE_TRACKER_PROXY', '1').strip().lower() in ('1','true','yes','y')
        trackers = {}
        tracker_list = []
        if not use_proxy:
            vive = ViveTrackerModule()
            vive.print_discovered_objects()
            trackers = vive.return_selected_devices("tracker")
            tracker_list = list(trackers.values())
            print(f"✅ Found {len(tracker_list)} trackers")
        else:
            print("🔌 Using tracker proxy process (isolates SDK crashes)")
        
        # Tracker assignment by serial (configurable via environment)
        RIGHT_TRACKER_SERIAL = os.environ.get('VIVE_TRACKER_RIGHT', None)
        LEFT_TRACKER_SERIAL = os.environ.get('VIVE_TRACKER_LEFT', None)
        trackers_by_serial = {t.get_serial(): t for t in tracker_list}
        
        # Initialize xArm mappers
        print(f"\n🤖 Initializing xArm teleoperation for {self.arms} arm(s)...")
        
        # Left arm (if active)
        if 'left' in self.active_arms:
            if not use_proxy:
                if LEFT_TRACKER_SERIAL in trackers_by_serial:
                    left_tracker = trackers_by_serial[LEFT_TRACKER_SERIAL]
                else:
                    left_tracker = tracker_list[0]
            else:
                left_tracker = start_tracker_proxy(LEFT_TRACKER_SERIAL)
            
            self.xarm_mappers['left'] = ViveToXArmMapper(
                xarm_ip=left_ip,
                tracker=left_tracker,
                arm_label="left",
                position_scale=self.speed_scale,
                rotation_scale=self.speed_scale
            )
            print("✅ Left xArm mapper initialized")
            # Prime controller readiness once (safe, no motion)
            try:
                ok_ready = self.xarm_mappers['left'].adapter._ensure_ready()
                ok_wait = self.xarm_mappers['left'].adapter._wait_until_ready(timeout_s=3.0)
                if ok_ready and ok_wait:
                    _, st = self.xarm_mappers['left'].adapter.arm.get_state()
                    _, err = self.xarm_mappers['left'].adapter.arm.get_err_warn_code()
                    print(f"   → Left arm post-prime: state={st}, err={err}")
            except Exception:
                pass
            
            # Start I/O proxy for reading real joint positions (optional, crash-isolated)
            use_io_proxy = os.environ.get('RECORDER_USE_XARM_IO_PROXY', '1').strip().lower() in ('1','true','yes','y')
            if use_io_proxy:
                print(f"   🔌 Starting xArm I/O proxy for left (isolated joint reads)...")
                self.xarm_io_clients['left'] = start_xarm_io_proxy(left_ip, poll_hz=100.0)
                print(f"   ✅ xArm I/O proxy started for left")
        
        # Right arm (if active)
        if 'right' in self.active_arms:
            if not use_proxy:
                if RIGHT_TRACKER_SERIAL in trackers_by_serial:
                    right_tracker = trackers_by_serial[RIGHT_TRACKER_SERIAL]
                else:
                    # Use second tracker if both arms, otherwise first
                    tracker_idx = 1 if self.arms == 'both' else 0
                    right_tracker = tracker_list[tracker_idx] if tracker_idx < len(tracker_list) else tracker_list[0]
            else:
                right_tracker = start_tracker_proxy(RIGHT_TRACKER_SERIAL)
            
            self.xarm_mappers['right'] = ViveToXArmMapper(
                xarm_ip=right_ip,
                tracker=right_tracker,
                arm_label="right",
                position_scale=self.speed_scale,
                rotation_scale=self.speed_scale
            )
            print("✅ Right xArm mapper initialized")
            # Prime controller readiness once (safe, no motion)
            try:
                ok_ready = self.xarm_mappers['right'].adapter._ensure_ready()
                ok_wait = self.xarm_mappers['right'].adapter._wait_until_ready(timeout_s=3.0)
                if ok_ready and ok_wait:
                    _, st = self.xarm_mappers['right'].adapter.arm.get_state()
                    _, err = self.xarm_mappers['right'].adapter.arm.get_err_warn_code()
                    print(f"   → Right arm post-prime: state={st}, err={err}")
            except Exception:
                pass
            
            # Start I/O proxy for reading real joint positions (optional, crash-isolated)
            use_io_proxy = os.environ.get('RECORDER_USE_XARM_IO_PROXY', '1').strip().lower() in ('1','true','yes','y')
            if use_io_proxy:
                print(f"   🔌 Starting xArm I/O proxy for right (isolated joint reads)...")
                self.xarm_io_clients['right'] = start_xarm_io_proxy(right_ip, poll_hz=100.0)
                print(f"   ✅ xArm I/O proxy started for right")
        
        if self.record_hands:
            # Initialize Quest receiver
            print("\n📡 Starting Quest hand receiver...")
            self.quest_receiver = QuestReceiverThread(port=9000)
            self.quest_receiver.start()
            print("✅ Quest receiver started")
            
            # Initialize Inspire Hands
            print(f"\n🖐️ Connecting to Inspire Hand(s) for {self.arms} arm(s)...")
            
            if 'left' in self.active_arms:
                self.inspire_hands['left'] = InspireHand(port='/dev/ttyUSB1', slave_id=1, debug=False)
                self.inspire_hands['left'].open()
                print("✅ Connected to left Inspire Hand")
            
            if 'right' in self.active_arms:
                self.inspire_hands['right'] = InspireHand(port='/dev/ttyUSB0', slave_id=1, debug=False)
                self.inspire_hands['right'].open()
                print("✅ Connected to right Inspire Hand")
            
            # Open all fingers
            for hand in self.inspire_hands.values():
                hand.open_all_fingers()
                hand.set_all_finger_speeds(1000)
        
        print("\n✅ All devices initialized")
    
    def _on_pedal_press(self):
        """Handle pedal press (toggle recording)."""
        if self.state == RecorderState.IDLE:
            # Start recording
            self._transition_to_recording()
        elif self.state == RecorderState.RECORDING:
            # Stop recording
            self._transition_to_stop()
        elif self.state in (RecorderState.STOPPING, RecorderState.SAVING, 
                           RecorderState.DISCARDING, RecorderState.HOMING):
            # Ignore pedal during state transitions (prevent double-trigger)
            print(f"⚠️ Pedal pressed during {self.state.value} (ignored)")
        else:
            print(f"⚠️ Pedal pressed in {self.state.value} state (ignored)")
    
    def _transition_to_recording(self):
        """Transition from IDLE to RECORDING."""
        print("\n" + "="*60)
        print("🎬 Starting recording sequence...")
        print("="*60)
        
        # Health checks
        self.state = RecorderState.HOMING
        print(f"\n[{self.state.value.upper()}] Running health checks...")
        healthy, reason = self._check_health()
        if not healthy:
            print(f"❌ Health check failed: {reason}")
            self.state = RecorderState.ERROR
            print(f"   Returning to IDLE. Fix issues and try again.")
            self.state = RecorderState.IDLE
            return
        print("✅ Health checks passed")
        
        # Home arms
        if not self.skip_home:
            print(f"\n[{self.state.value.upper()}] Homing arms...")
            if not self._home_arms():
                print("❌ Homing failed")
                self.state = RecorderState.ERROR
                print("   Returning to IDLE")
                self.state = RecorderState.IDLE
                return
            print("✅ Arms homed")
        else:
            print(f"\n⚠️  Skipping home move - will calibrate from current arm position")
            print("   Make sure arms are in desired starting positions!")
        
        # Calibrate trackers
        print(f"\n[{self.state.value.upper()}] Calibrating trackers (2.5s wait)...")
        print("   Hold BOTH trackers steady!")
        time.sleep(2.5)
        
        # Safe calibration: avoid vendor SDK calls here (defer tracker capture)
        for label, mapper in self.xarm_mappers.items():
            print("\n🎯 Calibrating home poses (safe mode)...")
            print("   Hold tracker steady in your starting position!")
            time.sleep(0.1)
            
            # Defer tracker home capture to first teleop frame to avoid SDK timing issues
            mapper.tracker_home = None
            print("✅ Will capture tracker home on first teleop frame (lazy)")
            
            # Determine arm home pose
            if self.skip_home:
                # Query current position as home
                current_tcp_pose = mapper.adapter._get_live_tcp_pose()
                if current_tcp_pose is None:
                    print(f"❌ Failed to query {label} arm current position")
                    self.state = RecorderState.ERROR
                    self.state = RecorderState.IDLE
                    return
                home = current_tcp_pose
                print(f"   Using current position as home: [{home[0]:.1f}, {home[1]:.1f}, {home[2]:.1f}] mm")
            else:
                # Use known HOME_POSE as arm home (do not query SDK)
                home = self.HOME_POSE
                
            mapper.arm_home_pose = {
                'x': float(home[0]), 'y': float(home[1]), 'z': float(home[2]),
                'roll': float(home[3]), 'pitch': float(home[4]), 'yaw': float(home[5])
            }
            # Build base_T_tcp_home from HOME_POSE
            try:
                import numpy as np
                from scipy.spatial.transform import Rotation as R
                t_home = np.array([home[0]/1000.0, home[1]/1000.0, home[2]/1000.0], dtype=np.float32)
                R_home = R.from_euler('xyz', [home[3], home[4], home[5]], degrees=True).as_matrix().astype(np.float32)
                mapper.base_T_tcp_home = np.eye(4, dtype=np.float32)
                mapper.base_T_tcp_home[:3, :3] = R_home
                mapper.base_T_tcp_home[:3, 3] = t_home
                print(f"✅ {label} calibration complete (safe)")
            except Exception as e:
                print(f"❌ Failed to compute home transform: {e}")
                self.state = RecorderState.ERROR
                print("   Returning to IDLE")
                self.state = RecorderState.IDLE
                return
            
            # Enable servo mode for real-time streaming (wrapped, may be deferred)
            try:
                mapper.adapter.arm.set_mode(1)
                mapper.adapter.arm.set_state(0)
                print("✅ Servo mode enabled for real-time control")
            except Exception as e:
                print(f"⚠️ Failed to enable servo mode: {e}")
        
        print("✅ Trackers calibrated")
        
        # Initialize writer
        print(f"\n[{self.state.value.upper()}] Initializing data writer...")
        output_dir = Path(__file__).parent.parent / 'data'
        self.writer = HDF5Writer(
            output_dir=str(output_dir),
            subject=self.subject,
            task=self.task,
            notes=self.notes,
            ephemeral=self.ephemeral,
            active_arms=self.active_arms,
            include_inspire=self.record_hands,
            include_xarm=True
        )
        
        config = {
            'xarm_left_ip': os.environ.get('XARM_IP_LEFT', '192.168.1.111'),
            'xarm_right_ip': os.environ.get('XARM_IP_RIGHT', '192.168.1.214'),
            'inspire_left_port': '/dev/ttyUSB1',
            'inspire_right_port': '/dev/ttyUSB0',
        }
        
        self.trial_id = self.writer.start(config=config)
        print(f"✅ Writer initialized (trial_id={self.trial_id})")
        
        # Initialize Inspire Hand senders (for teleoperation) if recording hands
        if self.record_hands:
            print(f"\n[RECORDING] Initializing Inspire Hand senders...")
            _, _, HandSender, AngleEMAFilter = _import_quest()
            
            self.hand_senders = {}
            for arm in self.active_arms:
                if arm in self.inspire_hands:
                    self.hand_senders[arm] = HandSender(self.inspire_hands[arm])
                    print(f"✅ {arm} hand sender initialized")
            for arm, sender in self.hand_senders.items():
                sender.start()
            
            # Initialize EMA filters
            ema_slow = 0.5
            ema_bypass = 8
            self.ema_filters = {arm: AngleEMAFilter(alpha_slow=ema_slow, bypass_threshold=ema_bypass)
                                for arm in self.active_arms}
        
        # Start recording workers
        print(f"\n[RECORDING] Starting workers...")
        for label, mapper in self.xarm_mappers.items():
            # Pass I/O client if available for real joint data
            io_client = self.xarm_io_clients.get(label)
            recorder = XArmRecorder(mapper, label, io_client=io_client)
            recorder.start_recording()
            self.xarm_recorders[label] = recorder
        
        # Create shared state dicts for each hand to pass commanded angles
        self.inspire_shared_state = {arm: {'last_commanded': None} for arm in self.active_arms} if self.record_hands else {}
        
        if self.record_hands:
            for label, hand in self.inspire_hands.items():
                recorder = InspireRecorder(hand, label, self.inspire_shared_state[label])
                recorder.start_recording()
                self.inspire_recorders[label] = recorder
        
        self.state = RecorderState.RECORDING
        self.record_start_time = time.time()
        
        print("\n" + "="*60)
        print("🔴 RECORDING IN PROGRESS")
        print("="*60)
        print(f"   Trial ID: {self.trial_id}")
        print(f"   Subject: {self.subject}")
        print(f"   Task: {self.task}")
        if self.ephemeral:
            print("   ⚠️  EPHEMERAL MODE - will discard on stop")
        print(f"   Press 'b' (pedal) to stop recording")
        print()
    
    def _transition_to_stop(self):
        """Transition from RECORDING to STOPPING/SAVING."""
        print("\n" + "="*60)
        print("⏹️  Stopping recording...")
        print("="*60)
        
        self.state = RecorderState.STOPPING
        
        # Stop hand senders
        if self.hand_senders:
            for sender in self.hand_senders.values():
                sender.stop()
            self.hand_senders = None
        
        # Stop all workers and collect data
        print(f"\n[{self.state.value.upper()}] Stopping workers...")
        
        for label, recorder in self.xarm_recorders.items():
            data = recorder.stop_recording()
            self.writer.append_xarm(label, data)
        
        for label, recorder in self.inspire_recorders.items():
            data = recorder.stop_recording()
            self.writer.append_inspire(label, data)
        
        # Calculate stats
        duration = time.time() - self.record_start_time if self.record_start_time else 0
        
        print(f"\n📊 Recording statistics:")
        print(f"   Duration: {duration:.1f}s")
        
        # Finalize or discard
        if self.ephemeral:
            self.state = RecorderState.DISCARDING
            print(f"\n[{self.state.value.upper()}] Discarding data...")
            self.writer.discard()
        else:
            self.state = RecorderState.SAVING
            print(f"\n[{self.state.value.upper()}] Saving data...")
            final_path = self.writer.finalize()
            if final_path:
                print(f"✅ Recording saved: {final_path}")
        
        # Return to idle
        self.state = RecorderState.IDLE
        self.writer = None
        self.xarm_recorders.clear()
        self.inspire_recorders.clear()
        
        print(f"\n[{self.state.value.upper()}] Ready for next recording")
        print(f"   Press 'b' (pedal) to start recording")
        print()
    
    def _check_health(self) -> Tuple[bool, str]:
        """
        Run pre-flight health checks.
        
        Returns:
            (healthy, reason): True if all checks pass, error message if not
        """
        # STRICTLY NO SDK CALLS HERE (avoid segfaults from vendor libs)
        # Validate only that we created devices during initialization.
        
        # xArm/Vive presence checks (no API calls)
        for arm_label in self.active_arms:
            if arm_label not in self.xarm_mappers:
                return False, f"xArm {arm_label} not initialized"
            mapper = self.xarm_mappers[arm_label]
            if mapper.tracker is None:
                return False, f"Vive tracker {arm_label} not initialized"
        
        # Inspire presence checks (no API calls) only if recording hands
        if self.record_hands:
            for hand_label in self.active_arms:
                if hand_label not in self.inspire_hands:
                    return False, f"Inspire {hand_label} not initialized"
                hand = self.inspire_hands[hand_label]
                if getattr(hand, 'is_connected', True) is False:
                    return False, f"Inspire {hand_label} not connected"
            
            # Quest receiver presence check (no socket calls)
            if len(self.inspire_hands) > 0 and (self.quest_receiver is None or not getattr(self.quest_receiver, '_running', True)):
                return False, "Quest receiver not active"
        
        return True, "OK"
    
    def _home_arms(self) -> bool:
        """
        Move arms to HOME_POSE and verify.
        
        Returns:
            True if homing successful
        """
        for label, mapper in self.xarm_mappers.items():
            print(f"   Homing {label} arm...")
            arm = mapper.adapter.arm
            
            arm.set_mode(0)  # Position mode
            arm.set_state(0)  # Ready
            
            ret = arm.set_position(*self.HOME_POSE, speed=200, wait=True, timeout=10)
            if ret != 0:
                print(f"   ❌ {label} arm homing failed (code={ret})")
                return False
            
            print(f"   ✅ {label} arm at home")
        
        # Open all Inspire fingers (if enabled)
        if self.record_hands:
            for label, hand in self.inspire_hands.items():
                hand.open_all_fingers()
        
        time.sleep(1.0)  # Settle
        return True
    
    def _run_xarm_teleop_step(self):
        """Run one step of xArm teleoperation (called at ~60Hz during recording)."""
        rate_hz = 100  # xArm servo mode rate
        # Scale speed proportionally with speed_scale
        speed_mm_s = int(100 * self.speed_scale)
        
        for label, mapper in self.xarm_mappers.items():
            try:
                # Get current tracker pose (guarded)
                try:
                    current_tracker_T = mapper.tracker.get_T()
                except Exception:
                    continue
                if current_tracker_T is None:
                    continue
                
                # Lazy-capture tracker home on first valid frame
                if mapper.tracker_home is None:
                    mapper.tracker_home = current_tracker_T.copy()
                    # Skip movement on the exact anchoring frame
                    mapper._last_valid_pose = None
                    continue
                
                # Compute target pose
                target_pose = mapper.compute_target_pose(current_tracker_T)
                
                # Workspace check
                in_workspace, ws_reason = mapper.is_pose_within_workspace(target_pose)
                if not in_workspace:
                    # Outside workspace - stop arm
                    if not mapper._was_outside_workspace:
                        try:
                            mapper.adapter.arm.set_mode(0)
                            mapper.adapter.arm.set_state(4)  # Stop
                        except:
                            pass
                    mapper._was_outside_workspace = True
                    continue
                
                # Optional IK reachability check (can crash vendor SDK); default: skip
                skip_ik = os.environ.get('XARM_SKIP_IK', '1').strip().lower() in ('1','true','yes','y')
                if not skip_ik:
                    is_reachable, ik_reason = mapper.is_pose_reachable(target_pose)
                    if not is_reachable:
                        continue
                
                # Handle re-engagement if needed
                pose_to_send = target_pose
                if mapper._was_outside_workspace:
                    if mapper._reengagement_counter == 0:
                        # Restart servo mode
                        try:
                            mapper.adapter.arm.set_mode(0)
                            mapper.adapter.arm.set_state(0)
                            time.sleep(0.05)
                            mapper.adapter.arm.set_mode(1)
                            mapper.adapter.arm.set_state(0)
                        except:
                            pass
                    
                    if mapper._reengagement_counter < mapper._reengagement_steps:
                        pose_to_send = mapper.compute_reengagement_pose(target_pose)
                        mapper._reengagement_counter += 1
                    else:
                        mapper._was_outside_workspace = False
                
                # Optionally disable xArm streaming to avoid SDK segfaults (diagnostic safe mode)
                disable_stream = os.environ.get('RECORDER_DISABLE_XARM_STREAM', '0').strip().lower() in ('1','true','yes','y')
                if not disable_stream:
                    # Send streaming command
                    mapper.send_pose_streaming(pose_to_send, speed_mm_s)
                else:
                    # Skip actual robot command; still update last_valid_pose for logging/recording
                    pass
                mapper._last_valid_pose = pose_to_send
                
            except Exception as e:
                print(f"\n⚠️ xArm {label} teleop error: {e}")
    
    def _run_inspire_teleop_step(self, last_sent: Dict):
        """Run one step of Inspire Hand teleoperation."""
        _, HandTrackingMapper, _, _ = _import_quest()
        QUANT = 4
        
        for hand_label in self.active_arms:
            latest = self.quest_receiver.get_latest(hand_label)
            if latest is None and last_sent.get(hand_label) is None:
                continue
            
            # Map to angles
            if latest is not None:
                mapped = HandTrackingMapper.map_hand_data(latest)
            else:
                mapped = last_sent[hand_label]
            
            # EMA smoothing
            if self.ema_filters and hand_label in self.ema_filters:
                mapped = self.ema_filters[hand_label].apply(mapped)
            
            # Quantize
            to_send = mapped
            if QUANT > 1:
                to_send = [int(round(v / QUANT) * QUANT) for v in to_send]
            
            # Send via async sender
            if self.hand_senders and hand_label in self.hand_senders:
                self.hand_senders[hand_label].send(to_send)
            
            # Update shared state for recorder (avoids concurrent Modbus reads)
            if hasattr(self, 'inspire_shared_state') and hand_label in self.inspire_shared_state:
                self.inspire_shared_state[hand_label]['last_commanded'] = to_send
            
            last_sent[hand_label] = to_send
    
    def _display_recording_status(self):
        """Display recording status (called periodically during recording)."""
        if not self.record_start_time:
            return
        
        elapsed = time.time() - self.record_start_time
        
        # Get buffer sizes (only active arms)
        xarm_samples = {label: rec.get_buffer_size() 
                       for label, rec in self.xarm_recorders.items() if label in self.active_arms}
        inspire_samples = {}
        if self.record_hands:
            inspire_samples = {label: rec.get_buffer_size() 
                              for label, rec in self.inspire_recorders.items() if label in self.active_arms}
        
        # Calculate expected vs actual samples
        xarm_expected = int(elapsed * 100)  # 100 Hz
        inspire_expected = int(elapsed * 60)  # 60 Hz
        
        # Print compact status
        status = f"\r🔴 {elapsed:.1f}s | "
        if 'left' in self.active_arms:
            status += f"xArm L:{xarm_samples.get('left', 0)}/{xarm_expected} "
        if 'right' in self.active_arms:
            status += f"xArm R:{xarm_samples.get('right', 0)}/{xarm_expected} "
        if self.record_hands:
            status += "| "
            if 'left' in self.active_arms:
                status += f"Inspire L:{inspire_samples.get('left', 0)}/{inspire_expected} "
            if 'right' in self.active_arms:
                status += f"Inspire R:{inspire_samples.get('right', 0)}/{inspire_expected}"
        
        print(status, end='', flush=True)
    
    def _shutdown(self):
        """Clean shutdown of all devices."""
        print("\n\n🛑 Shutting down...")
        
        # Stop any active recording
        if self.state == RecorderState.RECORDING:
            print("   Stopping active recording...")
            self._transition_to_stop()
        
        # Stop pedal
        if self.pedal:
            self.pedal.stop()
        
        # Disconnect arms (keep them in current position)
        for label, mapper in self.xarm_mappers.items():
            try:
                mapper.shutdown()
            except:
                pass
        
        # Open Inspire hands and disconnect
        for label, hand in self.inspire_hands.items():
            try:
                hand.open_all_fingers()
                time.sleep(0.5)
                hand.close()
            except:
                pass
        
        # Stop Quest receiver
        if self.quest_receiver:
            self.quest_receiver.stop()
        
        print("✅ Shutdown complete")

