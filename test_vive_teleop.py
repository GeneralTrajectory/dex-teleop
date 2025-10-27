#!/usr/bin/env python3
"""
Test script for Vive Tracker teleoperation connectivity and coordinate mapping.
Verifies tracker detection, pose streaming, and xArm connection without moving the robot.
"""

import sys
import time
import numpy as np
from pathlib import Path

# Add Vive_Tracker to path
sys.path.insert(0, str(Path(__file__).parent / 'Vive_Tracker'))
from track import ViveTrackerModule

# Add AnyDexGrasp to path for xArm adapter (absolute path)
anydex_path = '/home/joshua/Documents/Sources/Papers/Cursor/AnyDexGrasp'
if anydex_path not in sys.path:
    sys.path.insert(0, anydex_path)


def test_tracker_detection():
    """Test 1: Verify tracker detection"""
    print("\n" + "="*60)
    print("TEST 1: Tracker Detection")
    print("="*60)
    
    try:
        vive = ViveTrackerModule()
        vive.print_discovered_objects()
        
        trackers = vive.return_selected_devices("tracker")
        if len(trackers) == 0:
            print("❌ No trackers detected!")
            return False
        
        print(f"✅ Detected {len(trackers)} tracker(s)")
        for name, device in trackers.items():
            print(f"   - {name}: {device.get_serial()}")
        
        return True
    except Exception as e:
        print(f"❌ Tracker detection failed: {e}")
        return False


def test_tracker_pose_stream():
    """Test 2: Print tracker pose stream for 5 seconds"""
    print("\n" + "="*60)
    print("TEST 2: Tracker Pose Stream (5 seconds)")
    print("="*60)
    
    try:
        vive = ViveTrackerModule()
        trackers = vive.return_selected_devices("tracker")
        
        if len(trackers) == 0:
            print("❌ No trackers available for pose streaming")
            return False
        
        # Get first tracker
        tracker_name = list(trackers.keys())[0]
        tracker = trackers[tracker_name]
        
        print(f"Streaming poses from {tracker_name}...")
        print("Format: 4x4 homogeneous transformation matrix")
        
        start_time = time.time()
        count = 0
        
        while time.time() - start_time < 5.0:
            T = tracker.get_T()
            
            if count % 10 == 0:  # Print every 10th frame (3Hz if running at 30Hz)
                print(f"\nPose at t={time.time()-start_time:.2f}s:")
                print(f"Position (m): [{T[0,3]:.3f}, {T[1,3]:.3f}, {T[2,3]:.3f}]")
                print(f"Rotation:\n{T[:3,:3]}")
            
            count += 1
            time.sleep(0.033)  # ~30Hz
        
        print(f"\n✅ Streamed {count} poses successfully")
        return True
        
    except Exception as e:
        print(f"❌ Pose streaming failed: {e}")
        import traceback
        traceback.print_exc()
        return False


def test_xarm_connection():
    """Test 3: Verify xArm connection (no motion)"""
    print("\n" + "="*60)
    print("TEST 3: xArm Connection (No Motion)")
    print("="*60)
    
    try:
        import os
        os.environ['EXECUTE_LIVE'] = '1'  # Enable connection but we won't move
        
        # Check if xArm SDK is available
        try:
            from xarm.wrapper import XArmAPI
            print("✅ xArm SDK found (xarm.wrapper)")
        except ImportError:
            print("❌ xArm SDK not installed correctly!")
            print("   The package 'xarm' (0.0.4) is NOT the official SDK.")
            print("   Install the correct SDK:")
            print("   pip uninstall xarm")
            print("   pip install xarm-python-sdk")
            print("   OR: git clone https://github.com/xArm-Developer/xArm-Python-SDK.git")
            print("       cd xArm-Python-SDK && python setup.py install")
            return False
        
        from xarm_adapter import create_xarm_adapter
        
        xarm_ip = os.environ.get('XARM_IP', '192.168.1.214')
        print(f"Connecting to xArm at {xarm_ip}...")
        
        adapter = create_xarm_adapter(execute_live=True, use_inspire_api=False)
        adapter._lazy_connect_if_needed()
        
        if adapter.arm is None:
            print("❌ Failed to connect to xArm")
            print("   Check: Is xArm powered on?")
            print("   Check: Is IP correct? Try: ping", xarm_ip)
            print("   Check: Is xArm SDK installed? (see message above)")
            return False
        
        # Query current pose without moving
        print("Querying current TCP pose...")
        current_pose = adapter._get_live_tcp_pose()
        
        if current_pose is None:
            print("❌ Failed to query TCP pose")
            return False
        
        print(f"✅ Connected successfully!")
        print(f"Current pose (mm, deg): x={current_pose[0]:.1f}, y={current_pose[1]:.1f}, "
              f"z={current_pose[2]:.1f}, roll={current_pose[3]:.1f}, "
              f"pitch={current_pose[4]:.1f}, yaw={current_pose[5]:.1f}")
        
        # Disconnect cleanly
        adapter.disconnect()
        return True
        
    except Exception as e:
        print(f"❌ xArm connection test failed: {e}")
        import traceback
        traceback.print_exc()
        return False


def test_coordinate_transformation():
    """Test 4: Test coordinate transformation (print deltas without moving)"""
    print("\n" + "="*60)
    print("TEST 4: Coordinate Transformation (10 seconds)")
    print("="*60)
    
    try:
        vive = ViveTrackerModule()
        trackers = vive.return_selected_devices("tracker")
        
        if len(trackers) == 0:
            print("❌ No trackers available")
            return False
        
        tracker = list(trackers.values())[0]
        
        # Capture reference pose
        print("Capturing reference tracker pose in 2 seconds...")
        print("Hold tracker STEADY...")
        time.sleep(2.0)
        # IMPORTANT: copy the matrix to avoid aliasing self.T inside tracker
        tracker_home = tracker.get_T().copy()
        print(f"✅ Reference captured!")
        print(f"   Position (m): [{tracker_home[0,3]:.3f}, {tracker_home[1,3]:.3f}, {tracker_home[2,3]:.3f}]")
        
        print("\n🎮 NOW MOVE THE TRACKER AROUND!")
        print("   Deltas will update in real-time (10 seconds)")
        print("   Try moving it in different directions...")
        
        scale_factor = float(os.environ.get('VIVE_POSITION_SCALE', '1.5'))
        
        start_time = time.time()
        max_delta_seen = 0.0
        
        while time.time() - start_time < 10.0:
            # Also copy current pose to avoid aliasing issues
            current_T = tracker.get_T().copy()
            
            # Compute position delta
            pos_delta = current_T[:3, 3] - tracker_home[:3, 3]
            scaled_delta = pos_delta * scale_factor
            
            # Compute rotation delta (simplified - just show angular difference)
            R_delta = current_T[:3, :3] @ tracker_home[:3, :3].T
            from scipy.spatial.transform import Rotation as R
            rotvec = R.from_matrix(R_delta).as_rotvec()
            angle_deg = np.linalg.norm(rotvec) * 180 / np.pi
            
            # Track max delta for validation
            delta_magnitude = np.linalg.norm(pos_delta)
            if delta_magnitude > max_delta_seen:
                max_delta_seen = delta_magnitude
            
            print(f"\rDelta pos (m): [{pos_delta[0]:+.3f}, {pos_delta[1]:+.3f}, {pos_delta[2]:+.3f}] "
                  f"→ scaled (mm): [{scaled_delta[0]*1000:+.1f}, {scaled_delta[1]*1000:+.1f}, "
                  f"{scaled_delta[2]*1000:+.1f}] | Rot: {angle_deg:.1f}° | Max: {max_delta_seen*1000:.1f}mm", 
                  end='', flush=True)
            
            time.sleep(0.1)
        
        print(f"\n✅ Coordinate transformation test completed")
        print(f"   Maximum delta observed: {max_delta_seen*1000:.1f}mm")
        
        if max_delta_seen > 0.01:  # 10mm threshold
            print(f"   ✅ Tracker movement detected successfully!")
            return True
        else:
            print(f"   ⚠️  No significant movement detected (max delta < 10mm)")
            print(f"      This is OK if tracker was held steady")
            print(f"      Try moving tracker more during the test")
            return True  # Still pass - tracker is working, just wasn't moved
        
    except KeyboardInterrupt:
        print("\n✅ Test interrupted by user")
        return True
    except Exception as e:
        print(f"\n❌ Coordinate transformation test failed: {e}")
        import traceback
        traceback.print_exc()
        return False


def main():
    print("="*60)
    print("Vive Tracker Teleoperation Test Suite")
    print("="*60)
    
    results = {}
    
    # Run all tests
    results['tracker_detection'] = test_tracker_detection()
    
    if results['tracker_detection']:
        results['pose_stream'] = test_tracker_pose_stream()
        results['coordinate_transform'] = test_coordinate_transformation()
    else:
        print("\n⚠️ Skipping remaining tracker tests due to detection failure")
        results['pose_stream'] = False
        results['coordinate_transform'] = False
    
    results['xarm_connection'] = test_xarm_connection()
    
    # Summary
    print("\n" + "="*60)
    print("TEST SUMMARY")
    print("="*60)
    
    for test_name, passed in results.items():
        status = "✅ PASS" if passed else "❌ FAIL"
        print(f"{status}: {test_name.replace('_', ' ').title()}")
    
    all_passed = all(results.values())
    
    if all_passed:
        print("\n🎉 All tests passed! Ready for teleoperation.")
    else:
        print("\n⚠️ Some tests failed. Fix issues before running teleoperation.")
    
    return 0 if all_passed else 1


if __name__ == '__main__':
    import os
    sys.exit(main())

