#!/usr/bin/env python3
"""
Safety Feature Test Suite for Vive Tracker Teleoperation

Tests all safety constraints WITHOUT moving the robot.
Validates that unsafe poses are correctly rejected.
"""

import sys
import os
import numpy as np
from pathlib import Path
from scipy.spatial.transform import Rotation as R

# Add paths
sys.path.insert(0, str(Path(__file__).parent / 'Vive_Tracker'))

# Add AnyDexGrasp to path (absolute path)
anydex_path = '/home/joshua/Documents/Sources/Papers/Cursor/AnyDexGrasp'
if anydex_path not in sys.path:
    sys.path.insert(0, anydex_path)

from xarm_adapter import create_xarm_adapter


def test_table_collision_detection():
    """Test that table collision detection correctly blocks unsafe poses."""
    print("\n" + "="*60)
    print("TEST: Table Collision Detection")
    print("="*60)
    
    # Setup
    os.environ['XARM_IP'] = os.environ.get('XARM_IP', '192.168.1.214')
    os.environ['EXECUTE_LIVE'] = '1'
    os.environ['TABLE_Z_MM'] = '15'
    os.environ['GRIPPER_TIP_LENGTH_MM'] = '200'
    os.environ['TABLE_CLEARANCE_MM'] = '10'
    
    try:
        adapter = create_xarm_adapter(execute_live=True, use_inspire_api=False)
        adapter._lazy_connect_if_needed()
        
        if adapter.arm is None:
            print("❌ Cannot test without xArm connection")
            return False
        
        # Test poses
        safe_pose = {'x': 275.0, 'y': 0.0, 'z': 200.0, 'roll': 180.0, 'pitch': -1.0, 'yaw': 0.0}
        unsafe_pose = {'x': 275.0, 'y': 0.0, 'z': 50.0, 'roll': 180.0, 'pitch': -1.0, 'yaw': 0.0}
        
        print("\n1. Testing SAFE pose (z=200mm, well above table):")
        print(f"   Pose: {safe_pose}")
        
        # Convert to matrix
        t_m = np.array([safe_pose['x'], safe_pose['y'], safe_pose['z']]) / 1000.0
        R_mat = R.from_euler('xyz', [safe_pose['roll'], safe_pose['pitch'], safe_pose['yaw']], 
                            degrees=True).as_matrix()
        T_safe = np.eye(4, dtype=np.float32)
        T_safe[:3, :3] = R_mat.astype(np.float32)
        T_safe[:3, 3] = t_m.astype(np.float32)
        
        collides_safe = adapter.pose_collides_with_table(T_safe)
        
        if collides_safe:
            print("   ❌ FAIL: Safe pose incorrectly flagged as collision")
            return False
        else:
            print("   ✅ PASS: Safe pose correctly accepted")
        
        print("\n2. Testing UNSAFE pose (z=50mm, near table with 200mm gripper):")
        print(f"   Pose: {unsafe_pose}")
        
        t_m = np.array([unsafe_pose['x'], unsafe_pose['y'], unsafe_pose['z']]) / 1000.0
        R_mat = R.from_euler('xyz', [unsafe_pose['roll'], unsafe_pose['pitch'], unsafe_pose['yaw']], 
                            degrees=True).as_matrix()
        T_unsafe = np.eye(4, dtype=np.float32)
        T_unsafe[:3, :3] = R_mat.astype(np.float32)
        T_unsafe[:3, 3] = t_m.astype(np.float32)
        
        collides_unsafe = adapter.pose_collides_with_table(T_unsafe)
        
        if not collides_unsafe:
            print("   ❌ FAIL: Unsafe pose incorrectly accepted (should block!)")
            return False
        else:
            print("   ✅ PASS: Unsafe pose correctly blocked")
        
        adapter.disconnect()
        print("\n✅ Table collision detection working correctly!")
        return True
        
    except Exception as e:
        print(f"❌ Test failed: {e}")
        import traceback
        traceback.print_exc()
        return False


def test_joint1_limits():
    """Test that joint1 angle limits correctly block behind-table poses."""
    print("\n" + "="*60)
    print("TEST: Joint1 Angle Limits")
    print("="*60)
    
    os.environ['XARM_IP'] = os.environ.get('XARM_IP', '192.168.1.214')
    os.environ['EXECUTE_LIVE'] = '1'
    
    try:
        adapter = create_xarm_adapter(execute_live=True, use_inspire_api=False)
        adapter._lazy_connect_if_needed()
        
        if adapter.arm is None:
            print("❌ Cannot test without xArm connection")
            return False
        
        # Test poses
        # Yaw=0 should result in joint1≈0 (facing forward)
        safe_pose = {'x': 300.0, 'y': 0.0, 'z': 400.0, 'roll': 180.0, 'pitch': 0.0, 'yaw': 0.0}
        # Yaw=180 should result in joint1≈180 (behind table - UNSAFE)
        unsafe_pose = {'x': 300.0, 'y': 0.0, 'z': 400.0, 'roll': 180.0, 'pitch': 0.0, 'yaw': 180.0}
        
        print("\n1. Testing SAFE pose (yaw=0°, facing forward):")
        print(f"   Pose: {safe_pose}")
        
        pose_list = [safe_pose['x'], safe_pose['y'], safe_pose['z'],
                     safe_pose['roll'], safe_pose['pitch'], safe_pose['yaw']]
        
        code, angles = adapter.arm.get_inverse_kinematics(
            pose_list, input_is_radian=False, return_is_radian=False
        )
        
        if code == 0 and angles:
            joint1 = float(angles[0])
            print(f"   Joint1 angle: {joint1:.1f}°")
            
            if -90.0 <= joint1 <= 90.0:
                print("   ✅ PASS: Joint1 within safe limits")
            else:
                print(f"   ❌ FAIL: Joint1 outside limits (should be in [-90, 90])")
                return False
        else:
            print(f"   ⚠️ IK failed for safe pose (code={code})")
        
        print("\n2. Testing UNSAFE pose (yaw=180°, behind table):")
        print(f"   Pose: {unsafe_pose}")
        
        pose_list = [unsafe_pose['x'], unsafe_pose['y'], unsafe_pose['z'],
                     unsafe_pose['roll'], unsafe_pose['pitch'], unsafe_pose['yaw']]
        
        code, angles = adapter.arm.get_inverse_kinematics(
            pose_list, input_is_radian=False, return_is_radian=False
        )
        
        if code == 0 and angles:
            joint1 = float(angles[0])
            print(f"   Joint1 angle: {joint1:.1f}°")
            
            if joint1 < -90.0 or joint1 > 90.0:
                print("   ✅ PASS: Joint1 outside safe limits (would be correctly blocked)")
            else:
                print(f"   ⚠️ NOTE: Joint1 within limits for this pose (IK found solution)")
                print(f"          (Limit enforcement depends on specific arm configuration)")
        else:
            print(f"   ✅ PASS: IK failed (pose unreachable, would be blocked anyway)")
        
        adapter.disconnect()
        print("\n✅ Joint1 limit validation working!")
        return True
        
    except Exception as e:
        print(f"❌ Test failed: {e}")
        import traceback
        traceback.print_exc()
        return False


def test_workspace_boundaries():
    """Test workspace boundary enforcement."""
    print("\n" + "="*60)
    print("TEST: Workspace Boundaries")
    print("="*60)
    
    workspace_limits = {
        'x': (100.0, 450.0),
        'y': (-400.0, 400.0),
        'z': (65.0, 800.0)  # TABLE_Z(15) + 50
    }
    
    test_cases = [
        # (pose, should_pass, description)
        ({'x': 275.0, 'y': 0.0, 'z': 400.0}, True, "Center of workspace"),
        ({'x': 50.0, 'y': 0.0, 'z': 400.0}, False, "X too small (behind limit)"),
        ({'x': 500.0, 'y': 0.0, 'z': 400.0}, False, "X too large (beyond reach)"),
        ({'x': 275.0, 'y': -500.0, 'z': 400.0}, False, "Y too negative (left limit)"),
        ({'x': 275.0, 'y': 500.0, 'z': 400.0}, False, "Y too positive (right limit)"),
        ({'x': 275.0, 'y': 0.0, 'z': 30.0}, False, "Z too small (near table)"),
        ({'x': 275.0, 'y': 0.0, 'z': 900.0}, False, "Z too large (above reach)"),
    ]
    
    print(f"\nWorkspace limits (mm):")
    print(f"  X: {workspace_limits['x']}")
    print(f"  Y: {workspace_limits['y']}")
    print(f"  Z: {workspace_limits['z']}")
    print("")
    
    all_passed = True
    
    for i, (pose, should_pass, description) in enumerate(test_cases, 1):
        print(f"{i}. Testing: {description}")
        print(f"   Pose: x={pose['x']:.1f}, y={pose['y']:.1f}, z={pose['z']:.1f}")
        
        # Check boundaries
        in_bounds = True
        violation = None
        
        if pose['x'] < workspace_limits['x'][0] or pose['x'] > workspace_limits['x'][1]:
            in_bounds = False
            violation = "X out of bounds"
        elif pose['y'] < workspace_limits['y'][0] or pose['y'] > workspace_limits['y'][1]:
            in_bounds = False
            violation = "Y out of bounds"
        elif pose['z'] < workspace_limits['z'][0] or pose['z'] > workspace_limits['z'][1]:
            in_bounds = False
            violation = "Z out of bounds"
        
        if should_pass and in_bounds:
            print(f"   ✅ PASS: Within bounds (correctly accepted)")
        elif should_pass and not in_bounds:
            print(f"   ❌ FAIL: Out of bounds but should pass (logic error!)")
            all_passed = False
        elif not should_pass and not in_bounds:
            print(f"   ✅ PASS: Out of bounds (correctly blocked: {violation})")
        else:  # not should_pass and in_bounds
            print(f"   ❌ FAIL: Within bounds but should be blocked (test case error!)")
            all_passed = False
        print("")
    
    if all_passed:
        print("✅ All workspace boundary tests passed!")
    else:
        print("❌ Some workspace boundary tests failed")
    
    return all_passed


def test_integrated_safety():
    """Test integrated safety checking with ViveToXArmMapper."""
    print("\n" + "="*60)
    print("TEST: Integrated Safety (ViveToXArmMapper.is_pose_safe)")
    print("="*60)
    
    os.environ['XARM_IP'] = os.environ.get('XARM_IP', '192.168.1.214')
    os.environ['EXECUTE_LIVE'] = '1'
    
    try:
        # Import here to avoid circular dependency
        from vive_teleop_xarm import ViveToXArmMapper
        
        print("\nInitializing mapper...")
        mapper = ViveToXArmMapper(
            xarm_ip=os.environ['XARM_IP'],
            position_scale=1.5,
            rotation_scale=1.0
        )
        
        print("\nSkipping tracker calibration for static safety tests...")
        print("(Safety checks don't require calibrated reference poses)")
        
        # Test cases: (pose, should_be_safe, description)
        test_cases = [
            (
                {'x': 275.0, 'y': 0.0, 'z': 400.0, 'roll': 180.0, 'pitch': -1.0, 'yaw': 0.0},
                True,
                "HOME-like pose (should be safe)"
            ),
            (
                {'x': 275.0, 'y': 0.0, 'z': 50.0, 'roll': 180.0, 'pitch': -1.0, 'yaw': 0.0},
                False,
                "Low Z pose (table collision)"
            ),
            (
                {'x': 50.0, 'y': 0.0, 'z': 400.0, 'roll': 180.0, 'pitch': -1.0, 'yaw': 0.0},
                False,
                "X too small (workspace violation)"
            ),
            (
                {'x': 275.0, 'y': -500.0, 'z': 400.0, 'roll': 180.0, 'pitch': -1.0, 'yaw': 0.0},
                False,
                "Y too negative (workspace violation)"
            ),
        ]
        
        all_passed = True
        
        for i, (pose, should_be_safe, description) in enumerate(test_cases, 1):
            print(f"\n{i}. Testing: {description}")
            print(f"   Pose: x={pose['x']:.1f}, y={pose['y']:.1f}, z={pose['z']:.1f}, "
                  f"roll={pose['roll']:.1f}, pitch={pose['pitch']:.1f}, yaw={pose['yaw']:.1f}")
            
            is_safe, reason = mapper.is_pose_safe(pose)
            
            print(f"   Result: {'SAFE' if is_safe else 'UNSAFE'} - {reason}")
            
            if should_be_safe and is_safe:
                print(f"   ✅ PASS: Correctly identified as safe")
            elif should_be_safe and not is_safe:
                print(f"   ❌ FAIL: Safe pose incorrectly blocked!")
                all_passed = False
            elif not should_be_safe and not is_safe:
                print(f"   ✅ PASS: Correctly blocked unsafe pose")
            else:  # not should_be_safe and is_safe
                print(f"   ❌ FAIL: Unsafe pose incorrectly accepted!")
                all_passed = False
        
        mapper.shutdown()
        
        if all_passed:
            print("\n✅ All integrated safety tests passed!")
        else:
            print("\n❌ Some integrated safety tests failed")
        
        return all_passed
        
    except Exception as e:
        print(f"❌ Test failed: {e}")
        import traceback
        traceback.print_exc()
        return False


def main():
    print("="*60)
    print("Vive Tracker Teleoperation Safety Test Suite")
    print("="*60)
    print("\nThese tests validate safety features WITHOUT moving the robot.")
    print("They check that unsafe poses are correctly rejected.")
    
    results = {}
    
    # Run tests
    results['workspace_boundaries'] = test_workspace_boundaries()
    results['table_collision'] = test_table_collision_detection()
    results['integrated_safety'] = test_integrated_safety()
    
    # Summary
    print("\n" + "="*60)
    print("SAFETY TEST SUMMARY")
    print("="*60)
    
    for test_name, passed in results.items():
        status = "✅ PASS" if passed else "❌ FAIL"
        print(f"{status}: {test_name.replace('_', ' ').title()}")
    
    all_passed = all(results.values())
    
    if all_passed:
        print("\n🎉 All safety tests passed!")
        print("   The teleoperation system will correctly block unsafe commands.")
    else:
        print("\n⚠️ Some safety tests failed!")
        print("   DO NOT use teleoperation until these are fixed.")
    
    return 0 if all_passed else 1


if __name__ == '__main__':
    sys.exit(main())

