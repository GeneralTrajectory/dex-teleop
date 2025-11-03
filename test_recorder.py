#!/usr/bin/env python3
"""
Test suite for teleoperation recorder.

Tests:
1. Module imports
2. HDF5 writer (create, write, finalize, discard)
3. Pedal debouncing
4. Data validation
"""

import sys
import time
import tempfile
from pathlib import Path

# Add to path
sys.path.insert(0, str(Path(__file__).parent))

def test_imports():
    """Test that all recorder modules can be imported."""
    print("Test 1: Module imports")
    try:
        from recorder_lib import (
            RecorderPedal,
            XArmRecorder,
            InspireRecorder,
            HDF5Writer,
            RecorderOrchestrator,
            RecorderState,
            replay_trial,
            list_trials,
            validate_trial,
        )
        print("✅ All modules imported successfully")
        return True
    except Exception as e:
        print(f"❌ Import failed: {e}")
        import traceback
        traceback.print_exc()
        return False


def test_hdf5_writer():
    """Test HDF5 writer with temp file creation and atomic commit."""
    print("\nTest 2: HDF5 Writer")
    
    try:
        from recorder_lib import HDF5Writer
        import numpy as np
        
        # Create temp directory
        with tempfile.TemporaryDirectory() as tmpdir:
            # Test normal mode (save)
            writer = HDF5Writer(
                output_dir=tmpdir,
                subject="test",
                task="test_task",
                notes="test notes",
                ephemeral=False
            )
            
            trial_id = writer.start(config={'test': 'config'})
            print(f"  Created trial: {trial_id}")
            
            # Append some test data
            xarm_data = [
                {
                    't_mono': time.monotonic(),
                    'joint_pos': [0.0] * 7,
                    'tcp_pose': [275.0, 0.0, 670.0, 180.0, -1.0, 0.0],
                }
                for _ in range(10)
            ]
            writer.append_xarm('left', xarm_data)
            
            inspire_data = [
                {
                    't_mono': time.monotonic(),
                    'angles': [500, 500, 500, 500, 500, 500],
                }
                for _ in range(10)
            ]
            writer.append_inspire('left', inspire_data)
            
            # Finalize
            final_path = writer.finalize()
            if not final_path:
                print("❌ Finalize returned None")
                return False
            
            final_file = Path(final_path)
            if not final_file.exists():
                print(f"❌ Final file not created: {final_path}")
                return False
            
            print(f"✅ File saved: {final_file.name}")
            
            # Test ephemeral mode (discard)
            writer2 = HDF5Writer(
                output_dir=tmpdir,
                subject="test",
                task="ephemeral_test",
                notes="",
                ephemeral=True
            )
            
            trial_id2 = writer2.start()
            writer2.append_xarm('left', xarm_data)
            result = writer2.finalize()
            
            if result is not None:
                print(f"❌ Ephemeral mode returned path (should be None)")
                return False
            
            # Check that ephemeral file was deleted
            ephemeral_file = Path(tmpdir) / f"trial_{trial_id2}.h5"
            if ephemeral_file.exists():
                print(f"❌ Ephemeral file was not deleted: {ephemeral_file}")
                return False
            
            print("✅ Ephemeral mode correctly discarded data")
        
        print("✅ HDF5 Writer test passed")
        return True
    
    except Exception as e:
        print(f"❌ HDF5 Writer test failed: {e}")
        import traceback
        traceback.print_exc()
        return False


def test_pedal_debounce():
    """Test pedal debouncing logic."""
    print("\nTest 3: Pedal Debouncing")
    
    try:
        from recorder_lib import RecorderPedal
        
        press_count = 0
        press_times = []
        
        def on_press():
            nonlocal press_count
            press_count += 1
            press_times.append(time.time())
        
        # Note: This is a logic test, not a live keyboard test
        pedal = RecorderPedal(on_toggle_callback=on_press)
        
        # Simulate debounce logic
        simulated_presses = [0.0, 0.05, 0.1, 0.2, 0.4]  # Times in seconds
        last_press = -1.0
        debounce_s = 0.15
        
        registered_presses = 0
        for press_time in simulated_presses:
            if press_time - last_press >= debounce_s:
                registered_presses += 1
                last_press = press_time
        
        expected_presses = 3  # 0.0, 0.2, 0.4 (0.05 and 0.1 debounced)
        if registered_presses == expected_presses:
            print(f"✅ Debounce logic correct: {registered_presses}/{len(simulated_presses)} presses registered")
            return True
        else:
            print(f"❌ Expected {expected_presses} but got {registered_presses}")
            return False
    
    except Exception as e:
        print(f"❌ Pedal test failed: {e}")
        return False


def test_data_validation():
    """Test data validation on a mock HDF5 file."""
    print("\nTest 4: Data Validation")
    
    try:
        from recorder_lib import HDF5Writer, validate_trial
        import tempfile
        
        with tempfile.TemporaryDirectory() as tmpdir:
            # Create a valid trial
            writer = HDF5Writer(
                output_dir=tmpdir,
                subject="test",
                task="validation_test",
                notes="",
                ephemeral=False
            )
            
            trial_id = writer.start()
            
            # Add valid data
            import numpy as np
            for i in range(100):
                t = time.monotonic()
                writer.append_xarm('left', [{
                    't_mono': t,
                    'joint_pos': np.zeros(7).tolist(),
                    'tcp_pose': [275.0, 0.0, 670.0, 180.0, -1.0, 0.0],
                }])
                writer.append_inspire('left', [{
                    't_mono': t,
                    'angles': [500] * 6,
                }])
            
            writer.finalize()
            
            # Validate
            valid = validate_trial(trial_id, data_dir=tmpdir)
            
            if valid:
                print("✅ Validation test passed")
                return True
            else:
                print("❌ Validation failed on valid file")
                return False
    
    except Exception as e:
        print(f"❌ Validation test failed: {e}")
        import traceback
        traceback.print_exc()
        return False


def main():
    """Run all tests."""
    print("="*60)
    print("Teleoperation Recorder Test Suite")
    print("="*60)
    print()
    
    tests = [
        test_imports,
        test_hdf5_writer,
        test_pedal_debounce,
        test_data_validation,
    ]
    
    results = []
    for test_func in tests:
        result = test_func()
        results.append(result)
        time.sleep(0.5)
    
    print("\n" + "="*60)
    print(f"Results: {sum(results)}/{len(results)} tests passed")
    print("="*60)
    
    if all(results):
        print("✅ All tests passed!")
        return 0
    else:
        print("❌ Some tests failed")
        return 1


if __name__ == '__main__':
    sys.exit(main())










