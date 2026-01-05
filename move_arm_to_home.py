#!/usr/bin/env python3
"""
Helper script to move xArm to standard home position before teleoperation.
Safe, simple, single-purpose utility.
"""

import sys
import os
from pathlib import Path

# Import local xarm_adapter
from xarm_adapter import create_xarm_adapter, HOME_POSE


def main():
    """Move xArm to home position."""
    
    xarm_ip = os.environ.get('XARM_IP', '192.168.1.214')
    
    print("="*60)
    print("Move xArm to Home Position")
    print("="*60)
    print(f"\nTarget: xArm at {xarm_ip}")
    print(f"Home pose: {HOME_POSE}")
    print("")
    
    # Confirm
    try:
        response = input("Proceed? [y/N]: ").strip().lower()
        if response not in ('y', 'yes'):
            print("Cancelled by user")
            return 0
    except KeyboardInterrupt:
        print("\nCancelled by user")
        return 0
    
    # Initialize adapter
    print("\n🔌 Connecting to xArm...")
    os.environ['XARM_IP'] = xarm_ip
    os.environ['EXECUTE_LIVE'] = '1'
    
    try:
        adapter = create_xarm_adapter(execute_live=True, use_inspire_api=False)
        adapter._lazy_connect_if_needed()
        
        if adapter.arm is None:
            print(f"❌ Failed to connect to xArm at {xarm_ip}")
            return 1
        
        print("✅ Connected")
        
        # Query current position
        print("\n📍 Current position:")
        current = adapter._get_live_tcp_pose()
        if current:
            print(f"   x={current[0]:.1f}, y={current[1]:.1f}, z={current[2]:.1f} mm")
            print(f"   roll={current[3]:.1f}, pitch={current[4]:.1f}, yaw={current[5]:.1f}°")
        
        # Move to home
        print(f"\n🏠 Moving to home position...")
        adapter.go_home(speed=300)
        
        # Verify
        print("\n✅ Move complete! Verifying position...")
        final = adapter._get_live_tcp_pose()
        if final:
            print(f"   x={final[0]:.1f}, y={final[1]:.1f}, z={final[2]:.1f} mm")
            print(f"   roll={final[3]:.1f}, pitch={final[4]:.1f}, yaw={final[5]:.1f}°")
            
            # Check if close to home
            dx = abs(final[0] - HOME_POSE['x'])
            dy = abs(final[1] - HOME_POSE['y'])
            dz = abs(final[2] - HOME_POSE['z'])
            
            if dx < 5.0 and dy < 5.0 and dz < 5.0:
                print("\n✅ Arm is at home position (within 5mm tolerance)")
            else:
                print(f"\n⚠️ Position mismatch: Δx={dx:.1f}, Δy={dy:.1f}, Δz={dz:.1f} mm")
        
        adapter.disconnect()
        print("\n✅ Done! Arm is ready for teleoperation.")
        return 0
        
    except Exception as e:
        print(f"\n❌ Error: {e}")
        import traceback
        traceback.print_exc()
        return 1


if __name__ == '__main__':
    sys.exit(main())

