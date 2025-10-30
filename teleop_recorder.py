#!/usr/bin/env python3
"""
Dual-Tracker Teleoperation Recorder

Simultaneously records bimanual xArm (via Vive trackers) and Inspire Hands
(via Quest hand tracking) with pedal-triggered start/stop.

Usage:
    # Record a demonstration
    python teleop_recorder.py record --subject S1 --task pick --notes "red cube"
    
    # Ephemeral mode (testing, no save)
    python teleop_recorder.py record --ephemeral
    
    # List recordings
    python teleop_recorder.py list
    
    # Validate a recording
    python teleop_recorder.py validate --trial-id 20250130_143022
    
    # Replay a recording (dry-run)
    python teleop_recorder.py replay --trial-id 20250130_143022 --dry-run
    
    # Replay live at 0.5x speed
    python teleop_recorder.py replay --trial-id 20250130_143022 --speed 0.5

Controls:
    - Press 'b' (foot pedal) to start/stop recording
    - Press Ctrl+C to exit
"""

import sys
import argparse
from pathlib import Path

# Add recorder_lib to path
sys.path.insert(0, str(Path(__file__).parent))

from recorder_lib import (
    RecorderOrchestrator,
    replay_trial,
    list_trials,
    validate_trial,
)


def cmd_record(args):
    """Handle 'record' command."""
    recorder = RecorderOrchestrator(
        ephemeral=args.ephemeral,
        subject=args.subject,
        task=args.task,
        notes=args.notes,
        arms=args.arms
    )
    return recorder.run()


def cmd_list(args):
    """Handle 'list' command."""
    list_trials(data_dir=args.data_dir)
    return 0


def cmd_validate(args):
    """Handle 'validate' command."""
    if not args.trial_id:
        print("❌ --trial-id required for validate command")
        return 1
    
    success = validate_trial(args.trial_id, data_dir=args.data_dir)
    return 0 if success else 1


def cmd_replay(args):
    """Handle 'replay' command."""
    if not args.trial_id:
        print("❌ --trial-id required for replay command")
        return 1
    
    return replay_trial(
        trial_id=args.trial_id,
        speed=args.speed,
        dry_run=args.dry_run,
        data_dir=args.data_dir
    )


def main():
    """Main entry point with CLI argument parsing."""
    parser = argparse.ArgumentParser(
        description="Dual-tracker teleoperation recorder for xArm + Inspire Hands",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=__doc__
    )
    
    # Subcommands
    subparsers = parser.add_subparsers(dest='command', help='Command to execute')
    subparsers.required = True
    
    # Record command
    record_parser = subparsers.add_parser('record', help='Record a demonstration')
    record_parser.add_argument('--subject', help='Subject ID (e.g., S1)')
    record_parser.add_argument('--task', help='Task name (e.g., pick, place)')
    record_parser.add_argument('--notes', help='Session notes')
    record_parser.add_argument('--ephemeral', action='store_true',
                              help='Discard data after stop (testing mode)')
    record_parser.add_argument('--arms', choices=['left', 'right', 'both'], default='both',
                              help='Which arm(s) to use: left, right, or both (default: both)')
    record_parser.set_defaults(func=cmd_record)
    
    # List command
    list_parser = subparsers.add_parser('list', help='List all recordings')
    list_parser.add_argument('--data-dir', default='/home/joshua/Research/dex-teleop/data',
                            help='Data directory (default: ./data)')
    list_parser.set_defaults(func=cmd_list)
    
    # Validate command
    validate_parser = subparsers.add_parser('validate', help='Validate a recording')
    validate_parser.add_argument('--trial-id', required=True,
                                help='Trial ID to validate (e.g., 20250130_143022)')
    validate_parser.add_argument('--data-dir', default='/home/joshua/Research/dex-teleop/data',
                                help='Data directory (default: ./data)')
    validate_parser.set_defaults(func=cmd_validate)
    
    # Replay command
    replay_parser = subparsers.add_parser('replay', help='Replay a recording')
    replay_parser.add_argument('--trial-id', required=True,
                              help='Trial ID to replay (e.g., 20250130_143022)')
    replay_parser.add_argument('--speed', type=float, default=1.0,
                              help='Playback speed multiplier (default: 1.0)')
    replay_parser.add_argument('--dry-run', action='store_true',
                              help='Validate and show stats without robot motion')
    replay_parser.add_argument('--data-dir', default='/home/joshua/Research/dex-teleop/data',
                              help='Data directory (default: ./data)')
    replay_parser.set_defaults(func=cmd_replay)
    
    args = parser.parse_args()
    
    # Add data_dir to record command if not present
    if args.command == 'record':
        args.data_dir = '/home/joshua/Research/dex-teleop/data'
    
    # Execute command
    return args.func(args)


if __name__ == '__main__':
    sys.exit(main())


