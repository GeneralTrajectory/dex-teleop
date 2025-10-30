# Documentation Index

This directory contains all documentation for the dex-teleop system.

## Main Documentation Files

1. **[QUICKSTART.md](QUICKSTART.md)** - Complete user guide
   - Vive Tracker → xArm teleoperation
   - Bimanual xArm control
   - Quest Hand Tracking → Inspire Hands
   - Setup, usage, and basic troubleshooting

2. **[TECHNICAL.md](TECHNICAL.md)** - Technical reference
   - Architecture overview
   - Configuration guide
   - Performance tuning guides
   - Advanced topics

3. **[INSPIRE_TELEOP_LESSONS.md](INSPIRE_TELEOP_LESSONS.md)** - Quest troubleshooting
   - Lessons learned from achieving real-time control
   - Performance bottleneck diagnosis
   - Critical optimizations for low-latency operation

## Quick Navigation

**Getting Started:**
- Read [QUICKSTART.md](QUICKSTART.md) first

**Performance Issues:**
- Quest hands slow/laggy: See [INSPIRE_TELEOP_LESSONS.md](INSPIRE_TELEOP_LESSONS.md)
- Vive tracker tuning: See [TECHNICAL.md](TECHNICAL.md#performance-tuning)

**Configuration:**
- All environment variables: See [TECHNICAL.md](TECHNICAL.md#configuration)
- Quest setup script: Use `../setup_quest_teleop.sh`

## File Structure

```
notes/
├── QUICKSTART.md              # User guide (start here)
├── TECHNICAL.md               # Technical reference
├── INSPIRE_TELEOP_LESSONS.md  # Quest troubleshooting
└── README.md                  # This file
```
