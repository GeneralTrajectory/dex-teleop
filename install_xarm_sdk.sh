#!/bin/bash
# Install the correct xArm Python SDK
# The wrong 'xarm' package (0.0.4) is not the official SDK

echo "============================================================"
echo "Installing xArm Python SDK"
echo "============================================================"
echo ""

# Uninstall wrong package if present
echo "Step 1: Removing incorrect 'xarm' package (if installed)..."
pip uninstall -y xarm 2>/dev/null || echo "  (Package 'xarm' not found, skipping)"

echo ""
echo "Step 2: Installing official xArm-Python-SDK..."
echo "  Method: pip install from PyPI"

pip install xarm-python-sdk

echo ""
echo "Step 3: Verifying installation..."
python3 -c "
try:
    from xarm.wrapper import XArmAPI
    print('✅ xArm SDK installed correctly!')
    print('   Import: from xarm.wrapper import XArmAPI')
except ImportError as e:
    print('❌ Installation failed:', e)
    print('')
    print('Alternative: Install from GitHub:')
    print('  git clone https://github.com/xArm-Developer/xArm-Python-SDK.git')
    print('  cd xArm-Python-SDK')
    print('  python setup.py install')
    exit(1)
"

if [ $? -eq 0 ]; then
    echo ""
    echo "============================================================"
    echo "✅ Installation complete!"
    echo "============================================================"
    echo "You can now run:"
    echo "  python test_vive_teleop.py"
    echo "  python vive_teleop_xarm.py"
else
    echo ""
    echo "============================================================"
    echo "❌ Installation failed"
    echo "============================================================"
    exit 1
fi

