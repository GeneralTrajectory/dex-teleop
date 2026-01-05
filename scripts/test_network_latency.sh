#!/bin/bash
echo "Testing network latency to both xArms..."
echo ""
echo "Left arm (192.168.1.111):"
ping -c 50 -i 0.01 192.168.1.111 | grep -E 'rtt|packet'
echo ""
echo "Right arm (192.168.1.214):"
ping -c 50 -i 0.01 192.168.1.214 | grep -E 'rtt|packet'
