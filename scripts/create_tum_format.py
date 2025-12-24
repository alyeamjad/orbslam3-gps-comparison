#!/usr/bin/env python3
"""
Create TUM format rgb.txt file from timestamps
"""

import pandas as pd
from pathlib import Path

print("Creating TUM format dataset...")

# Load timestamps
timestamps_file = 'timestamps.txt'
if not Path(timestamps_file).exists():
    print(f"ERROR: {timestamps_file} not found!")
    exit(1)

# Read timestamps
with open(timestamps_file, 'r') as f:
    timestamps = [float(line.strip()) for line in f]

print(f"Found {len(timestamps)} timestamps")

# Create rgb.txt in TUM format
output_file = 'video_sequence/rgb.txt'
with open(output_file, 'w') as f:
    f.write("# timestamp filename\n")
    for i, ts in enumerate(timestamps):
        # Frame files are named: 000001.png, 000002.png, etc.
        frame_filename = f"{i+1:06d}.png"
        f.write(f"{ts:.6f} rgb/{frame_filename}\n")

print(f"✓ Created {output_file}")
print(f"✓ Dataset ready for ORB-SLAM3")

# Verify first frame exists
first_frame = Path(f"video_sequence/rgb/{1:06d}.png")
if first_frame.exists():
    print(f"✓ Verified: First frame exists")
else:
    print(f"⚠ Warning: First frame not found at {first_frame}")
