#!/usr/bin/env python3
"""
Visualize ORB-SLAM3 trajectory (GPS data was stationary)
"""

import numpy as np
import matplotlib.pyplot as plt

print("\n" + "=" * 70)
print("ORB-SLAM3 TRAJECTORY VISUALIZATION")
print("=" * 70 + "\n")

# Load SLAM trajectory
slam_file = 'KeyFrameTrajectory.txt'
print(f"Loading: {slam_file}")

slam_data = []
with open(slam_file, 'r') as f:
    for line in f:
        if line.startswith('#'):
            continue
        parts = line.strip().split()
        if len(parts) >= 4:
            timestamp = float(parts[0])
            x, y, z = float(parts[1]), float(parts[2]), float(parts[3])
            slam_data.append([timestamp, x, y, z])

if len(slam_data) == 0:
    print("❌ No SLAM data found!")
    exit(1)

slam_array = np.array(slam_data)
times = slam_array[:, 0]
positions = slam_array[:, 1:4]

print(f"✓ Loaded {len(positions)} SLAM keyframes")
print(f"  Duration: {times[-1]:.2f} seconds")

# Calculate trajectory length
distances = np.linalg.norm(np.diff(positions, axis=0), axis=1)
total_distance = np.sum(distances)

print(f"\nSLAM Trajectory Statistics:")
print(f"  Total distance: {total_distance:.2f} meters")
print(f"  X range: {positions[:, 0].min():.2f} to {positions[:, 0].max():.2f} m")
print(f"  Y range: {positions[:, 1].min():.2f} to {positions[:, 1].max():.2f} m")
print(f"  Z range: {positions[:, 2].min():.2f} to {positions[:, 2].max():.2f} m")

# Create visualization
fig = plt.figure(figsize=(16, 5))

# Plot 1: Top view (X-Z)
ax1 = fig.add_subplot(131)
ax1.plot(positions[:, 0], positions[:, 2], 'b-', linewidth=2, alpha=0.8)
ax1.plot(positions[0, 0], positions[0, 2], 'go', markersize=12, label='Start', zorder=5)
ax1.plot(positions[-1, 0], positions[-1, 2], 'rs', markersize=12, label='End', zorder=5)
ax1.set_xlabel('X (meters)', fontsize=12, fontweight='bold')
ax1.set_ylabel('Z (meters)', fontsize=12, fontweight='bold')
ax1.set_title('Camera Trajectory - Top View', fontsize=14, fontweight='bold')
ax1.legend(fontsize=11)
ax1.grid(True, alpha=0.3)
ax1.axis('equal')

# Plot 2: Side view (X-Y)
ax2 = fig.add_subplot(132)
ax2.plot(positions[:, 0], positions[:, 1], 'b-', linewidth=2, alpha=0.8)
ax2.plot(positions[0, 0], positions[0, 1], 'go', markersize=12, label='Start', zorder=5)
ax2.set_xlabel('X (meters)', fontsize=12, fontweight='bold')
ax2.set_ylabel('Y (meters)', fontsize=12, fontweight='bold')
ax2.set_title('Camera Trajectory - Side View', fontsize=14, fontweight='bold')
ax2.legend(fontsize=11)
ax2.grid(True, alpha=0.3)
ax2.axis('equal')

# Plot 3: 3D trajectory
ax3 = fig.add_subplot(133, projection='3d')
ax3.plot(positions[:, 0], positions[:, 2], positions[:, 1], 'b-', linewidth=2, alpha=0.8)
ax3.scatter(positions[0, 0], positions[0, 2], positions[0, 1], 
           c='green', s=100, label='Start', zorder=5)
ax3.scatter(positions[-1, 0], positions[-1, 2], positions[-1, 1], 
           c='red', s=100, marker='s', label='End', zorder=5)
ax3.set_xlabel('X (m)', fontsize=10)
ax3.set_ylabel('Z (m)', fontsize=10)
ax3.set_zlabel('Y (m)', fontsize=10)
ax3.set_title('3D Camera Trajectory', fontsize=14, fontweight='bold')
ax3.legend(fontsize=10)

plt.tight_layout()
plt.savefig('slam_trajectory_visualization.png', dpi=150, bbox_inches='tight')
print(f"\n✓ Saved: slam_trajectory_visualization.png")

# Save summary
with open('slam_results.txt', 'w') as f:
    f.write("ORB-SLAM3 Trajectory Results\n")
    f.write("=" * 70 + "\n\n")
    f.write(f"Video Duration: {times[-1]:.2f} seconds\n")
    f.write(f"Number of Keyframes: {len(positions)}\n")
    f.write(f"Total Distance Traveled: {total_distance:.2f} meters\n\n")
    f.write(f"Position Ranges:\n")
    f.write(f"  X: {positions[:, 0].min():.3f} to {positions[:, 0].max():.3f} m\n")
    f.write(f"  Y: {positions[:, 1].min():.3f} to {positions[:, 1].max():.3f} m\n")
    f.write(f"  Z: {positions[:, 2].min():.3f} to {positions[:, 2].max():.3f} m\n\n")
    f.write("GPS Data Status:\n")
    f.write("  ⚠️ GPS did not record movement (stationary at one point)\n")
    f.write("  All GPS coordinates were identical\n")
    f.write("  Possible causes:\n")
    f.write("    - GPS not updating during recording\n")
    f.write("    - Poor GPS signal\n")
    f.write("    - Recording done indoors\n\n")
    f.write("Note: ORB-SLAM3 trajectory shows camera movement successfully,\n")
    f.write("but GPS comparison was not possible due to stationary GPS data.\n")

print("✓ Saved: slam_results.txt")

print("\n" + "=" * 70)
print("✅ VISUALIZATION COMPLETE")
print("=" * 70)
print("\n⚠️  NOTE: GPS data was stationary (no movement recorded)")
print("For full project, you would need to re-record with moving GPS.")
print("\nYou can still demonstrate:")
print("  1. ORB-SLAM3 successfully tracked camera motion")
print("  2. Understanding of the methodology")
print("  3. What WOULD happen with proper GPS data")
print("  4. Your successful TUM dataset results as reference")

plt.show()
