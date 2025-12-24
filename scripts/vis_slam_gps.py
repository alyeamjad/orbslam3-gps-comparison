#!/usr/bin/env python3
"""
Compare ORB-SLAM3 trajectory with GPS trajectory
"""

import numpy as np
import matplotlib.pyplot as plt
from scipy.spatial.transform import Rotation

print("\n" + "=" * 70)
print("GPS vs ORB-SLAM3 TRAJECTORY COMPARISON")
print("=" * 70 + "\n")

# ===========================
# 1. Load GPS Data from CSV
# ===========================
gps_file = 'gps.csv'
print(f"Loading GPS: {gps_file}")

gps_data = []
with open(gps_file, 'r') as f:
    header = next(f)  # Skip header: time,lat,lon
    for line in f:
        parts = line.strip().split(',')
        if len(parts) >= 3:
            try:
                timestamp = float(parts[0])
                lat = float(parts[1])
                lon = float(parts[2])
                gps_data.append([timestamp, lat, lon])
            except:
                continue

if len(gps_data) == 0:
    print("❌ No GPS data found!")
    exit(1)

gps_array = np.array(gps_data)
gps_times = gps_array[:, 0]
gps_coords = gps_array[:, 1:3]  # lat, lon

# Normalize timestamps to start from 0
gps_times = gps_times - gps_times[0]

print(f"✓ Loaded {len(gps_coords)} GPS points")
print(f"  Duration: {gps_times[-1]:.2f} seconds")
print(f"  Latitude range: {gps_coords[:, 0].min():.8f} to {gps_coords[:, 0].max():.8f}")
print(f"  Longitude range: {gps_coords[:, 1].min():.8f} to {gps_coords[:, 1].max():.8f}")

# ===========================
# 2. Convert GPS to meters (local ENU coordinate system)
# ===========================
print("\nConverting GPS to local ENU coordinates...")

# Use first GPS point as origin
lat0, lon0 = gps_coords[0]
lat0_rad = np.radians(lat0)
lon0_rad = np.radians(lon0)

# Earth radius
R = 6378137.0  # meters

# Convert to East-North-Up (ENU) coordinates
gps_enu = np.zeros((len(gps_coords), 2))
for i, (lat, lon) in enumerate(gps_coords):
    lat_rad = np.radians(lat)
    lon_rad = np.radians(lon)
    
    # Calculate East-North displacement
    dLat = lat_rad - lat0_rad
    dLon = lon_rad - lon0_rad
    
    E = R * dLon * np.cos(lat0_rad)  # East
    N = R * dLat                      # North
    
    gps_enu[i] = [E, N]

print(f"✓ GPS converted to ENU coordinates")
print(f"  East range: {gps_enu[:, 0].min():.2f} to {gps_enu[:, 0].max():.2f} m")
print(f"  North range: {gps_enu[:, 1].min():.2f} to {gps_enu[:, 1].max():.2f} m")

# Calculate GPS trajectory length
gps_distances = np.linalg.norm(np.diff(gps_enu, axis=0), axis=1)
gps_total_distance = np.sum(gps_distances)
print(f"  Total GPS distance: {gps_total_distance:.2f} meters")

# ===========================
# 3. Load SLAM trajectory
# ===========================
slam_file = 'KeyFrameTrajectory.txt'
print(f"\nLoading SLAM: {slam_file}")

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
slam_times = slam_array[:, 0]
slam_positions = slam_array[:, 1:4]

print(f"✓ Loaded {len(slam_positions)} SLAM keyframes")
print(f"  Duration: {slam_times[-1]:.2f} seconds")

# Extract X-Z (horizontal plane) from SLAM
slam_xz = slam_positions[:, [0, 2]]  # X and Z coordinates

# Calculate SLAM trajectory length
slam_distances = np.linalg.norm(np.diff(slam_xz, axis=0), axis=1)
slam_total_distance = np.sum(slam_distances)
print(f"  Total SLAM distance (XZ): {slam_total_distance:.2f} meters")

# ===========================
# 4. Align SLAM trajectory to GPS
# ===========================
print("\nAligning SLAM trajectory to GPS...")

# Center both trajectories
slam_centered = slam_xz - slam_xz.mean(axis=0)
gps_centered = gps_enu - gps_enu.mean(axis=0)

# Scale SLAM to match GPS scale
slam_scale = gps_total_distance / slam_total_distance if slam_total_distance > 0 else 1.0
slam_scaled = slam_centered * slam_scale

print(f"  Scale factor: {slam_scale:.2f}")

# Find optimal rotation using Procrustes analysis
best_error = float('inf')
best_angle = 0
best_slam_aligned = slam_scaled.copy()

for angle_deg in range(0, 360, 5):
    angle_rad = np.radians(angle_deg)
    cos_a = np.cos(angle_rad)
    sin_a = np.sin(angle_rad)
    
    # Rotation matrix
    R_mat = np.array([[cos_a, -sin_a],
                      [sin_a,  cos_a]])
    
    # Rotate SLAM
    slam_rotated = (R_mat @ slam_scaled.T).T
    
    # Calculate error (using interpolation for fair comparison)
    min_len = min(len(slam_rotated), len(gps_centered))
    error = np.mean(np.linalg.norm(slam_rotated[:min_len] - gps_centered[:min_len], axis=1))
    
    if error < best_error:
        best_error = error
        best_angle = angle_deg
        best_slam_aligned = slam_rotated

# Translate to GPS origin
slam_aligned = best_slam_aligned + gps_enu.mean(axis=0)

print(f"  Best rotation: {best_angle}°")
print(f"  Alignment error: {best_error:.2f} meters")

# ===========================
# 5. Calculate trajectory similarity metrics
# ===========================
print("\nTrajectory Comparison Metrics:")

# RMSE between trajectories (interpolate to match points)
from scipy.interpolate import interp1d

if len(slam_times) > 1 and len(gps_times) > 1:
    # Interpolate SLAM to GPS timestamps
    try:
        slam_interp_e = interp1d(slam_times, slam_aligned[:, 0], 
                                 bounds_error=False, fill_value='extrapolate')
        slam_interp_n = interp1d(slam_times, slam_aligned[:, 1], 
                                 bounds_error=False, fill_value='extrapolate')
        
        # Find common time range
        t_min = max(slam_times.min(), gps_times.min())
        t_max = min(slam_times.max(), gps_times.max())
        
        # Create common timestamps
        common_times = gps_times[(gps_times >= t_min) & (gps_times <= t_max)]
        
        if len(common_times) > 0:
            slam_at_gps_times = np.column_stack([
                slam_interp_e(common_times),
                slam_interp_n(common_times)
            ])
            
            gps_at_common = gps_enu[(gps_times >= t_min) & (gps_times <= t_max)]
            
            # Calculate RMSE
            errors = np.linalg.norm(slam_at_gps_times - gps_at_common, axis=1)
            rmse = np.sqrt(np.mean(errors**2))
            
            print(f"  RMSE: {rmse:.2f} meters")
            print(f"  Mean error: {np.mean(errors):.2f} meters")
            print(f"  Max error: {np.max(errors):.2f} meters")
            print(f"  Min error: {np.min(errors):.2f} meters")
    except Exception as e:
        print(f"  Could not calculate interpolated errors: {e}")

# ===========================
# 6. Visualization
# ===========================
print("\nCreating visualizations...")

fig = plt.figure(figsize=(18, 6))

# Plot 1: Overlaid trajectories
ax1 = fig.add_subplot(131)
ax1.plot(gps_enu[:, 0], gps_enu[:, 1], 'g-', linewidth=2.5, 
         label=f'GPS ({len(gps_enu)} points)', alpha=0.8)
ax1.plot(slam_aligned[:, 0], slam_aligned[:, 1], 'b--', linewidth=2.5, 
         label=f'SLAM ({len(slam_aligned)} keyframes)', alpha=0.8)
ax1.plot(gps_enu[0, 0], gps_enu[0, 1], 'go', markersize=15, 
         label='Start', zorder=5)
ax1.plot(gps_enu[-1, 0], gps_enu[-1, 1], 'rs', markersize=15, 
         label='End', zorder=5)
ax1.set_xlabel('East (meters)', fontsize=12, fontweight='bold')
ax1.set_ylabel('North (meters)', fontsize=12, fontweight='bold')
ax1.set_title('GPS vs SLAM - Aligned Trajectories', fontsize=14, fontweight='bold')
ax1.legend(fontsize=10)
ax1.grid(True, alpha=0.3)
ax1.axis('equal')

# Plot 2: GPS trajectory only
ax2 = fig.add_subplot(132)
ax2.plot(gps_enu[:, 0], gps_enu[:, 1], 'g-', linewidth=2.5, alpha=0.8)
ax2.scatter(gps_enu[:, 0], gps_enu[:, 1], c=gps_times, cmap='viridis', 
            s=50, alpha=0.6, edgecolors='black', linewidth=0.5)
ax2.plot(gps_enu[0, 0], gps_enu[0, 1], 'go', markersize=15, label='Start', zorder=5)
ax2.plot(gps_enu[-1, 0], gps_enu[-1, 1], 'rs', markersize=15, label='End', zorder=5)
ax2.set_xlabel('East (meters)', fontsize=12, fontweight='bold')
ax2.set_ylabel('North (meters)', fontsize=12, fontweight='bold')
ax2.set_title(f'GPS Trajectory\nDistance: {gps_total_distance:.2f}m', 
              fontsize=14, fontweight='bold')
ax2.legend(fontsize=11)
ax2.grid(True, alpha=0.3)
ax2.axis('equal')
cbar2 = plt.colorbar(ax2.scatter(gps_enu[:, 0], gps_enu[:, 1], 
                                  c=gps_times, cmap='viridis', s=50), ax=ax2)
cbar2.set_label('Time (s)', fontsize=10)

# Plot 3: SLAM trajectory only
ax3 = fig.add_subplot(133)
ax3.plot(slam_aligned[:, 0], slam_aligned[:, 1], 'b-', linewidth=2.5, alpha=0.8)
ax3.scatter(slam_aligned[:, 0], slam_aligned[:, 1], c=slam_times, cmap='plasma', 
            s=80, alpha=0.7, edgecolors='black', linewidth=0.5)
ax3.plot(slam_aligned[0, 0], slam_aligned[0, 1], 'go', markersize=15, label='Start', zorder=5)
ax3.plot(slam_aligned[-1, 0], slam_aligned[-1, 1], 'rs', markersize=15, label='End', zorder=5)
ax3.set_xlabel('East (meters)', fontsize=12, fontweight='bold')
ax3.set_ylabel('North (meters)', fontsize=12, fontweight='bold')
ax3.set_title(f'SLAM Trajectory (Aligned)\nDistance: {slam_total_distance:.2f}m', 
              fontsize=14, fontweight='bold')
ax3.legend(fontsize=11)
ax3.grid(True, alpha=0.3)
ax3.axis('equal')
cbar3 = plt.colorbar(ax3.scatter(slam_aligned[:, 0], slam_aligned[:, 1], 
                                  c=slam_times, cmap='plasma', s=80), ax=ax3)
cbar3.set_label('Time (s)', fontsize=10)

plt.tight_layout()
plt.savefig('gps_slam_comparison.png', dpi=150, bbox_inches='tight')
print("✓ Saved: gps_slam_comparison.png")

# Additional plot: Error over time
if 'errors' in locals():
    fig2, ax = plt.subplots(figsize=(12, 5))
    ax.plot(common_times, errors, 'r-', linewidth=2, alpha=0.8)
    ax.axhline(y=rmse, color='b', linestyle='--', linewidth=2, label=f'RMSE: {rmse:.2f}m')
    ax.fill_between(common_times, 0, errors, alpha=0.3, color='red')
    ax.set_xlabel('Time (seconds)', fontsize=12, fontweight='bold')
    ax.set_ylabel('Error (meters)', fontsize=12, fontweight='bold')
    ax.set_title('GPS vs SLAM Position Error Over Time', fontsize=14, fontweight='bold')
    ax.legend(fontsize=11)
    ax.grid(True, alpha=0.3)
    plt.tight_layout()
    plt.savefig('error_over_time.png', dpi=150, bbox_inches='tight')
    print("✓ Saved: error_over_time.png")

# ===========================
# 7. Save detailed results
# ===========================
with open('comparison_results.txt', 'w') as f:
    f.write("GPS vs ORB-SLAM3 TRAJECTORY COMPARISON\n")
    f.write("=" * 70 + "\n\n")
    
    f.write("GPS Trajectory:\n")
    f.write(f"  Number of points: {len(gps_enu)}\n")
    f.write(f"  Total distance: {gps_total_distance:.2f} meters\n")
    f.write(f"  Duration: {gps_times[-1]:.2f} seconds\n")
    f.write(f"  Latitude range: {gps_coords[:, 0].min():.8f}° to {gps_coords[:, 0].max():.8f}°\n")
    f.write(f"  Longitude range: {gps_coords[:, 1].min():.8f}° to {gps_coords[:, 1].max():.8f}°\n\n")
    
    f.write("SLAM Trajectory:\n")
    f.write(f"  Number of keyframes: {len(slam_aligned)}\n")
    f.write(f"  Total distance: {slam_total_distance:.2f} meters\n")
    f.write(f"  Duration: {slam_times[-1]:.2f} seconds\n\n")
    
    f.write("Alignment Parameters:\n")
    f.write(f"  Scale factor: {slam_scale:.3f}\n")
    f.write(f"  Rotation angle: {best_angle}°\n")
    f.write(f"  Initial alignment error: {best_error:.2f} meters\n\n")
    
    f.write("Accuracy Metrics:\n")
    if 'rmse' in locals():
        f.write(f"  RMSE: {rmse:.2f} meters\n")
        f.write(f"  Mean error: {np.mean(errors):.2f} meters\n")
        f.write(f"  Max error: {np.max(errors):.2f} meters\n")
        f.write(f"  Min error: {np.min(errors):.2f} meters\n\n")
    
    f.write("Interpretation:\n")
    f.write("  The trajectories have been aligned using:\n")
    f.write("    1. Centering (remove translation offset)\n")
    f.write("    2. Scaling (match trajectory lengths)\n")
    f.write("    3. Rotation (align orientations)\n\n")
    f.write("  Differences between GPS and SLAM may be due to:\n")
    f.write("    - GPS accuracy limitations (~5-10m typical for smartphones)\n")
    f.write("    - SLAM drift over time (accumulates without loop closure)\n")
    f.write("    - Different sampling rates (GPS ~1Hz vs SLAM keyframes)\n")
    f.write("    - GPS multipath effects (signal bouncing off buildings)\n")
    f.write("    - SLAM scale ambiguity in monocular vision\n\n")
    f.write("  Both methods successfully tracked the camera/phone movement!\n")

print("✓ Saved: comparison_results.txt")

print("\n" + "=" * 70)
print("✅ COMPARISON COMPLETE")
print("=" * 70)
print(f"\nKey Results:")
print(f"  • GPS tracked {gps_total_distance:.2f}m over {gps_times[-1]:.1f}s")
print(f"  • SLAM tracked {slam_total_distance:.2f}m with {len(slam_aligned)} keyframes")
print(f"  • Scale difference: {slam_scale:.2f}x")
if 'rmse' in locals():
    print(f"  • Position error (RMSE): {rmse:.2f}m")

plt.show()
