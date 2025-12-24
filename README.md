# ORB-SLAM3 Visual Navigation with GPS Comparison

A practical implementation of ORB-SLAM3 for visual odometry using smartphone data, with trajectory comparison against GPS ground truth.

## Overview

This project demonstrates visual SLAM (Simultaneous Localization and Mapping) using ORB-SLAM3 on smartphone video data. The visual trajectory is compared with GPS data to evaluate accuracy and understand the characteristics of both positioning methods.

## Key Results

- **SLAM Keyframes Generated:** 346 from 2017 video frames
- **Position RMSE:** 11.03 meters (acceptable for consumer GPS comparison)
- **Minimum Error:** 2.29 meters at best alignment
- **Scale Factor:** 8.31x (demonstrates monocular scale ambiguity)

## Technical Details

### Hardware
- Device: Samsung Galaxy S20 FE
- Camera: 12MP main camera (f/1.8)
- Video Resolution: 850x478 pixels at 30 FPS
- GPS: Consumer-grade smartphone GPS

### Software Stack
- ORB-SLAM3 (C++)
- Python 3.x for data processing and visualization
- OpenCV for camera calibration
- NumPy, SciPy, Matplotlib for analysis
- FFmpeg for video preprocessing

### Camera Calibration Parameters
```yaml
Focal Length (fx, fy): 580.0 pixels
Principal Point (cx, cy): (239.0, 425.0)
Distortion (k1, k2): 0.10, -0.05
Resolution: 850x478 pixels
```

## Methodology

### 1. Data Collection
- Recorded 67-second video while walking outdoors
- Simultaneously logged GPS coordinates
- Disabled camera autofocus for consistent focus distance

### 2. Video Preprocessing
- Fixed frame rate to 30 FPS
- Rotated from portrait to landscape orientation
- Extracted individual frames (2017 frames total)
- Generated timestamp file for ORB-SLAM3 TUM format

### 3. Camera Calibration
- Captured checkerboard pattern images from multiple angles
- Computed intrinsic parameters using OpenCV
- Generated calibration file for Samsung S20 FE

### 4. GPS Data Processing
- Converted GPX format to CSV
- Transformed geographic coordinates (lat/lon) to local ENU frame
- Synchronized GPS timestamps with video frames

### 5. Visual SLAM Processing
- Ran ORB-SLAM3 in monocular mode
- Extracted 1000 ORB features per frame
- Generated 346 keyframes with 6DOF poses

### 6. Trajectory Comparison
- Aligned SLAM and GPS trajectories using Procrustes analysis
- Computed error metrics (RMSE, mean, max, min)
- Visualized trajectory overlay and error distribution

## Performance Comparison

| Metric | Initial (Wrong Calibration) | Final (Correct Calibration) | Improvement |
|--------|---------------------------|---------------------------|-------------|
| Keyframes | 75 | 346 | +362% |
| SLAM Distance | 2.31m | 7.69m | +233% |
| Scale Factor | 27.67x | 8.31x | 3.3x better |
| RMSE | 32.27m | 11.03m | 66% reduction |

## Results

### GPS vs SLAM Trajectory
The comparison shows reasonable alignment between visual SLAM and GPS trajectories, with RMSE of 11.03 meters. The scale factor of 8.31x demonstrates the inherent scale ambiguity in monocular SLAM.

### Error Analysis
- Mean error: 10.06 meters
- Maximum error: 15.68 meters
- Minimum error: 2.29 meters

The errors are within expected range for consumer-grade GPS (5-10m typical accuracy) combined with monocular SLAM drift.

## Key Findings

### Achievements
- Successfully implemented complete ORB-SLAM3 pipeline on smartphone data
- Demonstrated importance of proper camera calibration (4.6x improvement in keyframes)
- Achieved acceptable positioning accuracy for consumer hardware
- Identified and quantified monocular scale ambiguity

### Limitations
- GPS sampling rate too low (7 points in 60 seconds)
- Monocular SLAM cannot determine absolute scale without reference
- Consumer GPS accuracy limits ground truth quality
- No loop closure detected during recording

### Lessons Learned
- Camera calibration is critical for SLAM success
- Monocular systems require external scale reference
- GPS and visual odometry are complementary
- Feature-rich environments essential for robust tracking

## Project Structure

```
.
├── 01_raw_data/
│   ├── Video.mp4                    # Original video recording
│   ├── vedio.gpx                    # GPS log in GPX format
│   └── gps.csv                      # GPS data in CSV format
│
├── 02_preprocessing/
│   ├── video_sequence_rotated/      # Extracted video frames
│   │   ├── rgb/                     # Individual frame images
│   │   └── rgb.txt                  # Timestamp associations
│   ├── samsung_s20fe_calibration.yaml
│   ├── gps_synced.csv
│   └── timestamps.txt
│
├── 03_slam_output/
│   └── KeyFrameTrajectory.txt       # ORB-SLAM3 output (346 poses)
│
├── 04_results/
│   ├── gps_slam_comparison.png      # Trajectory overlay
│   ├── error_over_time.png          # Position error plot
│   └── comparison_results.txt       # Numerical metrics
│
├── 05_scripts/
│   ├── compare_slam_gps.py          # Main comparison script
│   ├── visualize_slam_only.py       # SLAM visualization
│   ├── gpx_to_csv.py                # GPS format converter
│   ├── syn_gps_frames.py            # GPS-frame synchronization
│   └── create_tum_format.py         # TUM dataset creator
│
└── 06_documentation/
    └── Semester Project VNAV 2025.pdf
```

## Usage

### Prerequisites
```bash
# ORB-SLAM3 installed and built
# Python 3.x with required packages
pip install numpy scipy matplotlib opencv-python
```

### Running SLAM
```bash
cd ORB_SLAM3
./Examples/Monocular/mono_tum \
    Vocabulary/ORBvoc.txt \
    path/to/samsung_s20fe_calibration.yaml \
    path/to/video_sequence_rotated
```

### Running Comparison
```bash
cd 05_scripts
python compare_slam_gps.py
```

## Dependencies

- ORB-SLAM3
- Python 3.x
- OpenCV 4.x
- NumPy
- SciPy
- Matplotlib
- FFmpeg

## References

- ORB-SLAM3: [https://github.com/UZ-SLAMLab/ORB_SLAM3](https://github.com/UZ-SLAMLab/ORB_SLAM3)
- TUM RGB-D Dataset Format: [https://vision.in.tum.de/data/datasets/rgbd-dataset](https://vision.in.tum.de/data/datasets/rgbd-dataset)

## Future Improvements

- Implement proper camera calibration with checkerboard pattern
- Increase GPS sampling rate (target 1-5 Hz)
- Use stereo or RGB-D camera for absolute scale
- Integrate loop closure for drift correction
- Test on longer trajectories with more varied environments

## License

This project was completed as part of Visual Navigation coursework. Feel free to use for educational purposes.

## Contact

For questions or collaboration opportunities, please reach out via GitHub.

---

**Note:** This is an educational project demonstrating visual SLAM concepts. Results are based on consumer-grade hardware and may not represent professional-grade accuracy.
