# Exercise 3: AprilTags, Transforms, and CSV Path Generation

## Overview

This project processes a recorded ROS2 bag file containing AprilTag detections to track the path of a vacuum cleaner (VC) relative to a charging station (CS). The system uses a service camera on the ceiling to detect AprilTags attached to the floor, charging station, and vacuum cleaner.

### Problem Statement

During the night, the vacuum cleaner ran out of power and stopped far from the charging station. The movement was recorded by a service camera. This exercise aims to:
- Find where the vacuum cleaner stopped
- Calculate how far it is from the charging station
- Visualize the complete path taken by the robot

### AprilTag Mapping

- **tag36h11:0** → Floor reference frame (coordinate system origin)
- **tag36h11:1** → Charging Station (CS)
- **tag36h11:2** → Vacuum Cleaner (VC)

## Project Structure

```
exercise-3/
├── es_bag2/                    # ROS2 bag file containing recorded data
│   ├── es_bag2.mcap
│   └── metadata.yaml
├── tag_path_generator/         # ROS2 package
│   ├── src/
│   │   └── path_saver.cpp      # Main node that processes transforms
│   ├── CMakeLists.txt
│   ├── package.xml
│   └── vc_path.csv             # Recorded CSV file with path data (pre-recorded)
├── plot_path.py                # Python script for visualization
└── README.md                   # This file
```

**Note**: A pre-recorded `vc_path.csv` file is already available in `tag_path_generator/` directory. This contains the path data that was recorded from the bag file. You can use this file directly for visualization, or record a new one by following the steps below.

## How It Works

### 1. Transform Processing (C++ Node)

The `path_saver` node:
- Listens to `/tf` topic from the bag file
- Extracts transforms for both CS and VC relative to the floor frame (`tag36h11:0`)
- Computes VC's position relative to CS using transform composition:
  ```
  T_CS_VC = T_Floor_CS^(-1) × T_Floor_VC
  ```
- Projects the path onto the floor plane (z=0)
- Saves the path to `vc_path.csv` with timestamps

### 2. Data Format

The CSV file contains:
- **time**: Timestamp from the bag (seconds since epoch)
- **x**: X position relative to CS (meters)
- **y**: Y position relative to CS (meters)
- **z**: Always 0 (projected to floor plane)

### 3. Visualization (Python Script)

The `plot_path.py` script:
- Loads the CSV file
- Identifies the final VC position (last unique position before robot stopped)
- Calculates the straight-line distance from final position to CS
- Creates a visualization showing:
  - Complete VC path (blue line)
  - Charging Station location with coordinates
  - Final VC position
  - Straight-line distance (red dashed line)

## Prerequisites

- ROS2 (tested with Jazzy)
- Python 3 with packages:
  - `pandas`
  - `matplotlib`
  - `numpy`

## Building the Package

```bash
cd tag_path_generator
colcon build
source install/setup.bash
```

## Running the Exercise

### Step 1: Play the Bag File

In **Terminal 1**, play the bag file with clock synchronization:

```bash
cd exercise-3
ros2 bag play es_bag2 --clock
```

The `--clock` flag synchronizes ROS time with the bag's timestamps.

### Step 2: Run the Path Saver Node

In **Terminal 2**, run the path tracking node:

```bash
cd exercise-3/tag_path_generator
source install/setup.bash
ros2 run tag_path_generator path_saver
```

The node will:
- Print the charging station location (once)
- Continuously save the VC path to `vc_path.csv`
- Stop when you press `Ctrl+C` (or when the bag finishes)

**Note**: The node will keep running after the bag finishes, recording the last known position. You can stop it manually with `Ctrl+C`.

### Step 3: Visualize the Path

After the bag has finished playing and you've stopped the node, run the visualization script:

```bash
cd exercise-3
python3 plot_path.py
```

**Alternative**: If you want to use the pre-recorded CSV file instead of recording a new one, you can skip Steps 1-2 and directly run the visualization script. The script will use the existing `tag_path_generator/vc_path.csv` file.

This will:
- Display the path visualization
- Print to console:
  - Final VC position
  - CS position
  - Distance from CS

## Understanding the Output

### Charging Station Location

When the node starts, it prints the CS location in the floor frame:
```
=== Charging Station Location ===
Frame: tag36h11:1 (Charging Station)
Reference Frame: tag36h11:0 (Floor)
Position (x, y, z): 0.1001, 3.2497, 1.1062
```

This is the **absolute position** of the CS in the floor's coordinate system.

### Path Coordinates

The CSV file contains positions **relative to the charging station**. This means:
- CS is at (0, 0) in the path coordinate system
- All VC positions are relative to CS
- The final position tells you where the VC stopped relative to CS

### Distance Calculation

The distance from the final VC position to CS is calculated as:
```
distance = sqrt(x_final² + y_final²)
```

This is the straight-line (Euclidean) distance in the floor plane.

## Key Implementation Details

### Transform Lookup

The node uses `tf2::TimePointZero` to get the latest available transform from the TF buffer. This ensures it always gets the most recent transform when playing a bag file.

### Projection to Floor Plane

All z-coordinates are set to 0, projecting the 3D path onto the 2D floor plane for visualization.

### Final Position Detection

The Python script identifies the final position by finding the last row where the position actually changed, ignoring duplicate entries that occur after the bag finishes playing.

## Troubleshooting

### No Data in CSV

If the CSV only contains the header:
- Make sure the bag is playing (`ros2 bag play es_bag2 --clock`)
- Check that transforms are being published: `ros2 topic echo /tf`
- Verify the node is running and not throwing transform exceptions

### Transform Errors

If you see warnings like "TransformException":
- Ensure the bag is playing with `--clock` flag
- Wait a few seconds for transforms to populate the buffer
- Check that all three tags (0, 1, 2) are present in the bag

### Visualization Issues

If the plot doesn't show correctly:
- Verify the CSV file has data (more than just header)
- Check that `pandas`, `matplotlib`, and `numpy` are installed
- Ensure the CSV path in `plot_path.py` is correct

## Results

After running the complete exercise, you should have:
1. A CSV file (`vc_path.csv`) with the complete path
2. A visualization showing:
   - The path the VC took
   - Where it stopped (final position)
   - How far it is from the charging station
3. Console output with exact coordinates and distance

## Files Generated

- `tag_path_generator/vc_path.csv`: Path data in CSV format (pre-recorded, or generated when running the node)
- Visualization plot (displayed when running `plot_path.py`)

**Pre-recorded Data**: The `vc_path.csv` file in `tag_path_generator/` directory contains the recorded path data from processing the bag file. This file can be used directly for visualization without needing to replay the bag.

## Notes

- The path is recorded at 10 Hz (every 100ms)
- All coordinates are in meters
- The path is relative to the charging station (CS at origin)
- The z-coordinate is always 0 (floor plane projection)

