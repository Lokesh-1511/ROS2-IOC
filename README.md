# Hexapod ROS2 Workspace

This repository contains the ROS2 workspace for a hexapod robot simulation and control.

## Overview

The hexapod workspace includes packages for robot description, visualization, and control of a six-legged walking robot.

## Structure

```
hexapod_ws/
├── src/
│   └── hexapod_description/    # Robot URDF description and launch files
├── build/                       # Build artifacts (excluded from git)
├── install/                     # Install space (excluded from git)
└── log/                         # Build logs (excluded from git)
```

## Prerequisites

- ROS2 (Humble/Iron/Jazzy or later)
- Python 3.8+
- colcon build tool
- rviz2

## Installation

1. Clone this repository:
   ```bash
   git clone https://github.com/Lokesh-1511/ROS2-IOC.git
   cd ROS2-IOC
   ```

2. Install dependencies:
   ```bash
   rosdep install --from-paths src --ignore-src -r -y
   ```

3. Build the workspace:
   ```bash
   colcon build
   ```

4. Source the workspace:
   ```bash
   source install/setup.bash
   ```

## Usage

### Launch Robot Visualization

To visualize the hexapod robot in RViz:

```bash
ros2 launch hexapod_description display.launch.py
```

## Packages

### hexapod_description

Contains the URDF/XACRO files describing the hexapod robot's physical structure, including:
- Robot geometry and kinematics
- Joint definitions
- Visual and collision meshes
- Launch files for visualization

## Development

### Building

To build the workspace after making changes:

```bash
colcon build --symlink-install
```

### Testing

Run tests with:

```bash
colcon test
```

## Contributing

Contributions are welcome! Please feel free to submit a Pull Request.

## License

[Add your license information here]

## Author

Lokesh-1511

## Acknowledgments

- ROS2 Community
- Open Robotics
