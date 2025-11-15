cat > README.md << 'EOF'
# Quadruped Robot - ROS 2 Project

A ROS 2 robotics project featuring SLAM, simulation, and sensor integration.

## Features
- LIO-SAM SLAM
- Gazebo Simulation
- Velodyne LiDAR Integration
- Husky Robot Support

## Build Instructions
```bash
colcon build
source install/setup.bash
```

## Usage
```bash
ros2 launch husky_minimal husky_lio_sam_full.launch.py
```
EOF
