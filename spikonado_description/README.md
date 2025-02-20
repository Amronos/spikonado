# spikonado_description

This package contains description files for the Spikonado robot in both URDF and SDF alongside the meshes (and Blender project files for them) needed for the description.

The `display.launch.py` launch file can be used for easy testing of the description.

```sh
ros2 launch spikonado_description display.launch.py
```

Also contains some config files, hardware descriptions, and simulator descriptions used by other Spikonado packages.

## Purpose of each directory

- `/config`: Contains config files that are used by various nodes.
  Some config files needed by other Spikonado packages are also included.
- `/description`: The main directory of the package, contains various description files for the robot in both URDF and SDF.
- `/hooks`: Contains environment hooks for editing environment variables.
- `/launch`: Contains launch files that can be used to test the robot descriptions.
- `/meshes`: Contains mesh (.stl) files of the various parts of the robot, alongside the Blender project files used to create them.
