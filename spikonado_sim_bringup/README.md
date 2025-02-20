# spikonado_sim_bringup

This package contains bringup files needed to simulate the Spikonado robot.

You can launch the full simulation in the following way:

```sh
ros2 launch spikonado_sim_bringup full_sim.launch.py
```

`simulator.launch.py` just launches the Gazebo simulator.
`spawner.launch.py` spawns the robot into the Gazebo world.
`full_sim.launch.py` executes both of the operations above.

## Purpose of each directory

- `/config`: Contains config files that are used by various nodes.
- `/launch`: Contains launch files for launching the simulation.
- `/worlds`: Contains various simulation worlds for the robot.
