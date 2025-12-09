# waypoint_manager

ROS 2 workspace for waypoint-driven navigation and visualization utilities. The repository groups several packages that cooperate to load waypoint definitions, dispatch navigation goals, run waypoint-scoped functions, and visualize progress.

## Repository Layout

- `waypoint_manager_utils`: Shared utilities for loading waypoint data from CSV files into typed structures.
- `waypoint_navigator`: Component-based node that publishes waypoint goals, interacts with Nav2, and orchestrates waypoint-specific command execution.
- `waypoint_visualizer`: RViz-facing node that renders loaded waypoints and highlights the active waypoint.
- `waypoint_function_server`: Core servers and example implementations for executing custom waypoint functions.
- `waypoint_function_msgs`: Service definitions for inter-package command exchange.
- `waypoint_rviz_plugins`: RViz tool extensions used alongside the visualizer.

Each package follows standard ROS 2 `ament_cmake` structure and can be built together or individually with `colcon`.

## Quick Start

```bash
colcon build --symlink-install
source install/setup.bash
```

### Launch Navigation Stack

```bash
ros2 launch waypoint_manager/launch/waypoint_manager.launch.py waypoints:="path/to/waypoints.csv" start_id:=0
```

Adjust parameters in the `config/` directories (for example `config_waypoint_navigator.yaml`) to set the waypoint CSV path, loop behavior, and visualization frame ID. The visualizer node is included in `waypoint_manager.launch.py`, so a separate launch is usually unnecessary.

## Waypoint CSV Format

The shared utility expects CSV files with the following columns:

1. Waypoint ID (integer)
2. Position X (meters)
3. Position Y (meters)
4. Position Z (meters)
5. Orientation X (quaternion)
6. Orientation Y (quaternion)
7. Orientation Z (quaternion)
8. Orientation W (quaternion)
9. Optional command strings (additional columns)

Rows after the header are converted into `waypoint_manager_utils::Waypoint` instances, exposing both pose data and any attached command strings.

### Command String Syntax

- Each additional column after the pose fields is treated as one command token.
- Tokens are passed, in order, to the waypoint function server via the `waypoint_function_msgs/srv/Command` service.
- Use simple ASCII strings such as `pause`, `skip`, or `wait:3.0`. Parsing of arguments (for example splitting on `:`) should be implemented inside the corresponding function server node.
- Leave cells empty when no command is required; empties are ignored automatically.

## Development Notes

- All packages rely on `rclcpp` components; prefer `ros2 run` with `--ros-args` or use provided launches.
- Custom waypoint functions should extend the base classes in `waypoint_function_server` and register via `RCLCPP_COMPONENTS_REGISTER_NODE`.
- The repository targets ROS 2 Humble or newer.

## License

Each package declares its own license (see individual `package.xml` files). The default license is Apache 2.0 unless stated otherwise.
