# ROS 2 workspace

`src/` contains the project-owned ROS 2 Jazzy packages. Generated `build/`, `install/`, and `log/` directories stay inside this workspace and are ignored by Git.

Build and test from the repository root through `just`; the recipes establish consistent colcon and Ninja arguments.
