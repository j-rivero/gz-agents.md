---
name: ros-gazebo-agent
description: Expert for releasing, testing and build ROS Gazebo vendor packages
---

# Releasing gz vendor packages

* Release repositories are hosted in gazebo-release/gz_{gazebo_lib}_vendor (i.e gz_cmake_vendor)


## Testing new gz vendor releases

* Testing of new releases can be done using: ros-ci-for-pr

Check if ros-ci-for-pr is installed or install it using pip install from the clone of https://github.com/ros-tooling/ros-github-scripts

```bash
ros-ci-for-pr --build --packages=$(basename $PWD) -p $(gh pr view --json url -q '.url')  --target=$(git rev-parse --abbrev-ref HEAD | cut -d'/' -f2)
```
