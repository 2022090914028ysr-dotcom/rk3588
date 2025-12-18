# ROS setup for this repository (NavOA)

This document gives a concise, tested-first set of steps to get the NavOA repository usable on a Linux machine with ROS (catkin) and a local Python development environment.

Target files and folders referenced from repository root:
- `NavOA/` — Python package (has `pyproject.toml`).
- `NavOA_ws/` — catkin workspace containing `NavOA_ros` and other ROS packages.
- `slam/` — standalone C++ SLAM project (built with CMake) — optional.

If you want me to run these on your machine, run the detection commands in a terminal and paste the outputs into the issue/PR or chat:

```bash
# distro & python info
lsb_release -a || cat /etc/os-release
python3 --version
which conda || echo "no conda"
which roslaunch || echo "roslaunch not found"
rosversion -d || echo "no rosversion available"
```

---

## 1. High-level plan

1. Create a conda environment for NavOA (Python 3.10+).
2. Install Python dependencies and install the `NavOA` package editable.
3. Ensure system ROS is installed for your Ubuntu/ROS distro and source it.
4. Build the catkin workspace `NavOA_ws` and source the workspace.
5. Run quick sanity checks: import the Python package and launch a ROS example.

> Note: This repository mixes system ROS (catkin) and a developer Python environment. We recommend using a conda env for Python dev and keeping ROS installed via the standard OS packages.

---

## 2. Create & prepare a conda environment (recommended)

If you don't have conda, you can use virtualenv, but conda is used in the project's README.

```bash
# create env with Python 3.10
conda create -n nav python=3.10 -y
conda activate nav

# upgrade packaging tools
pip install -U pip setuptools wheel
```

Install PyTorch (choose CPU or GPU wheel per your machine):

```bash
# CPU-only example (fast):
pip install "torch>=2.1.0" --index-url https://download.pytorch.org/whl/cpu

# OR follow https://pytorch.org to install matching CUDA build if you have GPU
```

Install the rest of the Python deps listed in `NavOA/pyproject.toml`:

```bash
pip install cvxpylayers numpy scipy rich dill gctl==1.2 colorama scikit-learn pyyaml ecos

# Install local package in editable mode (from repo root):
pip install -e ./NavOA

# quick check
python -c "import NavOA, torch, numpy; print('NavOA', getattr(NavOA,'__file__',None)); print('torch', torch.__version__); print('numpy', numpy.__version__)"
```

If any package fails to install, you may need system build tools:

```bash
sudo apt update
sudo apt install -y build-essential cmake python3-dev libblas-dev liblapack-dev libatlas-base-dev
```

Specific packages that sometimes need extra steps:
- `cvxpylayers` may need `C++` toolchain and `CMake`.
- `ecos` may compile native code; install via apt if available or ensure compilers are present.

---

## 3. Install / verify ROS (system packages)

If `roslaunch` is not present, install the appropriate ROS distro for your OS (e.g., Noetic for Ubuntu 20.04). See ROS installation docs for your distro. After installing, verify with:

```bash
rosversion -d
which roslaunch
```

Make sure `rosdep` is initialized (first-time only):

```bash
sudo rosdep init
rosdep update
```

---

## 4. Build the catkin workspace `NavOA_ws`

From the repository root:

```bash
cd NavOA_ws
# install system deps declared in package.xml files
rosdep install --from-paths src --ignore-src -r -y

# build the workspace
catkin_make

# after build, source the workspace so ROS can find packages
source devel/setup.bash
```

If the project uses `catkin_tools` instead of `catkin_make`, run:

```bash
# install catkin tools if needed
sudo apt install python3-catkin-tools
catkin build
source devel/setup.bash
```

If you use a conda env for development, activate it first and then source ROS so tools and Python packages are available together:

```bash
conda activate nav
source /opt/ros/<your-distro>/setup.bash
source /path/to/Robit_SlamNav/NavOA_ws/devel/setup.bash
```

Replace `<your-distro>` with the output of `rosversion -d` (for example `noetic`).

### Build notes
- To build only one package:

```bash
catkin_make --pkg NavOA_ros
```

- To clean and rebuild:

```bash
catkin_make clean
rm -rf build/ devel/
catkin_make
```

---

## 5. Quick-run sanity checks

1) Python import (from any terminal where `pip install -e ./NavOA` ran):

```bash
python -c "import NavOA; print('OK NavOA from', NavOA.__file__)"
```

2) Start ROS master and run a launch file from README

```bash
# in terminal A
source /opt/ros/<your-distro>/setup.bash
source NavOA_ws/devel/setup.bash
roscore &

# in terminal B
source /opt/ros/<your-distro>/setup.bash
source NavOA_ws/devel/setup.bash
roslaunch NavOA_ros NavOA_gazebo_limo.launch
```

3) Run a helper script (example):

```bash
# after sourcing as above, you can run scripts in NavOA_ws/src/NavOA_ros/scripts
python NavOA_ws/src/NavOA_ros/scripts/real_robot_cmd/SendCommands.py --help
```

4) If you see Python import errors for `rospy`, ensure the shell has ROS sourced and that `PYTHONPATH` contains ROS python packages.

---

## 6. Optional: build the C++ SLAM project

The `slam/` folder is a separate CMake project. To build it:

```bash
cd slam
mkdir -p build && cd build
cmake ..
make -j4
```

If the CMake step fails, install missing `-dev` system packages (see error messages).

---

## 7. Troubleshooting tips (common issues)

- roslaunch not found: ROS not installed or not sourced. Source `/opt/ros/<distro>/setup.bash`.
- Permission or shebang errors: ensure script files are executable: `chmod +x script.py` and have `#!/usr/bin/env python3`.
- Python version mismatch: ROS may be using system Python; prefer `conda activate nav` then `source /opt/ros/<distro>/setup.bash` so ROS python modules are visible to the env.
- Dependency install fails: use `rosdep` for apt deps and install build essentials: `sudo apt install build-essential cmake`.
- GPU/Torch: install the correct PyTorch wheel for your CUDA version or use CPU wheel for maximum compatibility.

---

## 8. Helpful commands recap

```bash
# source ROS and workspace
source /opt/ros/<distro>/setup.bash
source NavOA_ws/devel/setup.bash

# build
catkin_make

# run
roslaunch NavOA_ros NavOA_gazebo_limo.launch

# python package dev
conda activate nav
pip install -e ./NavOA
python -c "import NavOA; print(NavOA)"
```


Last updated: Dec 11, 2025
