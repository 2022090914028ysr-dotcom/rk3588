# Release Deployment for NavOA_ws

This workspace must be deployed as a binary runtime package, not as the full source tree.
The production robot should receive only the installed runtime image under `/opt/robit/nav`, not `/home/ubuntu/Nav_Release/Robit_SlamNav/NavOA_ws`.

## Build the release package

Run this from the workspace root:

```bash
cd /home/ubuntu/Nav_Release/Robit_SlamNav/NavOA_ws
./build_release.sh
```

The script builds the ROS workspace in Release mode and creates a Debian package named `robit_nav_1.0.0_arm64.deb`.

## What gets packaged

The deployment package is built from the catkin install space and includes:

- compiled binaries in `bin/`
- libraries in `lib/`
- ROS package manifests and launch files in `share/`
- the generated `setup.bash` for the installed runtime
- a stable install prefix at `/opt/robit/nav`

## Install on a production robot

Copy the generated `.deb` to the robot and install it:

```bash
sudo dpkg -i robit_nav_1.0.0_arm64.deb
```

This installs the runtime to `/opt/robit/nav`.

## Boot auto-start service

A sample systemd unit is provided at `packaging/robit-nav.service`.
Place it on the robot as `/etc/systemd/system/robit-nav.service` and enable it:

```bash
sudo cp packaging/robit-nav.service /etc/systemd/system/
sudo systemctl daemon-reload
sudo systemctl enable robit-nav.service
sudo systemctl start robit-nav.service
```

## Verify the runtime install

On the robot, verify the install tree and launch environment:

```bash
ls /opt/robit/nav
source /opt/ros/noetic/setup.bash
source /opt/robit/nav/setup.bash
roslaunch NavOA_ros NavOA_gazebo_limo_all.launch
```

## Important production rule

Do not deploy the source workspace directory `/home/ubuntu/Nav_Release/Robit_SlamNav/NavOA_ws` to production robots.
The source tree belongs in development and version control only. Production robots should run the installed runtime image.
