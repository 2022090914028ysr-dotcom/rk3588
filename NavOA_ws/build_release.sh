#!/usr/bin/env bash
set -euo pipefail

# 1. Environment and Path Declarations
ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
INSTALL_DIR="${ROOT_DIR}/install_isolated"
FAKE_ROOT="${ROOT_DIR}/fakeroot_staging"
PACKAGE_ROOT="${FAKE_ROOT}/opt/robit/nav"
DEB_FILE="${ROOT_DIR}/robit_nav_1.0.0_arm64.deb"

echo "================================================================="
echo " Starting Commercial Release Build Sequence (ARM64 / RK3588)"
echo "================================================================="

# 2. Runtime Environment Sanity Checks
if [ -z "${CONDA_DEFAULT_ENV:-}" ] || [ "${CONDA_DEFAULT_ENV}" != "control" ]; then
  echo "ERROR: This script must be executed within the active 'control' Conda environment." >&2
  echo "Please run: conda activate control" >&2
  exit 1
fi

if [ -z "${ROS_DISTRO:-}" ]; then
  echo "ERROR: ROS environment is not sourced. Please source your underlying ROS setup.bash first." >&2
  exit 1
fi

# 3. Clean and Compile Stage
echo -e "\n[1/5] Building ROS workspace via Isolated Ninja Pipeline (Release Mode)..."
cd "${ROOT_DIR}"

# Force isolated ninja build using flags optimized for embedded execution speed
catkin_make_isolated --use-ninja --install -DCMAKE_BUILD_TYPE=Release

# 4. Construct Deployment File Tree
echo -e "\n[2/5] Preparing packaging staging tree at: ${PACKAGE_ROOT}"
rm -rf "${FAKE_ROOT}"
mkdir -p "${PACKAGE_ROOT}"

if [ ! -d "${INSTALL_DIR}" ]; then
  echo "ERROR: Expected build output directory '${INSTALL_DIR}' was not generated." >&2
  exit 1
fi

echo "[3/5] Copying compiled binaries and obfuscated artifacts..."
# Copy everything from install_isolated directly into the staging package directory
cp -a "${INSTALL_DIR}/." "${PACKAGE_ROOT}/"

# 5. Build Control Configuration
echo "[4/5] Constructing Debian control manifests..."
mkdir -p "${FAKE_ROOT}/DEBIAN"
cat > "${FAKE_ROOT}/DEBIAN/control" <<'EOF'
Package: robit-nav
Version: 1.0.0
Section: robotics
Priority: optional
Architecture: arm64
Depends: ros-noetic-ros-core
Maintainer: Robit Release Team <noreply@example.com>
Description: Robit ROS navigation runtime package
 A production package containing the compiled, obfuscated navigation runtime for the RK3588 platform. Contains no high-level raw source blocks.
EOF

# 6. Immutable Binary Compression
echo "[5/5] Compiling debian payload container via fakeroot..."
fakeroot dpkg-deb --build "${FAKE_ROOT}" "${DEB_FILE}"

# Post-build garbage collection
rm -rf "${FAKE_ROOT}"

echo -e "\n================================================================="
echo " Production Package Complete: ${DEB_FILE}"
echo " Target Architecture: ARM64 (Native RK3588 Deployment)"
echo "================================================================="