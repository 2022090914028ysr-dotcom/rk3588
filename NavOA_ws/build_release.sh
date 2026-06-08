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

# 3. Pre-build Encoding Sanity (fix any ISO-8859 or other non-UTF-8 source files)
echo -e "\n[3/6] Scanning source tree for non-UTF-8 encoded text files and converting to UTF-8..."
NON_UTF8_FILES=$(find "${ROOT_DIR}/src" -type f \( -name "*.yaml" -o -name "*.yml" -o -name "*.launch" -o -name "*.xml" -o -name "*.py" -o -name "*.cpp" -o -name "*.h" -o -name "*.txt" -o -name "*.cfg" -o -name "*.json" \) -exec sh -c '
  for f; do
    enc=$(file -b "$f")
    if echo "$enc" | grep -qiE "ISO-8859|CP1252|LATIN"; then
      echo "$f"
    fi
  done
' sh {} + 2>/dev/null)
if [ -n "$NON_UTF8_FILES" ]; then
  echo "Found non-UTF-8 files, converting to UTF-8:"
  echo "$NON_UTF8_FILES"
  echo "$NON_UTF8_FILES" | while read -r f; do
    echo "  Converting: $f"
    iconv -f ISO-8859-1 -t UTF-8 "$f" -o "${f}.utf8" && mv "${f}.utf8" "$f"
  done
  echo "Conversion complete."
else
  echo "No non-UTF-8 text files found in source tree."
fi

# 4. Compile Stage
echo -e "\n[4/6] Building ROS workspace via Isolated Ninja Pipeline (Release Mode)..."
cd "${ROOT_DIR}"

# Force isolated ninja build using flags optimized for embedded execution speed
catkin_make_isolated --use-ninja --install -DCMAKE_BUILD_TYPE=Release

# 5. Construct Deployment File Tree
echo -e "\n[5/6] Preparing packaging staging tree at: ${PACKAGE_ROOT}"
rm -rf "${FAKE_ROOT}"
mkdir -p "${PACKAGE_ROOT}"

if [ ! -d "${INSTALL_DIR}" ]; then
  echo "ERROR: Expected build output directory '${INSTALL_DIR}' was not generated." >&2
  exit 1
fi

echo "Copying compiled binaries and installed artifacts..."
# Copy everything from install_isolated directly into the staging package directory
cp -a "${INSTALL_DIR}/." "${PACKAGE_ROOT}/"

# Install workspace map assets into the install tree so local roslaunch defaults can resolve them.
MAP_SOURCE_DIR="${ROOT_DIR}/src/carto_ws/map"
if [ -d "${MAP_SOURCE_DIR}" ]; then
  echo "Installing cartographer map assets from ${MAP_SOURCE_DIR} to ${PACKAGE_ROOT}/share/carto_ws/map"
  mkdir -p "${PACKAGE_ROOT}/share/carto_ws/map"
  cp -a "${MAP_SOURCE_DIR}/." "${PACKAGE_ROOT}/share/carto_ws/map/"
fi

# 6. Construct Debian control manifests and Compile deb
echo -e "\n[6/6] Constructing Debian control manifests and packaging..."
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

echo "Compiling debian payload container via fakeroot..."
fakeroot dpkg-deb --build "${FAKE_ROOT}" "${DEB_FILE}"

# Post-build garbage collection
rm -rf "${FAKE_ROOT}"

echo -e "\n================================================================="
echo " Production Package Complete: ${DEB_FILE}"
echo " Target Architecture: ARM64 (Native RK3588 Deployment)"
echo "================================================================="
