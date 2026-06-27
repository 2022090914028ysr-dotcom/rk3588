#!/usr/bin/env bash
set -euo pipefail

# 1. Environment and Path Declarations
ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
INSTALL_DIR="${ROOT_DIR}/install_isolated"
FAKE_ROOT="${ROOT_DIR}/fakeroot_staging"
APP_ROOT="${FAKE_ROOT}/data/app/nav"
DEB_FILE="${ROOT_DIR}/robit_nav_1.0.0_arm64.deb"
CONDA_TAR="${ROOT_DIR}/conda_control.tar.gz"

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
echo -e "\n[5/6] Preparing packaging staging tree at: ${APP_ROOT}"
rm -rf "${FAKE_ROOT}"
mkdir -p "${APP_ROOT}"

if [ ! -d "${INSTALL_DIR}" ]; then
  echo "ERROR: Expected build output directory '${INSTALL_DIR}' was not generated." >&2
  exit 1
fi

echo "Copying compiled binaries and installed artifacts..."
# Copy everything from install_isolated directly into the staging package directory
cp -a "${INSTALL_DIR}/." "${APP_ROOT}/"

# Install workspace map assets into the install tree so local roslaunch defaults can resolve them.
MAP_SOURCE_DIR="${ROOT_DIR}/src/carto_ws/map"
if [ -d "${MAP_SOURCE_DIR}" ]; then
  echo "Installing cartographer map assets from ${MAP_SOURCE_DIR} to ${APP_ROOT}/share/carto_ws/map"
  mkdir -p "${APP_ROOT}/share/carto_ws/map"
  cp -a "${MAP_SOURCE_DIR}/." "${APP_ROOT}/share/carto_ws/map/"
fi

# Bundle the conda environment using conda-pack (relocatable archive)
echo "Packing conda 'control' environment for target deployment..."
if command -v conda-pack &>/dev/null; then
  conda-pack -n control -o "${CONDA_TAR}" --compress-level 9 -q
elif [ -f "/home/ubuntu/miniconda3/envs/control/bin/conda-pack" ]; then
  /home/ubuntu/miniconda3/envs/control/bin/conda-pack -n control -o "${CONDA_TAR}" --compress-level 9 -q
else
  echo "ERROR: conda-pack not found. Install it with: conda install -n base conda-pack" >&2
  exit 1
fi
echo "Conda pack created: $(ls -lh "${CONDA_TAR}" | awk '{print $5}')"
cp -a "${CONDA_TAR}" "${APP_ROOT}/conda.tar.gz"

# Fix shebangs in auto-generated ROS scripts that point to the build machine's python
echo "Fixing shebangs and build-machine paths in auto-generated files..."
find "${APP_ROOT}" -name "*.py" -exec sed -i '1s|^#!.*python.*|#!/usr/bin/env python3|' {} \;

# Install the runtime wrapper script
echo "Installing service wrapper script..."
install -m 0755 "${ROOT_DIR}/packaging/run_robit_nav.sh" "${APP_ROOT}/run_robit_nav.sh"

# Install deployment documentation
echo "Installing deployment documentation..."
install -m 0644 "${ROOT_DIR}/docs/release-deployment.md" "${APP_ROOT}/release-deployment.md"

# Install the systemd service unit
echo "Installing systemd service unit..."
mkdir -p "${FAKE_ROOT}/etc/systemd/system"
cp -a "${ROOT_DIR}/packaging/robit-nav.service" "${FAKE_ROOT}/etc/systemd/system/robit-nav.service"

# 6. Construct Debian control manifests and packaging
echo -e "\n[6/6] Constructing Debian control manifests and packaging..."
mkdir -p "${FAKE_ROOT}/DEBIAN"
cat > "${FAKE_ROOT}/DEBIAN/control" <<'EOF'
Package: robit-nav
Version: 1.0.0
Section: robotics
Priority: optional
Architecture: arm64
Depends: ros-noetic-ros-core, systemd
Maintainer: Robit Release Team <noreply@example.com>
Description: Robit ROS navigation runtime package (userdata partition)
 Installed to /data/app/nav/ for OTA-updateable userdata partition.
 Bundles the 'control' conda environment (Python 3.10, PyTorch, NavOA stack).
 Contains the compiled navigation runtime for the RK3588 platform.
EOF

# Post-install script: extract conda environment, enable and start the service
cat > "${FAKE_ROOT}/DEBIAN/postinst" <<'POSTINST'
#!/bin/sh
set -e

# Extract bundled conda environment to /data/app/nav/conda/
CONDA_TARGET="/data/app/nav/conda"
if [ -f "/data/app/nav/conda.tar.gz" ]; then
  echo "robit-nav: Extracting bundled conda environment (this may take a minute)..."
  mkdir -p "${CONDA_TARGET}"
  tar -xzf "/data/app/nav/conda.tar.gz" -C "${CONDA_TARGET}"
  echo "robit-nav: Finalizing conda environment relocation..."
  ln -sf python3 "${CONDA_TARGET}/bin/python"
  PATH="${CONDA_TARGET}/bin:${PATH}" "${CONDA_TARGET}/bin/conda-unpack"
  rm -f "/data/app/nav/conda.tar.gz"
  echo "robit-nav: Conda environment ready at ${CONDA_TARGET}"
fi

if command -v systemctl >/dev/null 2>&1; then
  systemctl daemon-reload
  systemctl enable robit-nav.service
  systemctl restart robit-nav.service
fi
exit 0
POSTINST
chmod 0755 "${FAKE_ROOT}/DEBIAN/postinst"

# Pre-remove script: stop and disable the service
cat > "${FAKE_ROOT}/DEBIAN/prerm" <<'PRERM'
#!/bin/sh
set -e
if command -v systemctl >/dev/null 2>&1; then
  systemctl stop robit-nav.service || true
  systemctl disable robit-nav.service || true
fi
exit 0
PRERM
chmod 0755 "${FAKE_ROOT}/DEBIAN/prerm"

echo "Compiling debian payload container via fakeroot..."
fakeroot dpkg-deb --build "${FAKE_ROOT}" "${DEB_FILE}"

# Post-build garbage collection
rm -rf "${FAKE_ROOT}"

echo -e "\n================================================================="
echo " Production Package Complete: ${DEB_FILE}"
echo " Target Architecture: ARM64 (Native RK3588 Deployment)"
echo "================================================================="
