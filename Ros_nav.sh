#!/usr/bin/env bash
set -euo pipefail

# Ros_nav.sh (minimal, non-destructive installs)
# - Assumes conda (Miniconda/Miniforge) is already installed
# - Creates `nav` env if missing, activates it, and installs pinned packages
#   but will skip reinstalling a package if the exact requested version is already present.

ENV_NAME="nav"
PYTHON_VERSION="3.10"

PIP_INDEX_URL="https://pypi.tuna.tsinghua.edu.cn/simple"
PIP_TRUSTED_HOST="pypi.tuna.tsinghua.edu.cn"

PACKAGES=(
  "colorama==0.4.6"
  "cvxpy==1.7.4"
  "cvxpylayers==0.1.9"
  "diffcp==1.1.4"
  "dill==0.4.0"
  "ecos==2.0.10"
  "gctl==1.2"
  "ir-sim==2.7.5"
  "matplotlib==3.10.7"
  "Nuitka==2.8.6"
  "numpy==2.2.6"
  "PyYAML==6.0.3"
  "rich==14.2.0"
  "scikit-learn==1.7.2"
  "scipy==1.13.0"
  "scs==3.2.9"
  "setuptools==80.9.0"
  "torch==2.9.1"
  "zstandard==0.25.0"
)

echo "=== Ros_nav.sh (minimal, skip identical installs) ==="

if ! command -v conda >/dev/null 2>&1; then
  echo "ERROR: conda not found. Please install Miniconda/Miniforge first and re-run this script."
  echo "See: https://docs.conda.io/en/latest/miniconda.html or https://github.com/conda-forge/miniforge"
  exit 1
fi

CONDA_BASE=$(conda info --base)
# shellcheck source=/dev/null
. "$CONDA_BASE/etc/profile.d/conda.sh"

echo "Creating conda environment '$ENV_NAME' with python=$PYTHON_VERSION (if it doesn't exist)"
if conda info --envs | awk '{print $1}' | grep -qx "$ENV_NAME"; then
  echo "Conda environment '$ENV_NAME' already exists."
else
  conda create -y -n "$ENV_NAME" python="$PYTHON_VERSION"
fi

echo "Activating environment '$ENV_NAME'"
conda activate "$ENV_NAME"

echo "Upgrading pip/setuptools/wheel first (in the active env)"
python -m pip install -U pip setuptools wheel --index-url "$PIP_INDEX_URL" --trusted-host "$PIP_TRUSTED_HOST" || true

install_pkg_if_needed() {
  pkg_spec="$1"   # e.g. name==version or name
  if [[ "$pkg_spec" == *"=="* ]]; then
    name="${pkg_spec%%==*}"
    want_ver="${pkg_spec##*==}"
  else
    name="$pkg_spec"
    want_ver=""
  fi

  if python -m pip show "$name" >/dev/null 2>&1; then
    cur_ver=$(python -m pip show "$name" | awk '/^Version:/{print $2}')
    if [[ -n "$want_ver" && "$cur_ver" == "$want_ver" ]]; then
      echo "-> $name==$cur_ver already installed, skipping."
      return 0
    else
      if [[ -n "$want_ver" ]]; then
        echo "-> $name installed with version $cur_ver, requested $want_ver -> will (re)install requested version"
      else
        echo "-> $name installed with version $cur_ver, no specific version requested -> skipping"
        return 0
      fi
    fi
  else
    echo "-> $name not installed -> will install $pkg_spec"
  fi

  python -m pip install --no-cache-dir --index-url "$PIP_INDEX_URL" --trusted-host "$PIP_TRUSTED_HOST" "$pkg_spec" || {
    echo "Warning: install failed for $pkg_spec; continuing."
    return 1
  }
}

for pkg in "${PACKAGES[@]}"; do
  install_pkg_if_needed "$pkg"
done

echo "Done. If some packages failed (e.g., gazebo_ros or tf), install them via apt/rosdep as appropriate."
echo "To use the environment in a new shell:"
echo "  conda activate $ENV_NAME"

exit 0
