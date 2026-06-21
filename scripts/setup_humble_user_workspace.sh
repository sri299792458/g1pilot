#!/usr/bin/env bash
set -euo pipefail

ROS_DISTRO="${ROS_DISTRO:-humble}"
WS="${G1PILOT_WS:-${HOME}/g1pilot_ws}"
REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
JOBS="${JOBS:-4}"

EXTERNAL_DIR="${WS}/external"
UNITREE_SDK_DIR="${EXTERNAL_DIR}/unitree_sdk2_python"
UNITREE_SDK_REPO="${UNITREE_SDK_REPO:-https://github.com/lnotspotl/unitree_sdk2_python.git}"
UNITREE_SDK_COMMIT="${UNITREE_SDK_COMMIT:-7c661d27f4ae064ffd0dd633fd9d5b518ef0b508}"
ASTROVIZ_REPO="${ASTROVIZ_REPO:-https://github.com/CDonosoK/astroviz_interfaces.git}"

FOREST_WS="${WS}/deps/forest_ws"
SRC_DIR="${WS}/deps/src"
BUILD_DIR="${WS}/deps/build"
DEPS_PREFIX="${FOREST_WS}/install"

BUILD_OPENSOT=1
BUILD_LIVOX=1
BUILD_REALSENSE_PY=1

for arg in "$@"; do
  case "${arg}" in
    --skip-opensot)
      BUILD_OPENSOT=0
      ;;
    --skip-livox)
      BUILD_LIVOX=0
      ;;
    --skip-realsense-python)
      BUILD_REALSENSE_PY=0
      ;;
    *)
      echo "Unknown option: ${arg}"
      echo "Usage: $0 [--skip-opensot] [--skip-livox] [--skip-realsense-python]"
      exit 1
      ;;
  esac
done

if [ "${ROS_DISTRO}" != "humble" ]; then
  echo "This setup script is for ROS 2 Humble. Current ROS_DISTRO=${ROS_DISTRO}"
  exit 1
fi

if [ ! -f "/opt/ros/${ROS_DISTRO}/setup.bash" ]; then
  echo "ROS 2 Humble was not found at /opt/ros/${ROS_DISTRO}."
  exit 1
fi

clone_checkout() {
  local repo="$1"
  local dir="$2"
  local ref="${3:-}"
  if [ ! -d "${dir}/.git" ]; then
    git clone "${repo}" "${dir}"
  fi
  git -C "${dir}" fetch --tags --quiet
  if [ -n "${ref}" ]; then
    git -C "${dir}" checkout "${ref}"
  fi
  git -C "${dir}" submodule update --init --recursive
}

cmake_install() {
  local name="$1"
  local src="$2"
  shift 2
  local build="${BUILD_DIR}/${name}"
  mkdir -p "${build}"
  cmake -S "${src}" -B "${build}" \
    -DCMAKE_INSTALL_PREFIX:STRING="${DEPS_PREFIX}" \
    -DCMAKE_BUILD_TYPE:STRING=Release \
    "$@"
  cmake --build "${build}" -j"${JOBS}"
  cmake --install "${build}"
}

setup_workspace() {
  mkdir -p "${WS}/src" "${EXTERNAL_DIR}" "${FOREST_WS}" "${SRC_DIR}" "${BUILD_DIR}"

  if [ ! -e "${WS}/src/g1pilot" ]; then
    ln -s "${REPO_ROOT}" "${WS}/src/g1pilot"
    echo "Linked ${REPO_ROOT} -> ${WS}/src/g1pilot"
  elif [ "$(realpath "${WS}/src/g1pilot")" != "${REPO_ROOT}" ]; then
    echo "Warning: ${WS}/src/g1pilot already points somewhere else:"
    echo "  $(realpath "${WS}/src/g1pilot")"
    echo "Continuing without changing it."
  fi

  if [ ! -d "${WS}/src/astroviz_interfaces/.git" ]; then
    git clone "${ASTROVIZ_REPO}" "${WS}/src/astroviz_interfaces"
  fi
}

setup_python_env() {
  python3 -m venv --system-site-packages "${WS}/.venv"
  # shellcheck disable=SC1091
  source "${WS}/.venv/bin/activate"
  rm -f "${WS}/.venv/bin/register-python-argcomplete" 2>/dev/null || true

  python -m pip install -U pip "setuptools<80" wheel
  python -m pip install -U \
    "catkin_pkg" \
    "colcon-common-extensions" \
    "cyclonedds==0.10.2" \
    "empy" \
    "evdev" \
    "hhcm-forest" \
    "hidapi" \
    "lark" \
    "matplotlib" \
    "meshcat" \
    "mujoco" \
    "numpy==1.26.4" \
    "opencv-python<4.12" \
    "pyqt6" \
    "pyspacemouse" \
    "robot_descriptions" \
    "rich-click" \
    "ttictoc"
}

setup_unitree_sdk() {
  if [ ! -d "${UNITREE_SDK_DIR}/.git" ]; then
    git clone "${UNITREE_SDK_REPO}" "${UNITREE_SDK_DIR}"
  fi
  git -C "${UNITREE_SDK_DIR}" fetch --tags --quiet
  git -C "${UNITREE_SDK_DIR}" checkout "${UNITREE_SDK_COMMIT}"
  python -m pip install --no-deps -e "${UNITREE_SDK_DIR}"

  local crc_src="${UNITREE_SDK_DIR}/unitree_sdk2py/utils/lib/crc_amd64.so"
  if [ -f "${crc_src}" ]; then
    local unitree_site
    unitree_site="$(python - <<'PY'
import pathlib
import unitree_sdk2py
print(pathlib.Path(unitree_sdk2py.__file__).resolve().parent)
PY
)"
    mkdir -p "${unitree_site}/utils/lib"
    cp "${crc_src}" "${unitree_site}/utils/lib/"
    chmod +x "${unitree_site}/utils/lib/crc_amd64.so"
  fi
}

setup_build_env() {
  # shellcheck disable=SC1091
  source "/opt/ros/${ROS_DISTRO}/setup.bash"
  # shellcheck disable=SC1091
  source "${WS}/.venv/bin/activate"

  export HHCM_FOREST_CLONE_DEFAULT_PROTO=https
  export CMAKE_PREFIX_PATH="${DEPS_PREFIX}:${CMAKE_PREFIX_PATH:-}"
  export LD_LIBRARY_PATH="${DEPS_PREFIX}/lib:${DEPS_PREFIX}/lib/x86_64-linux-gnu:${LD_LIBRARY_PATH:-}"
  export PKG_CONFIG_PATH="${DEPS_PREFIX}/lib/pkgconfig:${DEPS_PREFIX}/lib/x86_64-linux-gnu/pkgconfig:${PKG_CONFIG_PATH:-}"
  export PYTHONPATH="${DEPS_PREFIX}/lib/python3.10/site-packages:${DEPS_PREFIX}/local/lib/python3.10/dist-packages:${PYTHONPATH:-}"
}

ensure_forest_matlogger2() {
  cd "${FOREST_WS}"
  if [ ! -f "${FOREST_WS}/setup.bash" ]; then
    forest init
  fi
  if [ ! -f "${FOREST_WS}/.g1pilot_multidof_recipes_added" ]; then
    if forest add-recipes https://github.com/advrhumanoids/multidof_recipes.git --tag ros2; then
      touch "${FOREST_WS}/.g1pilot_multidof_recipes_added"
    fi
  fi
  forest grow matlogger2 --verbose --jobs "${JOBS}" --pwd user
  # shellcheck disable=SC1091
  source "${FOREST_WS}/setup.bash"
}

build_opensot_stack() {
  ensure_forest_matlogger2

  clone_checkout https://github.com/humanoid-path-planner/hpp-fcl.git \
    "${SRC_DIR}/hpp-fcl" 45e60ca7ba81e5394605f8c1097c016245d221c2
  cmake_install hpp-fcl "${SRC_DIR}/hpp-fcl" \
    -DBUILD_PYTHON_INTERFACE=OFF

  clone_checkout https://github.com/stack-of-tasks/pinocchio.git \
    "${SRC_DIR}/pinocchio" v3.9.0
  cmake_install pinocchio "${SRC_DIR}/pinocchio" \
    -DBUILD_WITH_URDF_SUPPORT=ON \
    -DBUILD_WITH_COLLISION_SUPPORT=ON \
    -DBUILD_TESTING=FALSE \
    -DBUILD_PYTHON_INTERFACE=OFF

  clone_checkout https://github.com/ADVRHumanoids/xbot2_interface.git \
    "${SRC_DIR}/xbot2_interface" devel
  cmake_install xbot2_interface "${SRC_DIR}/xbot2_interface" \
    -DXBOT2_IFC_BUILD_TESTS=ON \
    -DXBOT2_IFC_BUILD_ROS=OFF \
    -DXBOT2_IFC_BUILD_ROS2=OFF \
    -DBoost_USE_DEBUG_RUNTIME=OFF

  clone_checkout https://github.com/oxfordcontrol/osqp.git \
    "${SRC_DIR}/osqp" 0b34f2ef5c5eec314e7945762e1c8167e937afbd
  cmake_install osqp "${SRC_DIR}/osqp" \
    -DDLONG=OFF

  clone_checkout https://github.com/Simple-Robotics/proxsuite.git \
    "${SRC_DIR}/proxsuite" f19f07b51f66268db1f16cbeb538e891bb6d4e21
  cmake_install proxsuite "${SRC_DIR}/proxsuite" \
    -DBUILD_WITH_VECTORIZATION_SUPPORT=OFF \
    -DBUILD_TESTING=OFF

  clone_checkout https://github.com/flexible-collision-library/fcl.git \
    "${SRC_DIR}/fcl" v0.6.0
  cmake_install fcl "${SRC_DIR}/fcl"
  mkdir -p "${DEPS_PREFIX}/lib/cmake/fcl"
  cat > "${DEPS_PREFIX}/lib/cmake/fcl/fclConfigVersion.cmake" <<'EOF'
set(PACKAGE_VERSION "0.6.1")
if(PACKAGE_FIND_VERSION)
  if(PACKAGE_VERSION VERSION_LESS PACKAGE_FIND_VERSION)
    set(PACKAGE_VERSION_COMPATIBLE FALSE)
  else()
    set(PACKAGE_VERSION_COMPATIBLE TRUE)
    if(PACKAGE_VERSION VERSION_EQUAL PACKAGE_FIND_VERSION)
      set(PACKAGE_VERSION_EXACT TRUE)
    endif()
  endif()
endif()
EOF

  clone_checkout https://github.com/qpSWIFT/qpSWIFT.git \
    "${SRC_DIR}/qpSWIFT"
  cmake_install qpSWIFT "${SRC_DIR}/qpSWIFT"

  clone_checkout https://github.com/ADVRHumanoids/OpenSoT.git \
    "${SRC_DIR}/OpenSoT" 4.0-devel_ros2
  cmake_install OpenSoT "${SRC_DIR}/OpenSoT"
}

build_livox_stack() {
  clone_checkout https://github.com/Livox-SDK/Livox-SDK2.git \
    "${SRC_DIR}/Livox-SDK2"
  for f in sdk_core/comm/define.h sdk_core/logger_handler/file_manager.h; do
    local local_file="${SRC_DIR}/Livox-SDK2/${f}"
    if [ -f "${local_file}" ] && ! grep -qE '^[[:space:]]*#include[[:space:]]*<cstdint>' "${local_file}"; then
      sed -i '/^[[:space:]]*#pragma[[:space:]]\+once/a #include <cstdint>' "${local_file}"
    fi
  done
  cmake_install Livox-SDK2 "${SRC_DIR}/Livox-SDK2"

  if [ ! -d "${WS}/src/livox_ros_driver2/.git" ]; then
    git clone https://github.com/Livox-SDK/livox_ros_driver2.git "${WS}/src/livox_ros_driver2"
  fi
  rm -f "${WS}/src/livox_ros_driver2/package.xml"
  cp "${WS}/src/livox_ros_driver2/package_ROS2.xml" "${WS}/src/livox_ros_driver2/package.xml"
  cp -rf "${WS}/src/livox_ros_driver2/launch_ROS2" "${WS}/src/livox_ros_driver2/launch"
}

write_env_file() {
  cat > "${WS}/deps/env_humble_full.sh" <<EOF
#!/usr/bin/env bash
export G1PILOT_DEPS_PREFIX="${DEPS_PREFIX}"
source /opt/ros/${ROS_DISTRO}/setup.bash
source "${WS}/.venv/bin/activate"
export CMAKE_PREFIX_PATH="${DEPS_PREFIX}:\${CMAKE_PREFIX_PATH:-}"
export LD_LIBRARY_PATH="${DEPS_PREFIX}/lib:${DEPS_PREFIX}/lib/x86_64-linux-gnu:\${LD_LIBRARY_PATH:-}"
export PKG_CONFIG_PATH="${DEPS_PREFIX}/lib/pkgconfig:${DEPS_PREFIX}/lib/x86_64-linux-gnu/pkgconfig:\${PKG_CONFIG_PATH:-}"
export PYTHONPATH="${DEPS_PREFIX}/lib/python3.10/site-packages:${DEPS_PREFIX}/local/lib/python3.10/dist-packages:\${PYTHONPATH:-}"
rm -f "${WS}/.venv/bin/register-python-argcomplete" 2>/dev/null || true
if [ -f "${WS}/install/setup.bash" ]; then
  source "${WS}/install/setup.bash"
fi
EOF
  chmod +x "${WS}/deps/env_humble_full.sh"

  cat > "${WS}/env_humble.sh" <<EOF
#!/usr/bin/env bash
source "${WS}/deps/env_humble_full.sh"
EOF
  chmod +x "${WS}/env_humble.sh"
}

build_ros_workspace() {
  # shellcheck disable=SC1091
  source "${WS}/deps/env_humble_full.sh"
  cd "${WS}"
  if [ "${BUILD_LIVOX}" -eq 1 ]; then
    python -m colcon build --symlink-install --packages-up-to livox_ros_driver2 --cmake-args -DROS_EDITION=ROS2 -DDISTRO_ROS="${ROS_DISTRO}"
  fi
  python -m colcon build --symlink-install --packages-up-to g1pilot
}

setup_workspace
setup_python_env
setup_unitree_sdk
setup_build_env

if [ "${BUILD_OPENSOT}" -eq 1 ]; then
  build_opensot_stack
fi

if [ "${BUILD_LIVOX}" -eq 1 ]; then
  build_livox_stack
fi

if [ "${BUILD_REALSENSE_PY}" -eq 1 ]; then
  python -m pip install -U "pyrealsense2" "numpy==1.26.4" "opencv-python<4.12"
fi

write_env_file
build_ros_workspace

echo
echo "Full user-space Humble workspace is ready: ${WS}"
echo "Source this in future offline ROS terminals:"
echo "  source ${WS}/env_humble.sh"
echo
echo "For real G1 terminals, source:"
echo "  source ${REPO_ROOT}/scripts/source_g1_humble.sh <network-interface>"
