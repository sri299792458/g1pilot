#!/usr/bin/env bash
set -euo pipefail

# Distro-aware user-space setup. Set ROS_DISTRO=humble on the Ubuntu 22 lab
# laptop, ROS_DISTRO=jazzy on Ubuntu 24/Jazzy, or leave it unset to auto-detect
# an installed ROS distro.

detect_ros_distro() {
  for distro in jazzy humble; do
    if [ -f "/opt/ros/${distro}/setup.bash" ]; then
      echo "${distro}"
      return
    fi
  done
  find /opt/ros -mindepth 2 -maxdepth 2 -name setup.bash 2>/dev/null \
    | sed -n 's#^/opt/ros/\([^/]*\)/setup.bash$#\1#p' \
    | sort \
    | tail -n 1
}

ROS_DISTRO="${ROS_DISTRO:-$(detect_ros_distro)}"
WS="${G1PILOT_WS:-${HOME}/g1pilot_${ROS_DISTRO}_ws}"
REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
JOBS="${JOBS:-4}"
BASE_PYTHON="${G1PILOT_PYTHON:-/usr/bin/python3}"

EXTERNAL_DIR="${WS}/external"
UNITREE_SDK_DIR="${EXTERNAL_DIR}/unitree_sdk2_python"
UNITREE_SDK_REPO="${UNITREE_SDK_REPO:-https://github.com/lnotspotl/unitree_sdk2_python.git}"
UNITREE_SDK_COMMIT="${UNITREE_SDK_COMMIT:-7c661d27f4ae064ffd0dd633fd9d5b518ef0b508}"
TELEIMAGER_DIR="${EXTERNAL_DIR}/teleimager"
TELEIMAGER_REPO="${TELEIMAGER_REPO:-https://github.com/unitreerobotics/teleimager.git}"
ASTROVIZ_REPO="${ASTROVIZ_REPO:-https://github.com/CDonosoK/astroviz_interfaces.git}"

FOREST_WS="${WS}/deps/forest_ws"
SRC_DIR="${WS}/deps/src"
BUILD_DIR="${WS}/deps/build"
DEPS_PREFIX="${FOREST_WS}/install"

BUILD_OPENSOT=1
BUILD_LIVOX=1
BUILD_REALSENSE_PY=1
INSTALL_TELEIMAGER_CLIENT=1

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
    --skip-teleimager-client)
      INSTALL_TELEIMAGER_CLIENT=0
      ;;
    *)
      echo "Unknown option: ${arg}"
      echo "Usage: $0 [--skip-opensot] [--skip-livox] [--skip-realsense-python] [--skip-teleimager-client]"
      exit 1
      ;;
  esac
done

if [ ! -f "/opt/ros/${ROS_DISTRO}/setup.bash" ]; then
  echo "ROS 2 ${ROS_DISTRO:-<unset>} was not found at /opt/ros/${ROS_DISTRO}/setup.bash."
  echo "Set ROS_DISTRO explicitly, for example ROS_DISTRO=humble or ROS_DISTRO=jazzy."
  exit 1
fi

if [ ! -x "${BASE_PYTHON}" ]; then
  echo "Python interpreter not found or not executable: ${BASE_PYTHON}"
  echo "Set G1PILOT_PYTHON to the Python interpreter used by ROS ${ROS_DISTRO}."
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

ensure_header_include() {
  local file="$1"
  local include="$2"

  if [ ! -f "${file}" ] || grep -qE "^[[:space:]]*#include[[:space:]]*<${include}>" "${file}"; then
    return
  fi

  if grep -qE '^[[:space:]]*#pragma[[:space:]]+once' "${file}"; then
    sed -i -E "/^[[:space:]]*#pragma[[:space:]]+once/a #include <${include}>" "${file}"
  elif grep -qE '^[[:space:]]*#define[[:space:]]+[^[:space:]]+' "${file}"; then
    sed -i -E "/^[[:space:]]*#define[[:space:]]+[^[:space:]]+/a #include <${include}>" "${file}"
  else
    sed -i "1i #include <${include}>" "${file}"
  fi
}

source_setup_file() {
  local setup_file="$1"
  set +u
  # shellcheck disable=SC1090
  source "${setup_file}"
  set -u
}

filter_conda_path_entries() {
  local value="${1:-}"
  local entry
  local out=""
  local conda_prefix="${CONDA_PREFIX:-}"

  IFS=':' read -r -a entries <<< "${value}"
  for entry in "${entries[@]}"; do
    case "${entry}" in
      ""|"${HOME}/miniconda3"*|"${HOME}/anaconda3"*)
        continue
        ;;
    esac
    if [ -n "${conda_prefix}" ] && [[ "${entry}" == "${conda_prefix}"* ]]; then
      continue
    fi
    out="${out:+${out}:}${entry}"
  done
  printf '%s' "${out}"
}

sanitize_native_build_env() {
  export CMAKE_PREFIX_PATH="$(filter_conda_path_entries "${CMAKE_PREFIX_PATH:-}")"
  export CMAKE_MODULE_PATH="$(filter_conda_path_entries "${CMAKE_MODULE_PATH:-}")"
  export LD_LIBRARY_PATH="$(filter_conda_path_entries "${LD_LIBRARY_PATH:-}")"
  export LIBRARY_PATH="$(filter_conda_path_entries "${LIBRARY_PATH:-}")"
  export PKG_CONFIG_PATH="$(filter_conda_path_entries "${PKG_CONFIG_PATH:-}")"

  local ignored_prefixes=("${HOME}/miniconda3" "${HOME}/anaconda3")
  if [ -n "${CONDA_PREFIX:-}" ]; then
    ignored_prefixes=("${CONDA_PREFIX}" "${ignored_prefixes[@]}")
  fi
  local ignored
  ignored="$(IFS=';'; echo "${ignored_prefixes[*]}")"
  export CMAKE_IGNORE_PREFIX_PATH="${ignored}${CMAKE_IGNORE_PREFIX_PATH:+;${CMAKE_IGNORE_PREFIX_PATH}}"
}

cmake_install() {
  local name="$1"
  local src="$2"
  shift 2
  local build="${BUILD_DIR}/${name}"
  mkdir -p "${build}"
  sanitize_native_build_env
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

ensure_virtualenv_module() {
  if "${BASE_PYTHON}" -m virtualenv --version >/dev/null 2>&1; then
    return
  fi

  local bootstrap_dir="${WS}/.bootstrap_python"
  mkdir -p "${bootstrap_dir}"
  "${BASE_PYTHON}" -m pip install --upgrade --target "${bootstrap_dir}" virtualenv
  export PYTHONPATH="${bootstrap_dir}:${PYTHONPATH:-}"
  "${BASE_PYTHON}" -m virtualenv --version >/dev/null
}

create_python_venv() {
  local venv_dir="$1"

  if [ -x "${venv_dir}/bin/python" ] && "${venv_dir}/bin/python" -m pip --version >/dev/null 2>&1; then
    return
  fi

  rm -rf "${venv_dir}"
  if "${BASE_PYTHON}" -m venv --system-site-packages "${venv_dir}"; then
    return
  fi

  echo "${BASE_PYTHON} -m venv failed; falling back to local virtualenv bootstrap."
  rm -rf "${venv_dir}"
  ensure_virtualenv_module
  "${BASE_PYTHON}" -m virtualenv --system-site-packages "${venv_dir}"
}

verify_base_python_matches_ros() {
  source_setup_file "/opt/ros/${ROS_DISTRO}/setup.bash"
  "${BASE_PYTHON}" - <<'PY'
import rclpy
print("Verified ROS Python import:", rclpy.__file__)
PY
}

prepare_cyclonedds_python_prefix() {
  local ros_prefix="/opt/ros/${ROS_DISTRO}"
  local prefix="${WS}/deps/cyclonedds_python_prefix"
  local lib_dir="${ros_prefix}/lib"
  local include_dir="${ros_prefix}/include"

  if [ -f "${ros_prefix}/lib/x86_64-linux-gnu/libddsc.so" ]; then
    lib_dir="${ros_prefix}/lib/x86_64-linux-gnu"
  fi
  if [ -d "${ros_prefix}/include/CycloneDDS" ]; then
    include_dir="${ros_prefix}/include/CycloneDDS"
  fi

  mkdir -p "${prefix}"
  ln -sfn "${include_dir}" "${prefix}/include"
  ln -sfn "${ros_prefix}/bin" "${prefix}/bin"
  ln -sfn "${lib_dir}" "${prefix}/lib"

  export CYCLONEDDS_HOME="${prefix}"
  export CMAKE_PREFIX_PATH="${prefix}:${ros_prefix}:${CMAKE_PREFIX_PATH:-}"
  export LD_LIBRARY_PATH="${prefix}/lib:${ros_prefix}/lib:${ros_prefix}/lib/x86_64-linux-gnu:${LD_LIBRARY_PATH:-}"
  export LIBRARY_PATH="${prefix}/lib:${LIBRARY_PATH:-}"
}

setup_python_env() {
  source_setup_file "/opt/ros/${ROS_DISTRO}/setup.bash"
  prepare_cyclonedds_python_prefix

  create_python_venv "${WS}/.venv"
  # shellcheck disable=SC1091
  source "${WS}/.venv/bin/activate"
  rm -f "${WS}/.venv/bin/register-python-argcomplete" 2>/dev/null || true

  python -m pip install -U pip "setuptools<80" wheel
  python -m pip install -U \
    "catkin_pkg" \
    "colcon-common-extensions" \
    "cyclonedds==0.10.2" \
    "empy<4" \
    "evdev" \
    "hhcm-forest" \
    "hidapi" \
    "lark" \
    "matplotlib" \
    "meshcat" \
    "mujoco" \
    "numpy==1.26.4" \
    "onnxruntime" \
    "opencv-python<4.12" \
    "pygame" \
    "pyqt6" \
    "pyspacemouse" \
    "robot_descriptions" \
    "rich-click" \
    "scipy==1.13.1" \
    "torch" \
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
    local crc_dest="${unitree_site}/utils/lib/crc_amd64.so"
    mkdir -p "$(dirname "${crc_dest}")"
    if [ "$(realpath "${crc_src}")" != "$(realpath -m "${crc_dest}")" ]; then
      cp "${crc_src}" "${crc_dest}"
      chmod +x "${crc_dest}"
    fi
  fi
}

setup_teleimager_client() {
  clone_checkout "${TELEIMAGER_REPO}" "${TELEIMAGER_DIR}"
  python -m pip install \
    "logging_mp" \
    "numpy==1.26.4" \
    "opencv-python<4.12" \
    "pyyaml" \
    "pyzmq"
  # TeleImager upstream documents Python 3.10 and currently declares
  # requires-python <3.12. The client imports under Jazzy/Python 3.12 once its
  # dependencies are installed, so keep the lab client available without letting
  # pip's resolver replace the pinned NumPy/OpenCV stack.
  python -m pip install --ignore-requires-python --no-deps -e "${TELEIMAGER_DIR}"
}

setup_build_env() {
  source_setup_file "/opt/ros/${ROS_DISTRO}/setup.bash"
  # shellcheck disable=SC1091
  source "${WS}/.venv/bin/activate"
  sanitize_native_build_env
  local py_version
  py_version="$(python - <<'PY'
import sys
print(f"{sys.version_info.major}.{sys.version_info.minor}")
PY
)"

  export HHCM_FOREST_CLONE_DEFAULT_PROTO=https
  export CMAKE_PREFIX_PATH="${DEPS_PREFIX}:${CMAKE_PREFIX_PATH:-}"
  export LD_LIBRARY_PATH="${DEPS_PREFIX}/lib:${DEPS_PREFIX}/lib/x86_64-linux-gnu:${LD_LIBRARY_PATH:-}"
  export LIBRARY_PATH="${DEPS_PREFIX}/lib:${DEPS_PREFIX}/lib/x86_64-linux-gnu:${LIBRARY_PATH:-}"
  export PKG_CONFIG_PATH="${DEPS_PREFIX}/lib/pkgconfig:${DEPS_PREFIX}/lib/x86_64-linux-gnu/pkgconfig:/opt/ros/${ROS_DISTRO}/lib/x86_64-linux-gnu/pkgconfig:${PKG_CONFIG_PATH:-}"
  export PYTHONPATH="${DEPS_PREFIX}/lib/python${py_version}/site-packages:${DEPS_PREFIX}/local/lib/python${py_version}/dist-packages:${PYTHONPATH:-}"
}

deps_python_site_packages() {
  local py_version
  py_version="$(python - <<'PY'
import sys
print(f"{sys.version_info.major}.{sys.version_info.minor}")
PY
)"
  echo "${DEPS_PREFIX}/lib/python${py_version}/site-packages"
}

install_opensot_collision_binding() {
  local py_site
  py_site="$(deps_python_site_packages)"
  local collision_build_dir="${BUILD_DIR}/OpenSoT/lib"

  shopt -s nullglob
  local collision_modules=("${collision_build_dir}"/pyopensot_collision*.so)
  shopt -u nullglob

  if [ "${#collision_modules[@]}" -eq 0 ]; then
    echo "ERROR: OpenSoT did not build pyopensot_collision under ${collision_build_dir}."
    echo "       enable_collision_avoidance:=true requires this Python module."
    exit 1
  fi

  mkdir -p "${py_site}"
  cp "${collision_modules[@]}" "${py_site}/"
}

verify_opensot_collision_binding() {
  local py_site
  py_site="$(deps_python_site_packages)"
  PYTHONPATH="${py_site}:${PYTHONPATH:-}" python - <<'PY'
import pyopensot
from pyopensot_collision.constraints.velocity import CollisionAvoidance
print("Verified pyopensot_collision import.")
PY
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
  source_setup_file "${FOREST_WS}/setup.bash"
  sanitize_native_build_env
  export AMENT_PREFIX_PATH="${DEPS_PREFIX}:/opt/ros/${ROS_DISTRO}:${AMENT_PREFIX_PATH:-}"
  export CMAKE_PREFIX_PATH="${DEPS_PREFIX}:/opt/ros/${ROS_DISTRO}:${CMAKE_PREFIX_PATH:-}"
  export LD_LIBRARY_PATH="${DEPS_PREFIX}/lib:${DEPS_PREFIX}/lib/x86_64-linux-gnu:${LD_LIBRARY_PATH:-}"
  export LIBRARY_PATH="${DEPS_PREFIX}/lib:${DEPS_PREFIX}/lib/x86_64-linux-gnu:${LIBRARY_PATH:-}"
  export PKG_CONFIG_PATH="${DEPS_PREFIX}/lib/pkgconfig:${DEPS_PREFIX}/lib/x86_64-linux-gnu/pkgconfig:/opt/ros/${ROS_DISTRO}/lib/x86_64-linux-gnu/pkgconfig:${PKG_CONFIG_PATH:-}"
}

build_opensot_stack() {
  ensure_forest_matlogger2

  clone_checkout https://github.com/OctoMap/octomap.git \
    "${SRC_DIR}/octomap" v1.9.8
  cmake_install octomap "${SRC_DIR}/octomap" \
    -DBUILD_OCTOVIS_SUBPROJECT=OFF \
    -DBUILD_DYNAMICETD3D_SUBPROJECT=OFF \
    -DBUILD_TESTING=OFF

  clone_checkout https://github.com/humanoid-path-planner/hpp-fcl.git \
    "${SRC_DIR}/hpp-fcl" 45e60ca7ba81e5394605f8c1097c016245d221c2
  cmake_install hpp-fcl "${SRC_DIR}/hpp-fcl" \
    -DBUILD_PYTHON_INTERFACE=OFF \
    -DBUILD_TESTING=OFF \
    -Doctomap_DIR="${DEPS_PREFIX}/share/octomap"

  clone_checkout https://github.com/stack-of-tasks/pinocchio.git \
    "${SRC_DIR}/pinocchio" v3.9.0
  local ros_multiarch="x86_64-linux-gnu"
  if command -v dpkg-architecture >/dev/null 2>&1; then
    ros_multiarch="$(dpkg-architecture -qDEB_HOST_MULTIARCH)"
  fi
  cmake_install pinocchio "${SRC_DIR}/pinocchio" \
    -DBUILD_WITH_URDF_SUPPORT=ON \
    -DBUILD_WITH_COLLISION_SUPPORT=ON \
    -DBUILD_TESTING=FALSE \
    -DBUILD_PYTHON_INTERFACE=OFF \
    -Doctomap_DIR="${DEPS_PREFIX}/share/octomap" \
    -Durdfdom_DIR="/opt/ros/${ROS_DISTRO}/lib/${ros_multiarch}/urdfdom/cmake" \
    -Durdfdom_headers_DIR="/opt/ros/${ROS_DISTRO}/lib/${ros_multiarch}/urdfdom_headers/cmake"

  clone_checkout https://github.com/ros-planning/srdfdom.git \
    "${SRC_DIR}/srdfdom" 2.0.7
  cmake_install srdfdom "${SRC_DIR}/srdfdom" \
    -DBUILD_TESTING=OFF \
    -Durdf_DIR="/opt/ros/${ROS_DISTRO}/share/urdf/cmake" \
    -Durdfdom_DIR="/opt/ros/${ROS_DISTRO}/lib/${ros_multiarch}/urdfdom/cmake" \
    -Durdfdom_headers_DIR="/opt/ros/${ROS_DISTRO}/lib/${ros_multiarch}/urdfdom_headers/cmake"

  clone_checkout https://github.com/ros/eigen_stl_containers.git \
    "${SRC_DIR}/eigen_stl_containers" 1.1.0
  cmake_install eigen_stl_containers "${SRC_DIR}/eigen_stl_containers" \
    -DBUILD_TESTING=OFF

  clone_checkout https://github.com/ros-planning/random_numbers.git \
    "${SRC_DIR}/random_numbers" 2.0.1
  cmake_install random_numbers "${SRC_DIR}/random_numbers" \
    -DBUILD_TESTING=OFF

  clone_checkout https://github.com/danfis/libccd.git \
    "${SRC_DIR}/libccd" v2.1
  cmake_install libccd "${SRC_DIR}/libccd" \
    -DBUILD_SHARED_LIBS=ON \
    -DENABLE_DOUBLE_PRECISION=ON

  clone_checkout https://github.com/flexible-collision-library/fcl.git \
    "${SRC_DIR}/fcl" v0.6.0
  cmake_install fcl "${SRC_DIR}/fcl" \
    -DBUILD_TESTING=OFF \
    -DFCL_BUILD_TESTS=OFF
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

  clone_checkout https://github.com/ros-planning/geometric_shapes.git \
    "${SRC_DIR}/geometric_shapes" 2.3.2
  cmake_install geometric_shapes "${SRC_DIR}/geometric_shapes" \
    -DBUILD_TESTING=OFF \
    -Doctomap_DIR="${DEPS_PREFIX}/share/octomap" \
    -Dfcl_DIR="${DEPS_PREFIX}/lib/cmake/fcl" \
    -DCMAKE_SHARED_LINKER_FLAGS="-L${DEPS_PREFIX}/lib"

  clone_checkout https://github.com/ADVRHumanoids/xbot2_interface.git \
    "${SRC_DIR}/xbot2_interface" devel
  cmake_install xbot2_interface "${SRC_DIR}/xbot2_interface" \
    -DXBOT2_IFC_BUILD_TESTS=OFF \
    -DBUILD_TESTING=OFF \
    -DXBOT2_IFC_BUILD_ROS=OFF \
    -DXBOT2_IFC_BUILD_ROS2=OFF \
    -DBoost_USE_DEBUG_RUNTIME=OFF \
    -Durdf_DIR="/opt/ros/${ROS_DISTRO}/share/urdf/cmake"

  clone_checkout https://github.com/oxfordcontrol/osqp.git \
    "${SRC_DIR}/osqp" 0b34f2ef5c5eec314e7945762e1c8167e937afbd
  cmake_install osqp "${SRC_DIR}/osqp" \
    -DDLONG=OFF

  clone_checkout https://github.com/Simple-Robotics/proxsuite.git \
    "${SRC_DIR}/proxsuite" f19f07b51f66268db1f16cbeb538e891bb6d4e21
  cmake_install proxsuite "${SRC_DIR}/proxsuite" \
    -DBUILD_WITH_VECTORIZATION_SUPPORT=OFF \
    -DBUILD_TESTING=OFF

  clone_checkout https://github.com/qpSWIFT/qpSWIFT.git \
    "${SRC_DIR}/qpSWIFT"
  cmake_install qpSWIFT "${SRC_DIR}/qpSWIFT"

  clone_checkout https://github.com/wg-perception/object_recognition_msgs.git \
    "${SRC_DIR}/object_recognition_msgs" 2.0.0
  cmake_install object_recognition_msgs "${SRC_DIR}/object_recognition_msgs" \
    -DBUILD_TESTING=OFF

  clone_checkout https://github.com/OctoMap/octomap_msgs.git \
    "${SRC_DIR}/octomap_msgs" 2.0.1
  cmake_install octomap_msgs "${SRC_DIR}/octomap_msgs" \
    -DBUILD_TESTING=OFF \
    -Doctomap_DIR="${DEPS_PREFIX}/share/octomap"

  clone_checkout https://github.com/ros-planning/moveit_msgs.git \
    "${SRC_DIR}/moveit_msgs" 2.2.1
  cmake_install moveit_msgs "${SRC_DIR}/moveit_msgs" \
    -DBUILD_TESTING=OFF

  clone_checkout https://github.com/ADVRHumanoids/OpenSoT.git \
    "${SRC_DIR}/OpenSoT" 4.0-devel_ros2
  cmake_install OpenSoT "${SRC_DIR}/OpenSoT"
  install_opensot_collision_binding
  verify_opensot_collision_binding
}

build_livox_stack() {
  clone_checkout https://github.com/Livox-SDK/Livox-SDK2.git \
    "${SRC_DIR}/Livox-SDK2"
  for f in sdk_core/comm/define.h sdk_core/logger_handler/file_manager.h; do
    ensure_header_include "${SRC_DIR}/Livox-SDK2/${f}" cstdint
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
  local py_version
  py_version="$(python - <<'PY'
import sys
print(f"{sys.version_info.major}.{sys.version_info.minor}")
PY
)"

  local env_full="${WS}/deps/env_${ROS_DISTRO}_full.sh"
  local env_short="${WS}/env_${ROS_DISTRO}.sh"

  cat > "${env_full}" <<EOF
#!/usr/bin/env bash
export G1PILOT_DEPS_PREFIX="${DEPS_PREFIX}"
source /opt/ros/${ROS_DISTRO}/setup.bash
source "${WS}/.venv/bin/activate"
export AMENT_PREFIX_PATH="${DEPS_PREFIX}:\${AMENT_PREFIX_PATH:-}"
export CMAKE_PREFIX_PATH="${DEPS_PREFIX}:\${CMAKE_PREFIX_PATH:-}"
export LD_LIBRARY_PATH="${DEPS_PREFIX}/lib:${DEPS_PREFIX}/lib/x86_64-linux-gnu:\${LD_LIBRARY_PATH:-}"
export LIBRARY_PATH="${DEPS_PREFIX}/lib:${DEPS_PREFIX}/lib/x86_64-linux-gnu:\${LIBRARY_PATH:-}"
export PKG_CONFIG_PATH="${DEPS_PREFIX}/lib/pkgconfig:${DEPS_PREFIX}/lib/x86_64-linux-gnu/pkgconfig:\${PKG_CONFIG_PATH:-}"
export PYTHONPATH="${DEPS_PREFIX}/lib/python${py_version}/site-packages:${DEPS_PREFIX}/local/lib/python${py_version}/dist-packages:\${PYTHONPATH:-}"
rm -f "${WS}/.venv/bin/register-python-argcomplete" 2>/dev/null || true
if [ -f "${WS}/install/setup.bash" ]; then
  source "${WS}/install/setup.bash"
fi
EOF
  chmod +x "${env_full}"

  cat > "${env_short}" <<EOF
#!/usr/bin/env bash
source "${env_full}"
EOF
  chmod +x "${env_short}"

  cat > "${WS}/env_ros.sh" <<EOF
#!/usr/bin/env bash
source "${env_full}"
EOF
  chmod +x "${WS}/env_ros.sh"
}

build_ros_workspace() {
  source_setup_file "${WS}/deps/env_${ROS_DISTRO}_full.sh"
  cd "${WS}"
  if [ "${BUILD_LIVOX}" -eq 1 ]; then
    python -m colcon build --symlink-install --packages-up-to livox_ros_driver2 --cmake-args -DROS_EDITION=ROS2 -DDISTRO_ROS="${ROS_DISTRO}"
  fi
  python -m colcon build --symlink-install --packages-up-to g1pilot
}

setup_workspace
verify_base_python_matches_ros
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

if [ "${INSTALL_TELEIMAGER_CLIENT}" -eq 1 ]; then
  setup_teleimager_client
fi

write_env_file
build_ros_workspace

echo
echo "Full user-space ROS 2 ${ROS_DISTRO} workspace is ready: ${WS}"
echo "Source this in future offline ROS terminals:"
echo "  source ${WS}/env_ros.sh"
echo
echo "For real G1 terminals, source:"
echo "  source ${REPO_ROOT}/scripts/source_g1.sh real <network-interface>"
echo
echo "For localhost MuJoCo/DDS terminals, source:"
echo "  source ${REPO_ROOT}/scripts/source_g1.sh sim"
