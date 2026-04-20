#!/usr/bin/env bash
# setcap_rt.sh — grant RT capabilities (CAP_SYS_NICE + CAP_IPC_LOCK) to
# the installed binaries so `--rt-mode true` actually engages SCHED_FIFO
# and mlockall.
#
# Paths are auto-discovered — no hard-coded workspace name, no hard-coded
# ROS distro. Just source your workspace beforehand:
#     source <your-ws>/install/setup.bash
# then:
#     bash setcap_rt.sh
#
# IMPORTANT — setcap cannot set caps on a symlink ("filename must be a
# regular file"). If your workspace was built with `colcon build
# --symlink-install`, install/.../<bin> is a symlink into build/ and
# setcap will reject it. Either:
#   (a) rebuild the C++ package WITHOUT --symlink-install:
#         colcon build --packages-select ur10e_teleop_control_ff_cpp
#   (b) or accept that this script follows the symlink chain with
#       `readlink -f` and applies caps to the real build/ binary.
# We do (b) — it works for both install styles.
#
# Usage:
#   bash setcap_rt.sh                # all 3 binaries
#   bash setcap_rt.sh leader_node    # specific
#   bash setcap_rt.sh --clear        # remove caps

set -euo pipefail

PKG="ur10e_teleop_control_ff_cpp"
CAPS="cap_sys_nice,cap_ipc_lock=ep"
DEFAULT_BINS=(leader_node follower_node jitter_benchmark)

# ---- locate install prefix (workspace-agnostic, distro-agnostic) -----------
prefix=""
if command -v ros2 >/dev/null 2>&1; then
    prefix=$(ros2 pkg prefix "${PKG}" 2>/dev/null || true)
fi
if [[ -z "${prefix}" ]]; then
    echo "[setcap_rt] ERROR: cannot locate install of '${PKG}'."
    echo "            Source your workspace first, e.g.:"
    echo "              source <your_ws>/install/setup.bash"
    echo "            Or run this after 'colcon build --packages-select ${PKG}'."
    exit 1
fi

bin_dir="${prefix}/lib/${PKG}"
if [[ ! -d "${bin_dir}" ]]; then
    echo "[setcap_rt] ERROR: binary dir not found: ${bin_dir}"
    exit 1
fi

# ---- parse args -------------------------------------------------------------
clear_mode=0
targets=()
for arg in "$@"; do
    case "${arg}" in
        --clear)       clear_mode=1 ;;
        -h|--help)     sed -n '2,/^$/p' "$0" | sed 's/^# \?//'; exit 0 ;;
        *)             targets+=("${arg}") ;;
    esac
done
if [[ ${#targets[@]} -eq 0 ]]; then
    targets=("${DEFAULT_BINS[@]}")
fi

echo "[setcap_rt] package : ${PKG}"
echo "[setcap_rt] prefix  : ${prefix}"
echo "[setcap_rt] targets : ${targets[*]}"
echo "[setcap_rt] caps    : $([[ ${clear_mode} -eq 1 ]] && echo '(CLEAR)' || echo ${CAPS})"
echo

# ---- apply ------------------------------------------------------------------
for b in "${targets[@]}"; do
    path="${bin_dir}/${b}"
    if [[ ! -e "${path}" ]]; then
        echo "  SKIP  ${b}  — not found at ${path}"
        continue
    fi
    # Resolve symlinks (e.g. colcon --symlink-install points into build/).
    real_path=$(readlink -f "${path}")
    if [[ "${real_path}" != "${path}" ]]; then
        echo "  (symlink ${path}  ->  ${real_path})"
    fi
    if [[ ${clear_mode} -eq 1 ]]; then
        sudo setcap -r "${real_path}"
        echo "  CLR   ${real_path}"
    else
        sudo setcap "${CAPS}" "${real_path}"
        echo "  SET   ${real_path}"
    fi
done

echo
echo "[setcap_rt] current caps:"
for b in "${targets[@]}"; do
    path="${bin_dir}/${b}"
    [[ -e "${path}" ]] || continue
    real_path=$(readlink -f "${path}")
    current=$(getcap "${real_path}" 2>/dev/null || echo "(none)")
    echo "  ${current}"
done

echo
echo "[setcap_rt] Done."
echo "            Verify RT engages:"
echo "              ${bin_dir}/jitter_benchmark --rt-mode true --duration 2 2>&1 | tail -1"
echo "            No 'Operation not permitted' in stderr = RT active."
