#!/usr/bin/env bash
# setcap_rt.sh — grant RT capabilities (CAP_SYS_NICE + CAP_IPC_LOCK) to the
# installed binaries so `--rt-mode true` actually engages SCHED_FIFO and
# mlockall. Without these caps the RT code paths fall through to normal
# scheduling and the RT benefit isn't visible.
#
# Needs sudo (invokes setcap).
#
# Usage:
#   bash script/setcap_rt.sh                # both teleop nodes + benchmark
#   bash script/setcap_rt.sh leader_node    # a specific binary
#   bash script/setcap_rt.sh --clear        # remove caps (debug)
#
# Re-runnable. Also safe after each colcon rebuild — rebuilt binary loses
# its caps, so re-run this.

set -euo pipefail

PKG="ur10e_teleop_real_cpp"
CAPS="cap_sys_nice+ep,cap_ipc_lock+ep"
DEFAULT_BINS=(leader_node follower_node jitter_benchmark)

# ---- locate install prefix --------------------------------------------------
prefix=""
if command -v ros2 >/dev/null 2>&1; then
    prefix=$(ros2 pkg prefix "${PKG}" 2>/dev/null || true)
fi
if [[ -z "${prefix}" ]]; then
    for candidate in \
        "${HOME}/colcon_ws/install/${PKG}" \
        "/opt/ros/jazzy/share/${PKG}" ; do
        if [[ -d "${candidate}" ]]; then prefix="${candidate}"; break; fi
    done
fi

if [[ -z "${prefix}" || ! -d "${prefix}" ]]; then
    echo "[setcap_rt] ERROR: cannot locate install prefix of ${PKG}."
    echo "            Did you run 'colcon build --packages-select ${PKG}'"
    echo "            and 'source install/setup.bash'?"
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
        --clear) clear_mode=1 ;;
        -h|--help)
            sed -n '2,/^$/p' "$0" | sed 's/^# \?//'
            exit 0 ;;
        *) targets+=("${arg}") ;;
    esac
done
if [[ ${#targets[@]} -eq 0 ]]; then
    targets=("${DEFAULT_BINS[@]}")
fi

# ---- apply ------------------------------------------------------------------
echo "[setcap_rt] prefix: ${prefix}"
echo "[setcap_rt] bin_dir: ${bin_dir}"
echo "[setcap_rt] target : ${targets[*]}"
echo "[setcap_rt] caps   : $([[ ${clear_mode} -eq 1 ]] && echo '(CLEAR)' || echo ${CAPS})"
echo

for b in "${targets[@]}"; do
    path="${bin_dir}/${b}"
    if [[ ! -x "${path}" ]]; then
        echo "  SKIP  ${b}  — not found at ${path}"
        continue
    fi
    if [[ ${clear_mode} -eq 1 ]]; then
        sudo setcap -r "${path}"
        echo "  CLR   ${path}"
    else
        sudo setcap "${CAPS}" "${path}"
        echo "  SET   ${path}"
    fi
done

echo
echo "[setcap_rt] current caps:"
for b in "${targets[@]}"; do
    path="${bin_dir}/${b}"
    if [[ -x "${path}" ]]; then
        current=$(getcap "${path}" || echo "(none)")
        echo "  ${current}"
    fi
done

echo
echo "[setcap_rt] Done. Verify with:  ${bin_dir}/jitter_benchmark --rt-mode true --duration 2"
echo "            (no 'Operation not permitted' in stderr means RT engaged)"
