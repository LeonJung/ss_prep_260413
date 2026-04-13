# ur10e_teleop

UR10E teleoperation — migration of [enactic/openarm_teleop](https://github.com/enactic/openarm_teleop).

Supports bilateral (force-feedback) and unilateral (position-only) leader-follower control of UR10E robots via RTDE.

## Architecture (preserved exactly from openarm_teleop)

```
PeriodicTimerThread (base class — unchanged)
├── LeaderThread   — reads leader UR10E state
├── FollowerThread — computes control law, sends to follower UR10E
└── AdminThread    — diagnostics, safety monitoring

Control class (bilateral + unilateral — unchanged)
  tau_f = Fc * tanh(k * dq) + Fv * dq + Fo
```

## Changes from openarm_teleop

| openarm_teleop | ur10e_teleop | Notes |
|----------------|-------------|-------|
| 7 joints per arm | 6 joints per arm | UR10E is 6-DOF |
| CAN interface arg (`can0`) | Robot IP arg (`192.168.1.10`) | RTDE replaces CAN |
| Damiao motor MITParam | RTDE JointCommand | Same {kp,kd,q,dq,tau} fields |
| openarm_can library | ur10e_rtde library | Same synchronous API |

## Dependencies

```bash
sudo apt install -y libeigen3-dev liborocos-kdl-dev libyaml-cpp-dev
# ur10e_rtde must be installed first
```

## Build

```bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build
```

## Usage

### Bilateral control (force feedback)

```bash
# Mirrors: ./script/launch_bilateral.sh right_arm can0 can2
./script/launch_bilateral.sh 192.168.1.10 192.168.1.11
```

### Unilateral control (position only)

```bash
# Mirrors: ./script/launch_unilateral.sh right_arm can0 can2
./script/launch_unilateral.sh 192.168.1.10 192.168.1.11
```

### Arguments

| openarm_teleop | ur10e_teleop | Default |
|---------------|-------------|---------|
| `right_arm` or `left_arm` | removed | — |
| `can0` (leader CAN) | `leader_ip` | `192.168.1.10` |
| `can2` (follower CAN) | `follower_ip` | `192.168.1.11` |
| — | `freq_hz` | `500` |

## Control Parameters

Edit `config/ur10e_control_params.yaml` to tune per-joint gains.

| Parameter | Meaning |
|-----------|---------|
| `Kp` | Position control gain [Nm/rad] |
| `Kd` | Velocity control gain [Nm·s/rad] |
| `Fc` | Coulomb friction [Nm] |
| `k` | Friction transition sharpness [s/rad] |
| `Fv` | Viscous friction [Nm·s/rad] |
| `Fo` | Friction bias [Nm] |

⚠️ **Safety**: Run at low gains first. UR10E shoulder at high gain with large position error can generate dangerous velocities.
