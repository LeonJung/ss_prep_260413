# Latency lever — pipeline CAN I/O (upstream driver patch)

Goal: raise the CM update rate from ~75 (leader) / ~86 (follower) Hz toward ~130-150 Hz
WITHOUT new hardware. Root cause (see tech_debt.md §8d): each CM cycle does TWO blocking
CAN round-trips, but one is redundant.

- read():  refresh_all() [send status requests] + recv_all(500)   <- round-trip ①
- write(): send MIT commands              + recv_all(1000)         <- round-trip ②

Every MIT command in write() already returns pos/vel/torque feedback, so read()'s separate
refresh_all() request is redundant. Pipeline it: read() just drains the feedback the
PREVIOUS write() already elicited; write() only sends.

File: `~/openarmx_ws/src/openarmx_ros2/openarmx_hardware/src/v10_simple_hardware.cpp`

### 1) read() (~line 475) — drop the refresh_all() round-trip
```cpp
- openarmx->refresh_all();
- openarmx->recv_all();
+ // openarmx->refresh_all();        // dropped: MIT feedback is enough (pipelined)
+ openarmx->recv_all();              // drains the previous write()'s MIT feedback
```

### 2) write() (~line 664) — send only, no recv
```cpp
- openarmx->recv_all(1000);
+ // openarmx->recv_all(1000);       // feedback drained at top of next read()
```

### Build
```bash
cd ~/openarmx_ws && colcon build --packages-select openarmx_hardware && source install/setup.bash
```

### Test protocol (do NOT skip step 2)
1. **Baseline first:** with the UNPATCHED driver, run `rate_probe.launch.py`, confirm
   joint_states ≈ 75 (leader) / 86 (follower) Hz.
2. **Patch + build, then verify MOTION before trusting rate:** run bringup + bilateral,
   move the leader. State is now ~1 cycle stale — confirm tracking still feels right and
   there's no added lag/instability. If it feels laggy or unstable, REVERT (uncomment the
   two lines) immediately.
3. **Measure:** run `rate_probe.launch.py` again. Expect joint_states 75/86 -> ~130-150 Hz
   and lower dt jitter. Compare the CSVs.

### Revert
Uncomment the two lines above and rebuild. (Note: this is upstream, not in ss_prep git —
a fresh openarmx clone loses it; re-apply from here.)

### Notes / risks
- State staleness: controllers at cycle N now use feedback from write() at cycle N-1
  (~6-13 ms old). Fine for teleop; watch for any coupling-feel change.
- If the DM motors ever need the periodic status request as keep-alive, dropping
  refresh_all() could matter — but the per-cycle MIT command is itself traffic/keep-alive.
- The USB tail-latency (jitter/popping, §8c) is NOT fixed by this — that still points to
  non-USB / HS-USB. This lever fixes mean RATE / overrun only.
