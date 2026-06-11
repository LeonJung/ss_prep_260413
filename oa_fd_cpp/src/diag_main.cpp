// diag_main.cpp — oa_diag: per-motor connectivity / health check over CAN.
//
// For each CAN interface (default: can0..can3 = leader R/L, follower R/L),
// checks the SocketCAN link, then polls each arm motor (id 1..7) and reports:
//   responding? position/velocity/torque, temperature, run pattern, error code.
// NEVER sends torque. Optionally (--enable) runs an enable->read->disable
// round-trip per interface to verify closed-loop entry.
//
// Usage:
//   oa_diag                         # probe can0 can1 can2 can3 (read-only)
//   oa_diag can0 can2               # probe specific interfaces
//   oa_diag --enable                # also test enable_all()/disable_all()
//   oa_diag --fd                    # CAN-FD sockets
//   oa_diag --tries N               # refresh/recv rounds per motor (default 10)

#include <cstdio>
#include <cstring>
#include <fstream>
#include <memory>
#include <string>
#include <vector>

#include <openarmx/can/socket/openarmx.hpp>
#include <openarmx/robstride_motor/rs_motor.hpp>
#include <openarmx/robstride_motor/rs_motor_constants.hpp>

namespace rm = openarmx::robstride_motor;

static const char* ROLE[4] = {"leader-right", "leader-left",
                              "follower-right", "follower-left"};

static const char* pattern_str(int p) {
  switch (p) {
    case 0: return "RESET";
    case 1: return "CALI";
    case 2: return "RUN";
    default: return "?";
  }
}

static std::string link_state(const std::string& iface) {
  std::ifstream f("/sys/class/net/" + iface + "/operstate");
  if (!f.good()) return "MISSING";
  std::string s;
  f >> s;
  return s;  // "up" / "down" / "unknown" (CAN often reports unknown when up)
}

struct MotorReport {
  bool responding = false;
  double q = 0, dq = 0, tau = 0;
  float temp = 0;
  int pattern = -1;
  uint8_t err = 0;
  bool enabled = false;
};

int main(int argc, char** argv) {
  std::vector<std::string> ifaces;
  bool do_enable = false, use_fd = false;
  int tries = 10;

  for (int i = 1; i < argc; ++i) {
    std::string a = argv[i];
    if (a == "--enable") do_enable = true;
    else if (a == "--fd") use_fd = true;
    else if (a == "--tries" && i + 1 < argc) tries = std::stoi(argv[++i]);
    else if (a[0] != '-') ifaces.push_back(a);
  }
  if (ifaces.empty()) ifaces = {"can0", "can1", "can2", "can3"};

  std::printf("oa_diag — OpenArm A2 motor connectivity check "
              "(read-only%s, %s)\n\n",
              do_enable ? " + enable round-trip" : "",
              use_fd ? "CAN-FD" : "classic CAN");

  int total_bad = 0;

  for (size_t k = 0; k < ifaces.size(); ++k) {
    const std::string& ifc = ifaces[k];
    const char* role = (k < 4 && ifaces.size() == 4) ? ROLE[k] : "";
    std::string ls = link_state(ifc);
    std::printf("=== %s  %s  [link: %s] ===\n", ifc.c_str(), role, ls.c_str());
    if (ls == "MISSING") {
      std::printf("  !! interface does not exist — check USB2CAN / "
                  "`ip link set %s up type can bitrate 1000000`\n\n", ifc.c_str());
      total_bad += 7;
      continue;
    }
    if (ls == "down") {
      std::printf("  !! link DOWN — `sudo ip link set %s up type can "
                  "bitrate 1000000`\n\n", ifc.c_str());
      total_bad += 7;
      continue;
    }

    std::unique_ptr<openarmx::can::socket::OpenArmX> dev;
    try {
      dev = std::make_unique<openarmx::can::socket::OpenArmX>(ifc, use_fd);
      const std::vector<rm::MotorType> types = {
          rm::MotorType::RS04, rm::MotorType::RS04,
          rm::MotorType::RS03, rm::MotorType::RS03,
          rm::MotorType::RS00, rm::MotorType::RS00, rm::MotorType::RS00};
      const std::vector<uint32_t> ids = {1, 2, 3, 4, 5, 6, 7};
      dev->init_arm_motors(types, ids, ids);
      dev->set_callback_mode_all(rm::CallbackMode::STATE);
    } catch (const std::exception& e) {
      std::printf("  !! open/init failed: %s\n\n", e.what());
      total_bad += 7;
      continue;
    }

    // Poll: several refresh+recv rounds; a motor that never updates
    // temperature/pattern is considered not responding (a live Robstride
    // reports temp > 0 and pattern in {0,1,2}).
    for (int t = 0; t < tries; ++t) {
      dev->refresh_all();
      dev->recv_all(2000);
    }

    auto motors = dev->get_arm().get_motors();
    std::vector<MotorReport> rep(7);
    std::printf("  %-3s %-6s %-5s %-8s %-8s %-8s %-6s %-7s %-5s %s\n",
                "id", "type", "resp", "q[rad]", "dq", "tau[Nm]", "T[C]",
                "pattern", "en", "error");
    for (size_t i = 0; i < motors.size() && i < 7; ++i) {
      auto* mo = motors[i];
      MotorReport& r = rep[i];
      r.q = mo->get_position();
      r.dq = mo->get_velocity();
      r.tau = mo->get_torque();
      r.temp = mo->get_temperature();
      r.pattern = mo->get_pattern();
      r.err = mo->get_error_code();
      r.enabled = mo->is_enabled();
      r.responding = (r.temp > 0.5f);   // live motor reports real temperature
      if (!r.responding) ++total_bad;
      const char* tname = (i < 2) ? "RS04" : (i < 4) ? "RS03" : "RS00";
      std::printf("  %-3zu %-6s %-5s %8.3f %8.3f %8.2f %6.1f %-7s %-5s ",
                  i + 1, tname, r.responding ? "OK" : "--",
                  r.q, r.dq, r.tau, r.temp, pattern_str(r.pattern),
                  r.enabled ? "yes" : "no");
      if (r.err) std::printf("0x%02X !!", r.err);
      else std::printf("-");
      std::printf("\n");
    }

    if (do_enable) {
      std::printf("  enable round-trip: ");
      bool ok = dev->enable_all();
      dev->refresh_all();
      dev->recv_all(2000);
      int run = 0;
      for (auto* mo : dev->get_arm().get_motors())
        if (mo->get_pattern() == 2) ++run;
      bool ok2 = dev->disable_all();
      std::printf("enable_all=%s, %d/7 in RUN, disable_all=%s\n",
                  ok ? "OK" : "FAIL", run, ok2 ? "OK" : "FAIL");
      if (!ok || run < 7) ++total_bad;
    }
    std::printf("\n");
  }

  if (total_bad == 0) {
    std::printf("RESULT: all motors responding on all interfaces ✔\n");
    return 0;
  }
  std::printf("RESULT: %d problem(s) found — see '--' / '!!' lines above\n",
              total_bad);
  return 1;
}
