// dashboard_client.hpp — minimal wrapper around UR Dashboard Server
// (port 29999) for automated Power On → Brake Release → Power Off.
//
// Uses a raw TCP socket rather than ur_client_library's DashboardClient so
// we don't have to deal with its PolyscopeX vs G5 policy selection for a
// simple use case.

#pragma once
#include <string>

namespace ur10e_teleop_real_cpp {

enum RobotDashboardMode {
  RM_NO_CONTROLLER  = 0,
  RM_DISCONNECTED   = 1,
  RM_CONFIRM_SAFETY = 2,
  RM_BOOTING        = 3,
  RM_POWER_OFF      = 4,
  RM_POWER_ON       = 5,
  RM_IDLE           = 6,
  RM_RUNNING        = 7,
};

const char* robot_mode_name(int m);

class DashboardClient {
public:
  DashboardClient(const std::string& host, int port = 29999,
                  double timeout_sec = 5.0);
  ~DashboardClient();

  bool connect();
  void close();

  // Send one command, return single-line response. Empty string on error.
  std::string ask(const std::string& cmd);

  int         get_robot_mode();     // RobotDashboardMode or -1
  std::string get_safety_mode();
  bool        is_in_remote_control();

  std::string power_on();
  std::string power_off();
  std::string brake_release();
  std::string stop_program();

  // Poll robotmode until target reached or timeout. Returns true on arrival.
  bool wait_for_mode(int target_mode, double timeout_sec = 30.0,
                     double poll_interval_sec = 0.25);

  // High-level sequences.
  bool power_up_sequence(double timeout_sec = 45.0);    // → RUNNING
  bool power_down_sequence(double timeout_sec = 30.0);  // → POWER_OFF

private:
  std::string host_;
  int    port_;
  double timeout_sec_;
  int    sock_fd_ = -1;
};

}  // namespace ur10e_teleop_real_cpp
