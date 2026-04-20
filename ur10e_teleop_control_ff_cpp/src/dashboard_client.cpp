// dashboard_client.cpp — raw-socket client for UR Dashboard (port 29999).

#include "ur10e_teleop_control_ff_cpp/dashboard_client.hpp"

#include <algorithm>
#include <chrono>
#include <cstdio>
#include <cstring>
#include <thread>

#include <arpa/inet.h>
#include <netdb.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <unistd.h>
#include <fcntl.h>

namespace ur10e_teleop_control_ff_cpp {

namespace {
const char* MODE_NAMES[] = {
    "NO_CONTROLLER", "DISCONNECTED", "CONFIRM_SAFETY", "BOOTING",
    "POWER_OFF", "POWER_ON", "IDLE", "RUNNING"
};
}  // namespace

const char* robot_mode_name(int m) {
  if (m < 0 || m > 7) return "?";
  return MODE_NAMES[m];
}

DashboardClient::DashboardClient(const std::string& host, int port,
                                  double timeout_sec)
    : host_(host), port_(port), timeout_sec_(timeout_sec) {}

DashboardClient::~DashboardClient() { close(); }

bool DashboardClient::connect() {
  if (sock_fd_ >= 0) return true;
  int fd = ::socket(AF_INET, SOCK_STREAM, 0);
  if (fd < 0) {
    std::fprintf(stderr, "[dashboard] socket() failed: %s\n", std::strerror(errno));
    return false;
  }

  struct timeval tv;
  tv.tv_sec  = static_cast<time_t>(timeout_sec_);
  tv.tv_usec = static_cast<suseconds_t>((timeout_sec_ - tv.tv_sec) * 1e6);
  setsockopt(fd, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));
  setsockopt(fd, SOL_SOCKET, SO_SNDTIMEO, &tv, sizeof(tv));

  struct sockaddr_in addr{};
  addr.sin_family = AF_INET;
  addr.sin_port   = htons(port_);
  if (inet_pton(AF_INET, host_.c_str(), &addr.sin_addr) != 1) {
    struct hostent* he = gethostbyname(host_.c_str());
    if (!he) {
      std::fprintf(stderr, "[dashboard] resolve %s failed\n", host_.c_str());
      ::close(fd);
      return false;
    }
    std::memcpy(&addr.sin_addr, he->h_addr, static_cast<size_t>(he->h_length));
  }
  if (::connect(fd, reinterpret_cast<struct sockaddr*>(&addr), sizeof(addr)) < 0) {
    std::fprintf(stderr, "[dashboard] connect(%s:%d) failed: %s\n",
                 host_.c_str(), port_, std::strerror(errno));
    ::close(fd);
    return false;
  }

  // drain welcome banner
  char buf[4096];
  ssize_t n = ::recv(fd, buf, sizeof(buf) - 1, 0);
  (void)n;

  sock_fd_ = fd;
  return true;
}

void DashboardClient::close() {
  if (sock_fd_ >= 0) {
    ::close(sock_fd_);
    sock_fd_ = -1;
  }
}

std::string DashboardClient::ask(const std::string& cmd) {
  if (sock_fd_ < 0 && !connect()) return "";
  std::string line = cmd + "\n";
  if (::send(sock_fd_, line.data(), line.size(), 0) < 0) {
    std::fprintf(stderr, "[dashboard] send(%s) failed: %s\n",
                 cmd.c_str(), std::strerror(errno));
    close();
    return "";
  }
  std::string resp;
  char buf[4096];
  while (resp.find('\n') == std::string::npos) {
    ssize_t n = ::recv(sock_fd_, buf, sizeof(buf), 0);
    if (n <= 0) break;
    resp.append(buf, buf + n);
  }
  // trim trailing \n
  while (!resp.empty() && (resp.back() == '\n' || resp.back() == '\r'))
    resp.pop_back();
  return resp;
}

int DashboardClient::get_robot_mode() {
  std::string resp = ask("robotmode");  // "Robotmode: RUNNING"
  auto colon = resp.rfind(':');
  std::string name = (colon == std::string::npos) ? resp
                     : resp.substr(colon + 1);
  // strip whitespace
  name.erase(0, name.find_first_not_of(" \t"));
  name.erase(name.find_last_not_of(" \t") + 1);
  for (int i = 0; i <= 7; ++i) {
    if (name == MODE_NAMES[i]) return i;
  }
  return -1;
}

std::string DashboardClient::get_safety_mode() { return ask("safetymode"); }

bool DashboardClient::is_in_remote_control() {
  std::string r = ask("is in remote control");
  std::transform(r.begin(), r.end(), r.begin(),
                 [](unsigned char c){ return std::tolower(c); });
  return r.rfind("true", 0) == 0;
}

std::string DashboardClient::power_on()      { return ask("power on"); }
std::string DashboardClient::power_off()     { return ask("power off"); }
std::string DashboardClient::brake_release() { return ask("brake release"); }
std::string DashboardClient::stop_program()  { return ask("stop"); }

bool DashboardClient::wait_for_mode(int target, double timeout_sec,
                                    double poll_sec) {
  auto deadline = std::chrono::steady_clock::now()
                + std::chrono::duration_cast<std::chrono::milliseconds>(
                      std::chrono::duration<double>(timeout_sec));
  int last = -2;
  while (std::chrono::steady_clock::now() < deadline) {
    int m = get_robot_mode();
    if (m != last) {
      std::fprintf(stderr, "[dashboard] robot_mode = %d (%s)\n",
                   m, robot_mode_name(m));
      last = m;
    }
    if (m == target) return true;
    std::this_thread::sleep_for(std::chrono::duration<double>(poll_sec));
  }
  std::fprintf(stderr, "[dashboard] wait_for_mode(%d) timed out (last %d=%s)\n",
               target, last, robot_mode_name(last));
  return false;
}

bool DashboardClient::power_up_sequence(double timeout_sec) {
  if (!connect()) return false;
  if (!is_in_remote_control()) {
    std::fprintf(stderr,
      "[dashboard] WARNING: robot NOT in Remote Control — power-cycle may fail.\n");
  }
  int mode = get_robot_mode();
  std::fprintf(stderr, "[dashboard] current mode = %d (%s)\n",
               mode, robot_mode_name(mode));
  if (mode == RM_RUNNING) {
    std::fprintf(stderr, "[dashboard] already RUNNING — skipping power-up\n");
    return true;
  }
  std::fprintf(stderr, "[dashboard] power on …\n");
  power_on();
  if (!wait_for_mode(RM_IDLE, timeout_sec)) return false;
  std::fprintf(stderr, "[dashboard] brake release …\n");
  brake_release();
  if (!wait_for_mode(RM_RUNNING, timeout_sec)) return false;
  std::fprintf(stderr, "[dashboard] power-up complete (RUNNING)\n");
  return true;
}

bool DashboardClient::power_down_sequence(double timeout_sec) {
  if (!connect()) return false;
  std::fprintf(stderr, "[dashboard] stop program …\n");
  stop_program();
  std::this_thread::sleep_for(std::chrono::milliseconds(500));
  std::fprintf(stderr, "[dashboard] power off …\n");
  power_off();
  wait_for_mode(RM_POWER_OFF, timeout_sec);
  std::fprintf(stderr, "[dashboard] power-down complete\n");
  return true;
}

}  // namespace ur10e_teleop_control_ff_cpp
