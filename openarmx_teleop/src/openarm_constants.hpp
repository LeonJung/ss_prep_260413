// Copyright 2025 Enactic, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#pragma once

#include <time.h>
#include <unistd.h>

#include <iostream>
#include <openarmx/robstride_motor/rs_motor_constants.hpp>
#include <vector>

constexpr double PI = 3.14159265358979323846;

// 8piecies including gripper
// Joints and motors don't always have a one-to-one correspondence
#define NJOINTS 8
#define NMOTORS 8

#define ROLE_LEADER 1
#define ROLE_FOLLOWER 2

#define CAN0 "can0"
#define CAN1 "can1"

#define CAN2 "can2"
#define CAN3 "can3"

#define TANHFRIC true

// [openarmx port] control loop rate. Our USB2CAN (full-speed, classic 1Mbps, CAN-FD
// unavailable on Robstride) sustains ~150 Hz UNIFORM for 8 motors/channel (see
// openarmx_bilateral/LATENCY_INVESTIGATION.md). enactic uses 1000 with CAN-FD. Start at
// 150 (known-safe uniform); the round-trip-paced loop may sustain a bit more — tune later.
#define FREQUENCY 150.0
#define CUTOFF_FREQUENCY 90.0

// [openarmx port] gravity-comp scale. Our robot floats up at 1.0 (over-comp); 0.93 verified
// on openarmx_bilateral. Applied to KDL gravity in control bilateral_step.
#define G_SCALE 0.93

#define ELBOWLIMIT 0.0

static const double INITIAL_POSITION[NMOTORS] = {0, 0, 0, PI / 5.0, 0, 0, 0, 0};

// safety limit position
static const double position_limit_max_L[] = {(2.0 / 3.0) * PI, PI,       PI / 2.0, PI,
                                              PI / 2.0,         PI / 2.0, PI / 2.0, PI};
static const double position_limit_min_L[] = {-(2.0 / 3.0) * PI, -PI / 2.0, -PI / 2.0, ELBOWLIMIT,
                                              -PI / 2.0,         -PI / 2.0, -PI / 2.0, -PI};
static const double position_limit_max_F[] = {(2.0 / 3.0) * PI, PI,       PI / 2.0, PI,
                                              PI / 2.0,         PI / 2.0, PI / 2.0, PI};
static const double position_limit_min_F[] = {-(2.0 / 3.0) * PI, -PI / 2.0, -PI / 2.0, ELBOWLIMIT,
                                              -PI / 2.0,         -PI / 2.0, -PI / 2.0, -PI};

// sefaty limit velocity
static const double velocity_limit_L[] = {8.0, 8.0, 8.0, 8.0, 8.0, 8.0, 8.0, 8.0};
static const double velocity_limit_F[] = {8.0, 8.0, 8.0, 8.0, 8.0, 8.0, 8.0, 8.0};
// sefaty limit effort
static const double effort_limit_L[] = {20.0, 20.0, 20.0, 20.0, 20.0, 20.0, 20.0, 20.0};
static const double effort_limit_F[] = {20.0, 20.0, 20.0, 20.0, 20.0, 20.0, 20.0, 20.0};

// Motor configuration structure
struct MotorConfig {
    std::vector<openarmx::robstride_motor::MotorType> arm_motor_types;
    std::vector<uint32_t> arm_send_can_ids;
    std::vector<uint32_t> arm_recv_can_ids;
    openarmx::robstride_motor::MotorType gripper_motor_type;
    uint32_t gripper_send_can_id;
    uint32_t gripper_recv_can_id;
};

// Global default motor configuration
// [openarmx port] Robstride motor types + CAN IDs (from openarmx_hardware v10_simple_hardware):
//   J1-2 RS04 (was DM8009), J3-4 RS03 (was DM4340), J5-7 RS00 (was DM4310), gripper RS00.
//   Robstride send/recv CAN IDs are the SAME per motor (0x01..0x07, gripper 0x08).
static const MotorConfig DEFAULT_MOTOR_CONFIG = {
    // 7-DOF arm motor types
    {openarmx::robstride_motor::MotorType::RS04, openarmx::robstride_motor::MotorType::RS04,
     openarmx::robstride_motor::MotorType::RS03, openarmx::robstride_motor::MotorType::RS03,
     openarmx::robstride_motor::MotorType::RS00, openarmx::robstride_motor::MotorType::RS00,
     openarmx::robstride_motor::MotorType::RS00},

    // arm CAN IDs (send == recv for Robstride)
    {0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07},
    {0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07},

    // gripper (RS00, CAN ID 0x08, recv == send)
    openarmx::robstride_motor::MotorType::RS00,
    0x08,
    0x08};

// opening function
inline void printOpenArmBanner() {
    std::cout << R"(

                                     ██████╗ ██████╗ ███████╗███╗   ██╗ █████╗ ██████╗ ███╗   ███╗
                                    ██╔═══██╗██╔══██╗██╔════╝████╗  ██║██╔══██╗██╔══██╗████╗ ████║
                                    ██║   ██║██████╔╝█████╗  ██╔██╗ ██║███████║██████╔╝██╔████╔██║
                                    ██║   ██║██╔═══╝ ██╔══╝  ██║╚██╗██║██╔══██║██╔══██╗██║╚██╔╝██║
                                    ╚██████╔╝██║     ███████╗██║ ╚████║██║  ██║██║  ██║██║ ╚═╝ ██║
                                     ╚═════╝ ╚═╝     ╚══════╝╚═╝  ╚═══╝╚═╝  ╚═╝╚═╝  ╚═╝╚═╝     ╚═╝

██████╗ ██╗██╗      █████╗ ████████╗███████╗██████╗  █████╗ ██╗          ██████╗ ██████╗ ███╗   ██╗████████╗██████╗  ██████╗ ██╗     ██╗██╗██╗██╗
██╔══██╗██║██║     ██╔══██╗╚══██╔══╝██╔════╝██╔══██╗██╔══██╗██║         ██╔════╝██╔═══██╗████╗  ██║╚══██╔══╝██╔══██╗██╔═══██╗██║     ██║██║██║██║
██████╔╝██║██║     ███████║   ██║   █████╗  ██████╔╝███████║██║         ██║     ██║   ██║██╔██╗ ██║   ██║   ██████╔╝██║   ██║██║     ██║██║██║██║
██╔══██╗██║██║     ██╔══██║   ██║   ██╔══╝  ██╔══██╗██╔══██║██║         ██║     ██║   ██║██║╚██╗██║   ██║   ██╔══██╗██║   ██║██║     ╚═╝╚═╝╚═╝╚═╝
██████╔╝██║███████╗██║  ██║   ██║   ███████╗██║  ██║██║  ██║███████╗    ╚██████╗╚██████╔╝██║ ╚████║   ██║   ██║  ██║╚██████╔╝███████╗██╗██╗██╗██╗
╚═════╝ ╚═╝╚══════╝╚═╝  ╚═╝   ╚═╝   ╚══════╝╚═╝  ╚═╝╚═╝  ╚═╝╚══════╝     ╚═════╝ ╚═════╝ ╚═╝  ╚═══╝   ╚═╝   ╚═╝  ╚═╝ ╚═════╝ ╚══════╝╚═╝╚═╝╚═╝╚═╝

    )" << std::endl;
}
