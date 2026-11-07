// Trot gait generator.
//
// - Diagonal leg pairs (FL+RR / FR+RL) move with a phase difference of 0.5.
// - Foot trajectory: during stance the foot moves in a straight line along
//   the ground; during swing it follows the upper half of an ellipse
//   (semi-axes: stride/2 horizontally, step_height vertically).
// - Output: one cycle of target angles for 8 servos, as a vector of rows.
//   Each row has 8 values in degrees.
//   Column order: [FL hip, FL knee, FR hip, FR knee,
//                  RL hip, RL knee, RR hip, RR knee]
//
// Conversion to servo angles:
//   servo_deg = servo_center + dir * joint_deg + offset
//   dir / offset are calibration parameters that absorb mounting orientation
//   and assembly error. On the real robot they are tuned per leg by
//   measurement (especially the knee offset).

#pragma once

#include <array>
#include <utility>
#include <vector>

#include "leg_ik.hpp"

namespace quadruped {

// Leg indices. The order matches the servo column order in generate_cycle().
enum class Leg : int { FL = 0, FR = 1, RL = 2, RR = 3 };

inline constexpr int kLegCount = 4;
inline constexpr int kServoCount = 8;

const char* leg_name(Leg leg);

struct GaitConfig {
    LegGeometry geo;
    double hip_height = 95.0;    // hip height = foot z (straight down) [mm]
    double stride = 36.0;        // foot travel per cycle, front-back [mm]
    double step_height = 14.0;   // foot lift during swing [mm]
    double duty = 0.5;           // stance fraction (0.5 for trot)
    int steps_per_cycle = 40;    // samples per cycle
    double direction = 1.0;      // +1: forward, -1: backward
    int elbow_sign = +1;         // knee configuration (same as leg_ik)

    // Trot phase offsets (fraction of one cycle), indexed by Leg.
    // FL and RR are in phase; FR and RL are shifted by 0.5.
    std::array<double, kLegCount> phase_offset = {0.0, 0.5, 0.5, 0.0};

    // Servo conversion calibration parameters (tuned on the real robot),
    // indexed by Leg.
    double servo_center = 90.0;
    std::array<double, kLegCount> hip_dir = {1.0, 1.0, 1.0, 1.0};
    std::array<double, kLegCount> knee_dir = {1.0, 1.0, 1.0, 1.0};
    std::array<double, kLegCount> hip_offset = {0.0, 0.0, 0.0, 0.0};
    std::array<double, kLegCount> knee_offset = {-25.0, -25.0, -25.0, -25.0};
};

// Phase in [0,1) -> foot target (x, z) [mm], hip-relative, z downward (+).
std::pair<double, double> foot_position(double phase, const GaitConfig& cfg);

// Phase -> joint angles of one leg (a1, a2) [deg].
std::pair<double, double> leg_joint_angles(double phase, const GaitConfig& cfg);

// Joint angles [deg] -> servo command angles [deg] (hip, knee).
std::pair<double, double> to_servo_angles(Leg leg, double a1_deg, double a2_deg,
                                          const GaitConfig& cfg);

// One cycle of servo target angles: steps_per_cycle rows x 8 columns [deg].
std::vector<std::array<double, kServoCount>> generate_cycle(
    const GaitConfig& cfg);

// One cycle of foot trajectories: steps_per_cycle rows x 4 legs of (x, z)
// [mm]. Used for verification.
std::vector<std::array<std::pair<double, double>, kLegCount>>
generate_foot_positions(const GaitConfig& cfg);

}  // namespace quadruped
