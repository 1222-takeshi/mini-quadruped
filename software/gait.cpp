#include "gait.hpp"

#include <cmath>

namespace quadruped {

namespace {
constexpr double kPi = 3.14159265358979323846;
constexpr double kRadToDeg = 180.0 / kPi;

int idx(Leg leg) { return static_cast<int>(leg); }
}  // namespace

const char* leg_name(Leg leg) {
    switch (leg) {
        case Leg::FL: return "FL";
        case Leg::FR: return "FR";
        case Leg::RL: return "RL";
        case Leg::RR: return "RR";
    }
    return "??";
}

std::pair<double, double> foot_position(double phase, const GaitConfig& cfg) {
    double u = std::fmod(phase, 1.0);
    if (u < 0.0) u += 1.0;
    const double half = cfg.stride / 2.0;
    double x;
    double lift;
    if (u < cfg.duty) {
        // Stance: +half -> -half at constant speed (body moves forward).
        x = half - cfg.stride * (u / cfg.duty);
        lift = 0.0;
    } else {
        // Swing: upper half of an ellipse (-half -> +half).
        const double v = (u - cfg.duty) / (1.0 - cfg.duty);
        x = -half * std::cos(kPi * v);
        lift = cfg.step_height * std::sin(kPi * v);
    }
    return {cfg.direction * x, cfg.hip_height - lift};
}

std::pair<double, double> leg_joint_angles(double phase, const GaitConfig& cfg) {
    const auto [x, z] = foot_position(phase, cfg);
    const auto [a1, a2] = inverse_kinematics(x, z, cfg.geo, cfg.elbow_sign);
    return {a1 * kRadToDeg, a2 * kRadToDeg};
}

std::pair<double, double> to_servo_angles(Leg leg, double a1_deg, double a2_deg,
                                          const GaitConfig& cfg) {
    const int i = idx(leg);
    const double hip = cfg.servo_center + cfg.hip_dir[i] * a1_deg + cfg.hip_offset[i];
    const double knee = cfg.servo_center + cfg.knee_dir[i] * a2_deg + cfg.knee_offset[i];
    return {hip, knee};
}

std::vector<std::array<double, kServoCount>> generate_cycle(
    const GaitConfig& cfg) {
    std::vector<std::array<double, kServoCount>> out(cfg.steps_per_cycle);
    for (int i = 0; i < cfg.steps_per_cycle; ++i) {
        const double base_phase =
            static_cast<double>(i) / cfg.steps_per_cycle;
        for (int j = 0; j < kLegCount; ++j) {
            const Leg leg = static_cast<Leg>(j);
            const auto [a1, a2] =
                leg_joint_angles(base_phase + cfg.phase_offset[j], cfg);
            const auto [hip, knee] = to_servo_angles(leg, a1, a2, cfg);
            out[i][2 * j] = hip;
            out[i][2 * j + 1] = knee;
        }
    }
    return out;
}

std::vector<std::array<std::pair<double, double>, kLegCount>>
generate_foot_positions(const GaitConfig& cfg) {
    std::vector<std::array<std::pair<double, double>, kLegCount>> out(
        cfg.steps_per_cycle);
    for (int i = 0; i < cfg.steps_per_cycle; ++i) {
        const double base_phase =
            static_cast<double>(i) / cfg.steps_per_cycle;
        for (int j = 0; j < kLegCount; ++j) {
            out[i][j] = foot_position(base_phase + cfg.phase_offset[j], cfg);
        }
    }
    return out;
}

}  // namespace quadruped
