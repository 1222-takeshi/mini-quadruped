// Verification binary for leg_ik and gait (numeric checks only, no drawing).
//
// Checks:
//   [IK] FK<->IK roundtrip (both elbow configurations), knee-back
//        configuration, unreachable targets throw, alternate geometry.
//   [1] Foot trajectory continuity (per-step travel, seam included)
//   [2] Servo angle continuity (per-step change, seam included)
//   [3] Servo angle range within [0, 180] deg
//   [4] Periodicity: state at phase 1.0 matches phase 0.0
//   [5] Diagonal pair: FL and RR servo angles are identical (trot)
//
// Tolerances follow the original numeric check: foot step <= 8 mm,
// servo step <= 12 deg, period tolerance 1e-6 deg, IK roundtrip 1e-9.
//
// Exit code 0 if everything passes, 1 otherwise.

#include <array>
#include <cmath>
#include <cstdio>
#include <stdexcept>
#include <string>

#include "../gait.hpp"
#include "../leg_ik.hpp"

namespace {

using namespace quadruped;

constexpr double kFootStepMaxMm = 8.0;
constexpr double kServoStepMaxDeg = 12.0;
constexpr double kServoMinDeg = 0.0;
constexpr double kServoMaxDeg = 180.0;
constexpr double kPeriodTolDeg = 1e-6;
constexpr double kIkTol = 1e-9;
constexpr double kDegToRad = 3.14159265358979323846 / 180.0;

int g_failures = 0;

void check(bool ok, const std::string& label) {
    std::printf("    %s  %s\n", ok ? "OK" : "NG", label.c_str());
    if (!ok) ++g_failures;
}

bool near(double a, double b, double tol) { return std::fabs(a - b) <= tol; }

void test_leg_ik() {
    std::printf("[IK] leg_ik roundtrip and error handling\n");
    const LegGeometry geo{50.0, 60.0};

    // Joint angles -> FK -> IK returns the original angles (both elbows).
    for (int elbow_sign : {+1, -1}) {
        double max_err = 0.0;
        for (int a1_deg = -60; a1_deg <= 60; a1_deg += 15) {
            for (int a2_deg = 10; a2_deg <= 120; a2_deg += 10) {
                const int signed_a2 = elbow_sign < 0 ? -a2_deg : a2_deg;
                const double a1 = a1_deg * kDegToRad;
                const double a2 = signed_a2 * kDegToRad;
                const auto [x, z] = forward_kinematics(a1, a2, geo);
                const auto [b1, b2] = inverse_kinematics(x, z, geo, elbow_sign);
                max_err = std::max(max_err, std::fabs(b1 - a1));
                max_err = std::max(max_err, std::fabs(b2 - a2));
            }
        }
        char buf[128];
        std::snprintf(buf, sizeof(buf),
                      "FK->IK roundtrip (elbow_sign=%+d): max err %.2e rad",
                      elbow_sign, max_err);
        check(max_err <= kIkTol, buf);
    }

    // Reachable foot positions -> IK -> FK returns the original position.
    for (int elbow_sign : {+1, -1}) {
        double max_err = 0.0;
        for (int xi = -60; xi <= 60; xi += 10) {
            for (double z : {40.0, 60.0, 80.0, 95.0, 105.0}) {
                const double x = static_cast<double>(xi);
                const double r = std::hypot(x, z);
                if (r > geo.max_reach() || r < geo.min_reach()) continue;
                const auto [a1, a2] = inverse_kinematics(x, z, geo, elbow_sign);
                const auto [x2, z2] = forward_kinematics(a1, a2, geo);
                max_err = std::max(max_err, std::fabs(x2 - x));
                max_err = std::max(max_err, std::fabs(z2 - z));
            }
        }
        char buf[128];
        std::snprintf(buf, sizeof(buf),
                      "IK->FK roundtrip (elbow_sign=%+d): max err %.2e mm",
                      elbow_sign, max_err);
        check(max_err <= kIkTol, buf);
    }

    // With elbow_sign=+1 and the foot straight below, the knee points back.
    {
        const auto [a1, a2] = inverse_kinematics(0.0, 95.0, geo, +1);
        const double knee_x = geo.l1 * std::sin(a1);
        check(a2 > 0.0 && knee_x < 0.0,
              "knee-back configuration: a2 > 0 and knee_x < 0");
    }

    // Out-of-reach targets throw.
    {
        int thrown = 0;
        const std::array<std::pair<double, double>, 3> targets = {
            std::make_pair(0.0, geo.max_reach() + 1.0),
            std::make_pair(200.0, 0.0),
            std::make_pair(0.0, 0.0),
        };
        for (const auto& [x, z] : targets) {
            try {
                inverse_kinematics(x, z, geo);
            } catch (const std::invalid_argument&) {
                ++thrown;
            }
        }
        check(thrown == 3, "unreachable targets throw std::invalid_argument");
    }

    // Link-length parameters are honored (roundtrip with other dimensions).
    {
        const LegGeometry other{35.0, 45.0};
        const auto [x, z] = forward_kinematics(0.3, 0.8, other);
        const auto [a1, a2] = inverse_kinematics(x, z, other);
        check(near(a1, 0.3, kIkTol) && near(a2, 0.8, kIkTol),
              "geometry parameters: roundtrip with L1=35, L2=45");
    }
    std::printf("\n");
}

void test_gait() {
    const GaitConfig cfg;
    const auto servo = generate_cycle(cfg);
    const auto feet = generate_foot_positions(cfg);
    const int steps = cfg.steps_per_cycle;

    std::printf("Config: steps/cycle=%d, stride=%.0fmm, step_height=%.0fmm, "
                "hip_height=%.0fmm, duty=%.1f\n\n",
                steps, cfg.stride, cfg.step_height, cfg.hip_height, cfg.duty);

    // [1] Foot trajectory continuity (seam included).
    std::printf("[1] Foot trajectory continuity (per-step travel, mm)\n");
    for (int j = 0; j < kLegCount; ++j) {
        double m = 0.0;
        for (int i = 0; i < steps; ++i) {
            const auto& cur = feet[i][j];
            const auto& nxt = feet[(i + 1) % steps][j];
            m = std::max(m, std::hypot(nxt.first - cur.first,
                                       nxt.second - cur.second));
        }
        char buf[128];
        std::snprintf(buf, sizeof(buf), "%s: max=%.2f  (limit %.1f)",
                      leg_name(static_cast<Leg>(j)), m, kFootStepMaxMm);
        check(m <= kFootStepMaxMm, buf);
    }
    std::printf("\n");

    // Servo column names in column order.
    std::array<std::string, kServoCount> names;
    for (int j = 0; j < kLegCount; ++j) {
        names[2 * j] = std::string(leg_name(static_cast<Leg>(j))) + " hip";
        names[2 * j + 1] = std::string(leg_name(static_cast<Leg>(j))) + " knee";
    }

    // [2] Servo angle continuity (seam included).
    std::printf("[2] Servo angle continuity (per-step change, deg)\n");
    for (int c = 0; c < kServoCount; ++c) {
        double m = 0.0;
        for (int i = 0; i < steps; ++i) {
            m = std::max(m, std::fabs(servo[(i + 1) % steps][c] - servo[i][c]));
        }
        char buf[128];
        std::snprintf(buf, sizeof(buf), "%s: max=%.2f  (limit %.1f)",
                      names[c].c_str(), m, kServoStepMaxDeg);
        check(m <= kServoStepMaxDeg, buf);
    }
    std::printf("\n");

    // [3] Servo angle range.
    std::printf("[3] Servo angle range (deg)\n");
    for (int c = 0; c < kServoCount; ++c) {
        double lo = servo[0][c], hi = servo[0][c];
        for (int i = 0; i < steps; ++i) {
            lo = std::min(lo, servo[i][c]);
            hi = std::max(hi, servo[i][c]);
        }
        char buf[128];
        std::snprintf(buf, sizeof(buf), "%s: %.1f .. %.1f  (allowed %.1f..%.1f)",
                      names[c].c_str(), lo, hi, kServoMinDeg, kServoMaxDeg);
        check(kServoMinDeg <= lo && hi <= kServoMaxDeg, buf);
    }
    std::printf("\n");

    // [4] Periodicity: servo angles at phase 1.0 + offset match row 0.
    std::printf("[4] Periodicity (phase 1.0 vs phase 0.0)\n");
    {
        double max_err = 0.0;
        for (int j = 0; j < kLegCount; ++j) {
            const Leg leg = static_cast<Leg>(j);
            const auto [a1, a2] =
                leg_joint_angles(1.0 + cfg.phase_offset[j], cfg);
            const auto [hip, knee] = to_servo_angles(leg, a1, a2, cfg);
            max_err = std::max(max_err, std::fabs(hip - servo[0][2 * j]));
            max_err = std::max(max_err, std::fabs(knee - servo[0][2 * j + 1]));
        }
        char buf[128];
        std::snprintf(buf, sizeof(buf), "max diff vs first row: %.2e deg  "
                                         "(limit %.0e)",
                      max_err, kPeriodTolDeg);
        check(max_err <= kPeriodTolDeg, buf);
    }
    std::printf("\n");

    // [5] Diagonal pair: FL (columns 0-1) and RR (columns 6-7) are identical.
    std::printf("[5] Diagonal pair consistency (trot)\n");
    {
        double max_err = 0.0;
        for (int i = 0; i < steps; ++i) {
            max_err = std::max(max_err, std::fabs(servo[i][0] - servo[i][6]));
            max_err = std::max(max_err, std::fabs(servo[i][1] - servo[i][7]));
        }
        char buf[128];
        std::snprintf(buf, sizeof(buf),
                      "FL vs RR max angle diff: %.2e deg (0 for trot)", max_err);
        check(max_err <= 1e-9, buf);
    }
    std::printf("\n");
}

}  // namespace

int main() {
    test_leg_ik();
    test_gait();
    std::printf("Result: %s\n", g_failures == 0 ? "ALL PASS" : "FAILURES PRESENT");
    return g_failures == 0 ? 0 : 1;
}
