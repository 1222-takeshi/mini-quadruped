// Two-link leg inverse kinematics (hip pitch + knee).
//
// Coordinate and angle conventions (within the leg's sagittal plane):
//   - Origin at the hip joint. x = forward (+), z = downward (+)
//   - a1 (hip pitch): angle of the upper link measured from the straight-down
//     direction. Leaning forward is +.
//   - a2 (knee): angle of the lower link relative to the upper link.
//     With elbow_sign = +1 (default) the knee points backward.
//   - Units: radians internally, lengths in mm.
//
// Forward kinematics:
//   x = L1*sin(a1) + L2*sin(a1+a2)
//   z = L1*cos(a1) + L2*cos(a1+a2)

#pragma once

#include <utility>

namespace quadruped {

// Link lengths of one leg [mm].
struct LegGeometry {
    double l1 = 50.0;  // upper link (hip -> knee)
    double l2 = 60.0;  // lower link (knee -> foot tip)

    double max_reach() const { return l1 + l2; }
    double min_reach() const { return l1 > l2 ? l1 - l2 : l2 - l1; }
};

// Joint angles [rad] -> foot tip position (x, z) [mm].
std::pair<double, double> forward_kinematics(double a1, double a2,
                                             const LegGeometry& geo);

// Foot tip position (x, z) [mm] -> joint angles (a1, a2) [rad].
//
// elbow_sign = +1: knee points backward (default for quadruped robots)
// elbow_sign = -1: knee points forward
// Throws std::invalid_argument if the target is unreachable or
// elbow_sign is not +1 / -1.
std::pair<double, double> inverse_kinematics(double x, double z,
                                             const LegGeometry& geo,
                                             int elbow_sign = +1);

}  // namespace quadruped
