#include "leg_ik.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace quadruped {

std::pair<double, double> forward_kinematics(double a1, double a2,
                                             const LegGeometry& geo) {
    const double x = geo.l1 * std::sin(a1) + geo.l2 * std::sin(a1 + a2);
    const double z = geo.l1 * std::cos(a1) + geo.l2 * std::cos(a1 + a2);
    return {x, z};
}

std::pair<double, double> inverse_kinematics(double x, double z,
                                             const LegGeometry& geo,
                                             int elbow_sign) {
    if (elbow_sign != +1 && elbow_sign != -1) {
        throw std::invalid_argument("elbow_sign must be +1 or -1");
    }

    const double r2 = x * x + z * z;
    const double r = std::sqrt(r2);
    if (r > geo.max_reach()) {
        throw std::invalid_argument("unreachable: target beyond max reach");
    }
    if (r < geo.min_reach()) {
        throw std::invalid_argument("unreachable: target inside min reach");
    }
    if (r < 1e-9) {
        throw std::invalid_argument("unreachable: target at hip origin");
    }

    double cos_a2 = (r2 - geo.l1 * geo.l1 - geo.l2 * geo.l2) /
                    (2.0 * geo.l1 * geo.l2);
    cos_a2 = std::max(-1.0, std::min(1.0, cos_a2));  // clamp numeric error
    const double a2 = elbow_sign * std::acos(cos_a2);

    // a1: direction from origin to foot (measured from straight down),
    // minus the knee offset term.
    const double a1 = std::atan2(x, z) -
                      std::atan2(geo.l2 * std::sin(a2),
                                 geo.l1 + geo.l2 * std::cos(a2));
    return {a1, a2};
}

}  // namespace quadruped
