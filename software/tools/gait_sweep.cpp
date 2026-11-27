#include <algorithm>
#include <array>
#include <cmath>
#include <cstdio>
#include <limits>
#include <vector>

#include "../gait.hpp"

namespace {

constexpr int kSamples = 400;
constexpr double kInfinity = std::numeric_limits<double>::infinity();

struct Point {
    double x;
    double y;
};

double distance_to_segment(Point a, Point b) {
    const double dx = b.x - a.x;
    const double dy = b.y - a.y;
    const double length_squared = dx * dx + dy * dy;
    const double t = std::clamp(-(a.x * dx + a.y * dy) / length_squared,
                                0.0, 1.0);
    return std::hypot(a.x + t * dx, a.y + t * dy);
}

double support_margin(const std::array<Point, quadruped::kLegCount>& feet,
                      int count) {
    // Match stability_check: use unsigned distance for two-foot support.
    if (count == 2) return distance_to_segment(feet[0], feet[1]);

    double distance = kInfinity;
    bool inside = true;
    for (int i = 0; i < count; ++i) {
        const Point a = feet[i];
        const Point b = feet[(i + 1) % count];
        distance = std::min(distance, distance_to_segment(a, b));
        if (a.x * b.y - a.y * b.x < 0.0) inside = false;
    }
    return inside ? distance : -distance;
}

struct Result {
    double stride;
    double duty;
    double step_height;
    bool feasible = true;
    double min_angle = kInfinity;
    double max_angle = -kInfinity;
    double min_margin = kInfinity;
    double mean_margin = 0.0;
    double under_three_percent = 0.0;
};

Result evaluate(const quadruped::GaitConfig& cfg) {
    Result result{cfg.stride, cfg.duty, cfg.step_height};
    for (const auto& row : quadruped::generate_cycle(cfg)) {
        for (double angle : row) {
            result.min_angle = std::min(result.min_angle, angle);
            result.max_angle = std::max(result.max_angle, angle);
            result.feasible &= angle >= 0.0 && angle <= 180.0;
        }
    }

    using quadruped::Leg;
    // Visit hips counterclockwise. The swept strides keep front feet
    // ahead of rear feet, so this is also the stance polygon's order.
    constexpr std::array<Leg, quadruped::kLegCount> legs{
        Leg::FL, Leg::RL, Leg::RR, Leg::FR};
    constexpr std::array<Point, quadruped::kLegCount> hips{{
        {45.0, 36.0}, {-45.0, 36.0}, {-45.0, -36.0}, {45.0, -36.0}}};

    double margin_sum = 0.0;
    int under_three = 0;
    for (int sample = 0; sample < cfg.steps_per_cycle; ++sample) {
        const double phase = static_cast<double>(sample) / cfg.steps_per_cycle;
        std::array<Point, quadruped::kLegCount> stance{};
        int count = 0;
        for (int i = 0; i < quadruped::kLegCount; ++i) {
            const Leg leg = legs[i];
            const double leg_phase = phase + cfg.phase_offset[static_cast<int>(leg)];
            if (std::fmod(leg_phase, 1.0) < cfg.duty) {
                const double x = quadruped::foot_position(leg_phase, leg, cfg).first;
                stance[count++] = {hips[i].x + x, hips[i].y};
            }
        }
        const double margin = support_margin(stance, count);
        result.min_margin = std::min(result.min_margin, margin);
        margin_sum += margin;
        if (margin < 3.0) ++under_three;
    }
    result.mean_margin = margin_sum / cfg.steps_per_cycle;
    result.under_three_percent = 100.0 * under_three / cfg.steps_per_cycle;
    return result;
}

}  // namespace

int main() {
    std::vector<Result> results;
    quadruped::GaitConfig cfg;
    cfg.steps_per_cycle = kSamples;
    cfg.turn = 0.0;
    for (double stride : {24.0, 30.0, 36.0, 42.0}) {
        cfg.stride = stride;
        for (double duty : {0.50, 0.55, 0.60, 0.65}) {
            cfg.duty = duty;
            for (double height : {10.0, 14.0, 18.0}) {
                cfg.step_height = height;
                results.push_back(evaluate(cfg));
            }
        }
    }

    std::stable_sort(results.begin(), results.end(),
                     [](const Result& a, const Result& b) {
                         return a.mean_margin > b.mean_margin;
                     });
    std::puts("Stride(mm)  Duty  Step height(mm)  Status  Min(mm)  Mean(mm)  Under 3mm");
    for (const Result& result : results) {
        std::printf("%10.0f  %4.2f  %15.0f  %-6s  %7.2f  %8.2f  %8.2f%%",
                    result.stride, result.duty, result.step_height,
                    result.feasible ? "OK" : "REJECT", result.min_margin,
                    result.mean_margin, result.under_three_percent);
        if (!result.feasible) {
            std::printf("  servo min/max %.2f / %.2f deg",
                        result.min_angle, result.max_angle);
        }
        std::putchar('\n');
    }

    std::puts("\nTop 5 candidates");
    int rank = 0;
    for (const Result& result : results) {
        if (!result.feasible) continue;
        std::printf("%d. Stride %.0f mm, duty %.2f, step height %.0f mm, mean margin %.2f mm\n",
                    ++rank, result.stride, result.duty, result.step_height,
                    result.mean_margin);
        if (rank == 5) break;
    }
    if (rank == 0) std::puts("No feasible configurations");
}
