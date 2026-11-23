// Offline trot stability-margin analysis (issue #21, for #6 tuning).
//
// For each sampled phase, the stance feet (from the same trajectory
// math as the firmware) form a support segment (2 feet, normal trot)
// or polygon (3-4 feet when duty > 0.5 creates overlap). The margin
// is the distance from the body-centre ground projection to that
// support region: positive-inside for polygons, absolute distance
// for segments. A trot at duty 0.5 passes through near-zero margin
// twice per cycle (support line sweeps past the centre), so this
// tool is comparative, not a pass/fail gate: it shows how stride and
// duty change the margin profile before hardware tuning starts.
//
// Build/run: make stability (from software/)

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <vector>

#include "../gait.hpp"

namespace {

struct Pt { double x, y; };

double cross(const Pt& o, const Pt& a, const Pt& b) {
    return (a.x - o.x) * (b.y - o.y) - (a.y - o.y) * (b.x - o.x);
}

double dist_pt_seg(const Pt& p, const Pt& a, const Pt& b) {
    double dx = b.x - a.x, dy = b.y - a.y;
    double len2 = dx * dx + dy * dy;
    double t = len2 > 0 ? ((p.x - a.x) * dx + (p.y - a.y) * dy) / len2 : 0.0;
    t = std::max(0.0, std::min(1.0, t));
    double ex = a.x + t * dx - p.x, ey = a.y + t * dy - p.y;
    return std::sqrt(ex * ex + ey * ey);
}

std::vector<Pt> convex_hull(std::vector<Pt> pts) {
    std::sort(pts.begin(), pts.end(), [](const Pt& a, const Pt& b) {
        return a.x < b.x || (a.x == b.x && a.y < b.y);
    });
    pts.erase(std::unique(pts.begin(), pts.end(), [](const Pt& a, const Pt& b) {
                  return a.x == b.x && a.y == b.y;
              }),
              pts.end());
    if (pts.size() <= 1) return pts;
    std::vector<Pt> lo, hi;
    for (const Pt& p : pts) {
        while (lo.size() >= 2 && cross(lo[lo.size() - 2], lo.back(), p) <= 0)
            lo.pop_back();
        lo.push_back(p);
    }
    for (int i = (int)pts.size() - 1; i >= 0; --i) {
        const Pt& p = pts[i];
        while (hi.size() >= 2 && cross(hi[hi.size() - 2], hi.back(), p) <= 0)
            hi.pop_back();
        hi.push_back(p);
    }
    lo.pop_back();
    hi.pop_back();
    lo.insert(lo.end(), hi.begin(), hi.end());
    return lo;
}

// Signed margin of the origin w.r.t. the stance feet [mm].
double margin(const std::vector<Pt>& stance) {
    const Pt origin{0.0, 0.0};
    if (stance.empty()) return -100.0;
    if (stance.size() == 1) return -dist_pt_seg(origin, stance[0], stance[0]);
    if (stance.size() == 2) return dist_pt_seg(origin, stance[0], stance[1]);
    std::vector<Pt> hull = convex_hull(stance);
    double best = 1e9;
    bool inside = true;
    int n = hull.size();
    for (int i = 0; i < n; ++i) {
        const Pt& a = hull[i];
        const Pt& b = hull[(i + 1) % n];
        best = std::min(best, dist_pt_seg(origin, a, b));
        if (cross(a, b, origin) < 0) inside = false;
    }
    return inside ? best : -best;
}

void evaluate(const char* name, const quadruped::GaitConfig& cfg) {
    const double hip_x[4] = {45.0, 45.0, -45.0, -45.0};
    const double hip_y[4] = {36.0, -36.0, 36.0, -36.0};
    const int kSamples = 400;
    double min_m = 1e9, sum = 0.0;
    int near = 0;
    for (int s = 0; s < kSamples; ++s) {
        double base = (double)s / kSamples;
        std::vector<Pt> stance;
        for (int j = 0; j < 4; ++j) {
            double u = std::fmod(base + cfg.phase_offset[j], 1.0);
            if (u < cfg.duty) {
                auto [fx, fz] = quadruped::foot_position(
                    base + cfg.phase_offset[j],
                    static_cast<quadruped::Leg>(j), cfg);
                (void)fz;
                stance.push_back({hip_x[j] + fx, hip_y[j]});
            }
        }
        double m = margin(stance);
        min_m = std::min(min_m, m);
        sum += m;
        if (m < 3.0) ++near;
    }
    std::printf("%-28s min %6.2f mm   mean %6.2f mm   cycle <3mm: %3.0f%%\n",
                name, min_m, sum / kSamples, 100.0 * near / kSamples);
}

}  // namespace

int main() {
    using quadruped::GaitConfig;
    std::printf("Trot stability margin (body-centre projection, mm)\n");
    GaitConfig base;
    evaluate("default (stride36 duty.50)", base);
    GaitConfig d60 = base; d60.duty = 0.6;
    evaluate("duty 0.60", d60);
    GaitConfig d65 = base; d65.duty = 0.65;
    evaluate("duty 0.65", d65);
    GaitConfig s24 = base; s24.stride = 24.0;
    evaluate("stride 24", s24);
    GaitConfig s24d60 = base; s24d60.stride = 24.0; s24d60.duty = 0.6;
    evaluate("stride 24 + duty 0.60", s24d60);
    return 0;
}
