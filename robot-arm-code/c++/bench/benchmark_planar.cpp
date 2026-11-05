// Accuracy benchmark: planar::naive_inverse (acos) vs planar::inverse (atan2 + area formula) in float.
//
// For each relative distance delta to a singularity (full stretch or full fold),
// random arms and targets are generated. The target is rounded to float first; the
// double reference is computed from exactly those rounded values, so the table shows
// the error added by each algorithm, not the unavoidable input rounding.
//
// Both solvers return two optional slots (elbow branches). Every slot that exists in
// the double reference is compared with the same slot of the float result:
//  - error columns: max absolute angle error in radians over all compared slots
//  - "missing":     the reference has the slot, the float solver does not
//                   (e.g. a NaN from acos, or the ground check flipping due to rounding)
//  - "extra":       the float solver has a slot the reference does not

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <numbers>
#include <random>
#include "kinematics/analytic/planar.hpp"

using namespace kinematics;

// Difference of two angles, wrapped into [-pi, pi]
static double angle_diff(double a, double b)
{
    return std::remainder(a - b, 2.0 * std::numbers::pi);
}

struct Stats {
    double max_phi1 = 0.0, max_phi2 = 0.0;
    int missing = 0, extra = 0;

    void add(const planar::Solutions<float>& sol, const planar::Solutions<double>& ref)
    {
        for (std::size_t slot = 0; slot < 2; ++slot) {
            if (!ref[slot]) { if (sol[slot]) ++extra; continue; }
            if (!sol[slot]) { ++missing; continue; }
            const auto& phi = *sol[slot];
            const auto& r   = *ref[slot];
            max_phi1 = std::max(max_phi1, std::abs(angle_diff(phi[0], r[0])));
            max_phi2 = std::max(max_phi2, std::abs(angle_diff(phi[1], r[1])));
        }
    }
};

enum class Singularity { stretch, fold };

static void run(Singularity sing, int samples_per_delta)
{
    std::mt19937 rng(1);
    std::uniform_real_distribution<double> uni(0.0, 1.0);
    const double pi = std::numbers::pi;

    std::printf("\n%s\n", sing == Singularity::stretch ? "Towards full stretch" : "Towards full fold");
    std::printf("%8s | %12s %12s %7s %5s | %12s %12s %7s %5s | %7s\n",
                "delta", "naive phi1", "naive phi2", "missing", "extra",
                "stable phi1", "stable phi2", "missing", "extra", "skipped");

    for (int k = 1; k <= 7; ++k) {
        const double delta = std::pow(10.0, -k);
        Stats naive, stable;
        int skipped = 0;

        for (int i = 0; i < samples_per_delta; ++i) {
            // Arm with R1 > R2, rounded to float; everything below uses the float values
            double a = 0.5 + 9.5 * uni(rng), b = 0.5 + 9.5 * uni(rng);
            float R1f = float(std::max(a, b)), R2f = float(std::min(a, b));
            if (R1f - R2f < 1e-2f) { ++skipped; continue; }
            double R1 = R1f, R2 = R2f;

            // Target at relative distance delta (relative to the arm length R1 + R2),
            // above the shoulder (the table), so the ground check usually keeps a slot
            double r = (sing == Singularity::stretch) ? (R1 + R2) - delta * (R1 + R2)
                                                      : (R1 - R2) + delta * (R1 + R2);
            double theta = pi * uni(rng);
            float xf = float(r * std::cos(theta)), yf = float(r * std::sin(theta));

            // Rounding may have moved the target onto or out of the workspace boundary
            double x = xf, y = yf, rr = std::hypot(x, y);
            if (rr >= R1 + R2 || rr <= R1 - R2) { ++skipped; continue; }

            const auto ref = planar::inverse(x, y, R1, R2);          // double reference
            naive.add(planar::naive_inverse(xf, yf, R1f, R2f), ref);
            stable.add(planar::inverse(xf, yf, R1f, R2f), ref);
        }
        std::printf("%8.0e | %12.2e %12.2e %7d %5d | %12.2e %12.2e %7d %5d | %7d\n", delta,
                    naive.max_phi1, naive.max_phi2, naive.missing, naive.extra,
                    stable.max_phi1, stable.max_phi2, stable.missing, stable.extra, skipped);
    }
}

int main()
{
    const int samples_per_delta = 20000;
    std::printf("float accuracy of the planar IK, %d samples per row, max abs error [rad]\n",
                samples_per_delta);
    run(Singularity::stretch, samples_per_delta);
    run(Singularity::fold, samples_per_delta);
    return 0;
}