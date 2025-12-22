#pragma once
#include <cmath>

// Small lightweight vector types used by the localization code.
// These are intentionally simple PODs (floats) to keep conversions
// between Eigen and this project's types cheap.
struct Vec2 {
    float x{0.0f};
    float y{0.0f};
};

struct Vec3 {
    float x{0.0f};
    float y{0.0f};
    float theta{0.0f};
};

// Simple Gaussian-like function used by sensors to score distances.
inline double cheap_norm_pdf(double x) {
    return std::exp(-0.5 * x * x);
}

// Angle difference function (returns difference constrained to [-pi, pi])
inline double angleDifference(double a, double b) {
    double diff = a - b;
    while (diff > M_PI) diff -= 2 * M_PI;
    while (diff < -M_PI) diff += 2 * M_PI;
    return diff;
}