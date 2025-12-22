#pragma once

#include "localization/config.hpp"
#include "localization/sensor.hpp"
#include "localization/utils.hpp"
#include "units/units.hpp"
#include <cmath>
#include "api.h"
#include <optional>
#include <vector>
#include <algorithm>

// Use shared Vec2/Vec3 from localization/utils.hpp (simple POD float types)
// Rotate a 2D vector by angle (radians)
inline Vec2 rotate2D(const Vec2& vec, double angle) {
    double c = std::cos(angle);
    double s = std::sin(angle);
    return {static_cast<float>(vec.x * c - vec.y * s), static_cast<float>(vec.x * s + vec.y * c)};
}

// =======================
// LineSensor class
// =======================
class LineSensor : public Sensor {
private:
    Vec2 sensorOffset;
    pros::adi::LineSensor lineSensor;
    bool measured{false};

    // Line positions (y-values) in meters
    const std::vector<double> LINES_Y = {0.0, 1.47828, -1.47828};

public:
    LineSensor(const Vec2& sensor_offset, int line_port)
        : sensorOffset(sensor_offset), lineSensor(line_port) {}

    void update() override {
        measured = lineSensor.get_value() < static_cast<int>(LOCO_CONFIG::LINE_SENSOR_THRESHOLD);
    }

    std::optional<double> p(const Vec3& x) override {
        // Compute sensor position in field coordinates
    // X is the robot pose; utils::Vec3 uses 'theta' for the heading
    Vec2 pose{static_cast<float>(x.x), static_cast<float>(x.y)};
    Vec2 sensor_position = rotate2D(sensorOffset, x.theta);
    sensor_position.x += pose.x;
    sensor_position.y += pose.y;

        double predictedDistance = 50.0; // default large value in meters

        for (double line_y : LINES_Y) {
            predictedDistance = std::min(std::abs(sensor_position.y - line_y), predictedDistance);
        }

    // LOCO_CONFIG::LINE_SENSOR_DISTANCE_THRESHOLD is specified in inches
    // in config.hpp; convert to meters (1 in = 0.0254 m) for comparison
    bool predicted = predictedDistance < (LOCO_CONFIG::LINE_SENSOR_DISTANCE_THRESHOLD * 0.0254);

        if (predicted && measured) {
            return 1.0 * LOCO_CONFIG::LINE_WEIGHT;
        } else if (!predicted && !measured) {
            return 1.0 * LOCO_CONFIG::LINE_WEIGHT;
        } else {
            return 0.4 * LOCO_CONFIG::LINE_WEIGHT;
        }
    }

    ~LineSensor() override = default;
};