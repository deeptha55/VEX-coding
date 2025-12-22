#pragma once

#include "localization/config.hpp"
#include "localization/sensor.hpp"
#include "units/units.hpp"   // For Angle and unit conversions
#include "localization/utils.hpp"  // For cheap_norm_pdf and Vec3
#include "api.h"

// =======================
// GPS Sensor class (uses shared Vec3 from localization/utils.hpp)
// =======================
class GpsSensor : public Sensor {
private:
    pros::Gps gps;
    Angle sensorAngleOffset;
    Vec2 point{};  // from localization/utils.hpp (float x,y)
    double stdDev{0.0};
    bool notInstalled{false};

public:
    // Construct from a port number to avoid copying pros::Gps
    GpsSensor(const Angle sensorAngleOffset, int gps_port)
        : gps(gps_port), sensorAngleOffset(sensorAngleOffset) {}

    void update() override {
        notInstalled = !gps.is_installed() || gps.get_error() > 0.015;
    auto pos = gps.get_position();
    // pros::Gps::get_position() returns a gps_position_s_t with members
    // 'x' and 'y' (not .first/.second), so use those fields.
    double x = pos.x;
    double y = pos.y;
        point = {static_cast<float>(-y), static_cast<float>(x)};  // convert to field coordinates
        stdDev = gps.get_error() * 8.0;
    }

    std::optional<double> p(const Vec3& X) override {
        if (notInstalled) {
            return std::nullopt;
        }

        // X.x/X.y are floats in Vec3 from utils.hpp; convert to double for calculation
        double dx = static_cast<double>(X.x) - static_cast<double>(point.x);
        double dy = static_cast<double>(X.y) - static_cast<double>(point.y);
        double distanceEstimate = std::sqrt(dx * dx + dy * dy) / 2.0;
        return cheap_norm_pdf(distanceEstimate) * LOCO_CONFIG::GPS_WEIGHT;
    }

    Angle getAngle() {
        return -gps.get_yaw() * 1_deg - sensorAngleOffset;
    }

    pros::Gps& getGps() {
        return gps;
    }

    ~GpsSensor() override = default;
};