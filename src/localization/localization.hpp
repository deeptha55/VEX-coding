#pragma once

#include "localization/particle_filter.hpp"
#include "localization/distance.hpp"
#include "localization/gps.hpp"
#include "localization/line.hpp"
#include "units/units.hpp"
#include "api.h"

// Example number of particles
constexpr size_t NUM_PARTICLES = 100;

class Localization {
private:
    ParticleFilter<NUM_PARTICLES> pf;

    // Example sensors; replace ports with robot ports
    Distance distanceFront;
    GpsSensor gps;
    LineSensor lineLeft;
    LineSensor lineRight;

public:
    Localization()
        : pf([](){ return 0_deg; }), // Placeholder angle function; replace with gyro
          distanceFront({0.0, 0.0, 0.0}, 1.0, 1), // sensor offset, tuning constant, port
          gps(0_deg, 2), // angle offset, GPS port
          lineLeft({0.0f, 0.0f}, 3),  // sensor offset, port
          lineRight({0.0f, 0.0f}, 4) {

        // Add sensors to particle filter
        pf.addSensor(&distanceFront);
        pf.addSensor(&gps);
        pf.addSensor(&lineLeft);
        pf.addSensor(&lineRight);

        // Initialize particles (uniform across field)
        pf.initUniform(-72_in, -72_in, 72_in, 72_in);
    }

    // Update sensors only
    void updateSensors() {
        pf.updateSensors();
    }

    // Update particle filter with robot motion (delta movement)
    // Change robotDelta to be computed from odometry/IMU
    void update(const Vec2& robotDelta) {
        pf.update([&]() -> Vec2 { return robotDelta; });
    }

    // Get current predicted robot position
    Vec3 getPose() {
        return pf.getPrediction();
    }

    // Access particles for debugging/visualization
    std::array<Vec3, NUM_PARTICLES> getParticles() {
        return pf.getParticles();
    }

    Angle getAngle() {
        return pf.getAngle();
    }
};