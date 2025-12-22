#pragma once

#include "units/units.hpp"
#include "localization/sensor.hpp"
#include "localization/utils.hpp"
#include <array>
#include <vector>
#include <random>
#include <functional>
#include <algorithm>
#include <iostream>

// =======================
// Vec structs (if not already included globally)
struct Vec2 {
    double x;
    double y;

    Vec2 operator+(const Vec2& other) const {
        return {x + other.x, y + other.y};
    }

    Vec2 operator-(const Vec2& other) const {
        return {x - other.x, y - other.y};
    }

    Vec2 operator*(double scalar) const {
        return {x * scalar, y * scalar};
    }

    double norm() const {
        return std::sqrt(x*x + y*y);
    }
};

struct Vec3 {
    double x;
    double y;
    double z; // angle in radians
};

// Rotate a 2D vector by angle (radians)
inline Vec2 rotate2D(const Vec2& vec, double angle) {
    double c = cos(angle);
    double s = sin(angle);
    return {vec.x * c - vec.y * s, vec.x * s + vec.y * c};
}

// =======================
// Particle Filter Class
// =======================
template<size_t L>
class ParticleFilter {
    static_assert(L <= 500, "Too many particles: limit to 500 or less");

private:
    std::array<std::array<double, 2>, L> particles;
    std::array<std::array<double, 2>, L> oldParticles;
    std::array<double, L> weights;

    Vec3 prediction{};

    std::vector<Sensor*> sensors;

    QLength distanceSinceUpdate = 0.0;
    QTime lastUpdateTime = 0.0;

    QLength maxDistanceSinceUpdate = 2_in;
    QTime maxUpdateInterval = 2.0_s;

    std::function<Angle()> angleFunction;
    std::ranlux24_base de;

    std::uniform_real_distribution<> fieldDist{-1.78308, 1.78308};

public:
    explicit ParticleFilter(std::function<Angle()> angle_function)
        : angleFunction(std::move(angle_function)) {
        for (auto& particle : particles) {
            particle[0] = 0.0;
            particle[1] = 0.0;
        }
    }

    Vec3 getPrediction() {
        return prediction;
    }

    std::array<Vec3, L> getParticles() {
        std::array<Vec3, L> out;
        double angle = angleFunction().convert(radian);
        for (size_t i = 0; i < L; i++) {
            out[i] = {particles[i][0], particles[i][1], angle};
        }
        return out;
    }

    Vec3 getParticle(size_t i) {
        return {particles[i][0], particles[i][1], angleFunction().convert(radian)};
    }

    float weightParticle(const Vec3& p) {
        float totalWeight = 1.0f;
        for (auto sensor : sensors) {
            if (auto w = sensor->p(p); w.has_value() && std::isfinite(w.value())) {
                totalWeight *= static_cast<float>(w.value());
            }
        }
        return totalWeight;
    }

    void updateSensors() {
        for (auto& sensor : sensors) {
            sensor->update();
        }
    }

    void update(const std::function<Vec2()>& predictionFunction) {
        if (!std::isfinite(angleFunction().convert(radian))) return;

        Vec2 pred = predictionFunction();

        for (auto& particle : particles) {
            particle[0] += pred.x;
            particle[1] += pred.y;
        }

        distanceSinceUpdate += pred.norm();

        if (distanceSinceUpdate < maxDistanceSinceUpdate && maxUpdateInterval > pros::millis() * millisecond - lastUpdateTime) {
            double xSum = 0.0, ySum = 0.0;
            for (size_t i = 0; i < L; i++) {
                xSum += particles[i][0];
                ySum += particles[i][1];
            }
            prediction = {xSum / L, ySum / L, angleFunction().convert(radian)};
            return;
        }

        updateSensors();

        double totalWeight = 0.0;
        for (size_t i = 0; i < L; i++) {
            if (outOfField(particles[i])) {
                particles[i][0] = fieldDist(de);
                particles[i][1] = fieldDist(de);
            }

            Vec3 p{particles[i][0], particles[i][1], angleFunction().convert(radian)};
            weights[i] = weightParticle(p);
            totalWeight += weights[i];
        }

        if (totalWeight == 0.0) {
            std::cout << "Warning: Total weight equal to 0" << std::endl;
            return;
        }

        double avgWeight = totalWeight / L;
        std::uniform_real_distribution<double> distribution(0.0, avgWeight);
        double randWeight = distribution(de);

        for (size_t i = 0; i < L; i++) {
            oldParticles[i] = particles[i];
        }

        size_t j = 0;
        double cumulativeWeight = 0.0;
        double xSum = 0.0, ySum = 0.0;

        for (size_t i = 0; i < L; i++) {
            double w = i * avgWeight + randWeight;
            while (cumulativeWeight < w && j < L) {
                cumulativeWeight += weights[j];
                j++;
            }
            particles[i][0] = oldParticles[j-1][0];
            particles[i][1] = oldParticles[j-1][1];
            xSum += particles[i][0];
            ySum += particles[i][1];
        }

        prediction = {xSum / L, ySum / L, angleFunction().convert(radian)};
        lastUpdateTime = pros::millis() * millisecond;
        distanceSinceUpdate = 0.0;
    }

    static bool outOfField(const std::array<double, 2>& v) {
        return v[0] > 1.78308 || v[0] < -1.78308 || v[1] < -1.78308 || v[1] > 1.78308;
    }

    void initUniform(const QLength minX, const QLength minY, const QLength maxX, const QLength maxY) {
        std::uniform_real_distribution<double> xDistribution(minX.getValue(), maxX.getValue());
        std::uniform_real_distribution<double> yDistribution(minY.getValue(), maxY.getValue());

        for (auto& particle : particles) {
            particle[0] = xDistribution(de);
            particle[1] = yDistribution(de);
        }
    }

    void addSensor(Sensor* sensor) {
        sensors.emplace_back(sensor);
    }

    Angle getAngle() {
        return angleFunction();
    }
};