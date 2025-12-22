#pragma once

#include "localization/sensor.hpp"
#include "localization/utils.hpp"
#include "localization/config.hpp"

#include "api.h"

#include <optional>
#include <algorithm>
#include <cmath>

constexpr double WALL_POS_X =  72.0;
constexpr double WALL_NEG_X = -72.0;
constexpr double WALL_POS_Y =  72.0;
constexpr double WALL_NEG_Y = -72.0;

class Distance : public Sensor {
private:
    Vec3 sensorOffset;          // inches, radians
    pros::Distance distance;

    double measured = 0.0;      // inches
    double sigma = 1.0;         // noise
    bool exit = false;

    double tuningConstant;

public:
        Distance(const Vec3& sensor_offset,
                         double tuningConstant,
                         int distance_port)
                : sensorOffset(sensor_offset),
                    distance(distance_port),
                    tuningConstant(tuningConstant) {}

    void update() override {
        const double measuredMM = distance.get();
        exit = measuredMM >= 9999;

        // mm → inches
        measured = tuningConstant * measuredMM * 0.0393701;

        sigma = 0.2 * measured;
        if (sigma < 0.5) sigma = 0.5;
    }

    std::optional<double> p(const Vec3& X) override {
        if (exit) return std::nullopt;

        const double angle = X.theta + sensorOffset.theta;

        const double sx = X.x + std::cos(X.theta) * sensorOffset.x
                                - std::sin(X.theta) * sensorOffset.y;
        const double sy = X.y + std::sin(X.theta) * sensorOffset.x
                                + std::cos(X.theta) * sensorOffset.y;

        double predicted = 1e9;

        const double c = std::cos(angle);
        const double s = std::sin(angle);

        if (std::abs(c) > 1e-6) {
            if (c > 0) predicted = std::min(predicted, (WALL_POS_X - sx) / c);
            else       predicted = std::min(predicted, (WALL_NEG_X - sx) / c);
        }

        if (std::abs(s) > 1e-6) {
            if (s > 0) predicted = std::min(predicted, (WALL_POS_Y - sy) / s);
            else       predicted = std::min(predicted, (WALL_NEG_Y - sy) / s);
        }

        if (predicted < 0) return std::nullopt;

        const double error = (predicted - measured) / sigma;

        return cheap_norm_pdf(error) * LOCO_CONFIG::DISTANCE_WEIGHT;
    }

    ~Distance() override = default;
};