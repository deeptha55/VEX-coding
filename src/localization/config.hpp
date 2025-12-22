#pragma once

#include <cstddef>

// MCL Code


namespace LOCO_CONFIG {

    // Line sensor detection threshold (raw value)
    constexpr std::size_t LINE_SENSOR_THRESHOLD = 2000;

    // Distance (inches) at which line sensor data is considered valid
    constexpr double LINE_SENSOR_DISTANCE_THRESHOLD = 3.0;

    // Sensor weighting factors
    constexpr double DISTANCE_WEIGHT = 1.0;
    constexpr double GPS_WEIGHT = 1.0;
    constexpr double LINE_WEIGHT = 1.0;

}