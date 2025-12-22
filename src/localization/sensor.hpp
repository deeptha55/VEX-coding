#pragma once

#include <optional>
#include "localization/utils.hpp"

class Sensor {
public:
	// All sensors operate on the project's simple Vec3 pose type
	virtual std::optional<double> p(const Vec3& x) = 0;
	virtual void update() = 0;
	virtual ~Sensor() = default;
};