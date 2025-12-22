// Minimal, standalone units compatibility header used by localization
// and a few other small modules. This intentionally does NOT depend on
// Okapi — it provides tiny lightweight types and literals used by the
// project so headers can remain unit-testable / build without the
// full Okapi unit system.
#pragma once

#include <cmath>
#include <cstdint>

// --- Angle ---
struct Angle {
	double rad{0.0};
	constexpr Angle() noexcept = default;
	constexpr explicit Angle(double r) noexcept : rad(r) {}
	double getValue() const noexcept { return rad; }
};

// scale an Angle by a scalar (used like: double_value * 1_deg)
inline Angle operator*(double scalar, const Angle &a) noexcept { return Angle(scalar * a.rad); }
inline Angle operator-(const Angle &a, const Angle &b) noexcept { return Angle(a.rad - b.rad); }
inline Angle operator+(const Angle &a, const Angle &b) noexcept { return Angle(a.rad + b.rad); }

// user-defined literals for degrees -> radians
inline Angle operator"" _deg(long double d) noexcept { return Angle(static_cast<double>(d * M_PI / 180.0)); }
inline Angle operator"" _deg(unsigned long long d) noexcept { return Angle(static_cast<double>(d) * M_PI / 180.0); }

// --- QLength (lengths, stored in meters) ---
struct QLength {
	double meters{0.0};
	constexpr QLength() noexcept = default;
	constexpr explicit QLength(double m) noexcept : meters(m) {}
	double getValue() const noexcept { return meters; }
};

// inches to meters literal
inline QLength operator"" _in(long double v) noexcept { return QLength(static_cast<double>(v) * 0.0254); }
inline QLength operator"" _in(unsigned long long v) noexcept { return QLength(static_cast<double>(v) * 0.0254); }

// --- QTime (time, stored in seconds) ---
struct QTime {
	double seconds{0.0};
	constexpr QTime() noexcept = default;
	constexpr explicit QTime(double s) noexcept : seconds(s) {}
	double getValue() const noexcept { return seconds; }
	explicit operator double() const noexcept { return seconds; }
};

// time literals: seconds and milliseconds
inline QTime operator"" _s(long double v) noexcept { return QTime(static_cast<double>(v)); }
inline QTime operator"" _s(unsigned long long v) noexcept { return QTime(static_cast<double>(v)); }
inline QTime operator"" _ms(long double v) noexcept { return QTime(static_cast<double>(v) / 1000.0); }
inline QTime operator"" _ms(unsigned long long v) noexcept { return QTime(static_cast<double>(v) / 1000.0); }

// convenient named unit
inline constexpr QTime millisecond{0.001};

// arithmetic for QTime
inline QTime operator+(const QTime &a, const QTime &b) noexcept { return QTime(a.seconds + b.seconds); }
inline QTime operator-(const QTime &a, const QTime &b) noexcept { return QTime(a.seconds - b.seconds); }
inline QTime operator*(double scalar, const QTime &t) noexcept { return QTime(scalar * t.seconds); }
inline bool operator>(const QTime &a, const QTime &b) noexcept { return a.seconds > b.seconds; }
inline bool operator<(const QTime &a, const QTime &b) noexcept { return a.seconds < b.seconds; }

// Provide short aliases expected by localization code
using Angle_t = Angle; // (not used widely, but harmless)
using QLength_t = QLength;

// The project historically referenced the following type names — provide
// them so localization code can keep the same identifiers.
using Angle = ::Angle;
using QLength = ::QLength;
using QTime = ::QTime;

