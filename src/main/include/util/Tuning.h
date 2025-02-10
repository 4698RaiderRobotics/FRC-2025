#pragma once 

#include <units/length.h>
#include <units/velocity.h>

namespace units {
UNIT_ADD(jerk, meters_per_second_cubed, meters_per_second_cubed,
         mps_cu, compound_unit<length::meter, inverse<cubed<time::seconds>>>)
UNIT_ADD(jerk, feet_per_second_cubed, feet_per_second_cubed,
         fps_cu, compound_unit<length::feet, inverse<cubed<time::seconds>>>)
}

/**
 * Controller Parameters kP, kI, kD, kS, kG, kV, kA
*/
struct TuningParams {
    double kP;
    double kI;
    double kD;
    double kS;
    double kG;
    double kV;
    double kA;
};

template<class Distance> struct MotionParams {
    using Distance_t = units::unit_t<Distance>;
    using Velocity =
        units::compound_unit<Distance, units::inverse<units::seconds>>;
    using Velocity_t = units::unit_t<Velocity>;
    using Acceleration =
        units::compound_unit<Velocity, units::inverse<units::seconds>>;
    using Acceleration_t = units::unit_t<Acceleration>;
    using Jerk =
        units::compound_unit<Acceleration, units::inverse<units::seconds>>;
    using Jerk_t = units::unit_t<Jerk>;

    Velocity_t MaxVelocity;
    Acceleration_t MaxAcceleration;
    Jerk_t MaxJerk;
};

template<class Distance> struct MotionConfig {
    TuningParams tuner;
    MotionParams<Distance> mp;
};
