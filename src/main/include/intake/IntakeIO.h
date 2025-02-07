#pragma once

#include <string>

#include <units/angle.h>
#include <units/angular_velocity.h>
#include <units/voltage.h>
#include <units/current.h>

class IntakeIO {
public:
    struct Metrics {
        units::radian_t upperPosition = 0_rad;
        units::radian_t lowerPosition = 0_rad;
        units::radians_per_second_t upperVelocity = 0_rad_per_s;
        units::radians_per_second_t lowerVelocity = 0_rad_per_s;
        units::volt_t upperAppliedVolts = 0_V;
        units::volt_t lowerAppliedVolts = 0_V;
        units::ampere_t upperCurrent = 0_A;
        units::ampere_t lowerCurrent = 0_A;
        bool centerBeam = false;
        bool endBeam = false;

        void logMetrics( const std::string &key );
    };

    virtual void UpdateMetrics( Metrics &m ) =0;

    virtual void SpinMotors( double upperSpeed, double lowerSpeed ) = 0;
    virtual bool isCenterBroken( ) = 0;
    virtual bool isEndBroken( ) = 0;
};
