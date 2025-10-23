#pragma once

#include <string>

#include <units/angle.h>
#include <units/angular_velocity.h>
#include <units/voltage.h>
#include <units/current.h>

class ClimberIO {
public:
    struct Metrics {
        units::degree_t angle = 0_deg;
        units::degree_t goal = 0_deg;
        units::revolutions_per_minute_t velocity = 0_rpm;
        units::volt_t appliedVolts = 0_V;
        units::ampere_t current = 0_A;

        units::revolutions_per_minute_t rollerVelocity = 0_rpm;
        units::volt_t rollerAppliedVolts = 0_V;
        units::ampere_t rollerCurrent = 0_A;

        bool cageSwitchTripped{false};

        bool doingClimbSequence{false};

        void Log( const std::string &key );
    };

    virtual void Update( Metrics &m ) =0;

    virtual void SetGoal( units::degree_t goal ) =0;
    virtual void SetOpenLoop( double percent ) =0;
    virtual void ResetAngle( ) =0;
    virtual void SetRollers( bool enable ) =0;
};
