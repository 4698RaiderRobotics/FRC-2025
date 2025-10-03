#pragma once

#include <string>

#include <units/length.h>
#include <units/velocity.h>
#include <units/angular_velocity.h>
#include <units/voltage.h>
#include <units/current.h>

class ClimberIO {
public:
    struct Metrics {
        units::inch_t height = 0_in;
        units::inch_t goal = 0_in;
        units::feet_per_second_t velocity = 0_fps;
        units::volt_t appliedVolts = 0_V;
        units::ampere_t current = 0_A;

        units::revolutions_per_minute_t rollerVelocity = 0_rpm;
        units::volt_t rollerAppliedVolts = 0_V;
        units::ampere_t rollerCurrent = 0_A;

        bool homeSwitchTripped{false};
        bool cageSwitchTripped{false};

        bool doingClimbSequence{false};

        void Log( const std::string &key );
    };

    virtual void Update( Metrics &m ) =0;

    virtual void SetGoal( units::inch_t goal ) =0;
    virtual void SetOpenLoop( double percent ) =0;
    virtual void ResetHeight( ) =0;
    virtual void SetRollers( bool enable ) =0;
};
