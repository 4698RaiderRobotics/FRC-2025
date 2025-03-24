#pragma once

#include <string>

#include <units/length.h>
#include <units/velocity.h>
#include <units/voltage.h>
#include <units/current.h>

class ElevatorIO {
public:
    struct Metrics {
        units::inch_t height = 0_in;
        units::inch_t goal = 0_in;
        units::feet_per_second_t velocity = 0_fps;
        units::volt_t M1AppliedVolts = 0_V;
        units::ampere_t M1Current = 0_A;
        units::volt_t M2AppliedVolts = 0_V;
        units::ampere_t M2Current = 0_A;

        void Log( const std::string &key );
    };

    virtual void Update( Metrics &m ) =0;

    virtual void ResetPosition( units::inch_t position ) =0;
    virtual void SetOpenLoop( double percentOutput ) =0;
    virtual void SetGoal( units::inch_t goal ) =0;
};
