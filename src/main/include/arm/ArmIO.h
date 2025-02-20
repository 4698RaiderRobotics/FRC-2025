#pragma once

#include <string>

#include <units/angle.h>
#include <units/angular_velocity.h>
#include <units/voltage.h>
#include <units/current.h>

class ArmIO {
public:
    enum WristPosition { WristHorizontal, WristVertical };

    struct Metrics {
        units::degree_t elbowPosition = 0_deg;
        units::degree_t elbowGoal = 0_deg;
        units::revolutions_per_minute_t elbowVelocity = 0_rpm;
        units::degree_t elbowEncPosition = 0_deg;
        units::volt_t elbowAppliedVolts = 0_V;
        units::ampere_t elbowCurrent = 0_A;

        units::degree_t wristPosition = 0_deg;
        units::degree_t wristGoal = 0_deg;
        units::revolutions_per_minute_t wristVelocity = 0_rpm;
        units::volt_t wristAppliedVolts = 0_V;
        units::ampere_t wristCurrent = 0_A;

        void Log( const std::string &key );
    };

    virtual void Update( Metrics &m ) =0;

    virtual void SetElbowGoal( units::degree_t goal ) =0;
    virtual void SetWristPosition( WristPosition pos ) =0;
    virtual void ResetWristAngle( units::degree_t position ) =0;
    virtual void SetWristOpenLoop( double percentOutput ) =0;
    virtual void UpdateElbowOffset() =0;
};
