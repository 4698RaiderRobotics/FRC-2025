#pragma once

#include <string>
#include <vector>

#include <units/angle.h>
#include <units/angular_velocity.h>
#include <units/voltage.h>
#include <units/current.h>

#include <frc/geometry/Rotation2d.h>

#include "util/Tuning.h"

struct ModuleConfigs {
    int index;
    std::string canBus = "";
    int turnCanId;
    int driveCanId;
    int encoderCanId;
    units::turn_t absoluteEncoderOffset;
    MotionConfig<units::radian> turnTune;
    MotionConfig<units::radian> driveTune;
};


class ModuleIO {
public:
    struct Inputs {
        units::radian_t drivePosition = 0_rad;
        units::radians_per_second_t driveVelocity = 0_rad_per_s;
        units::volt_t driveAppliedVolts = 0_V;
        units::ampere_t driveCurrent = 0_A;

        units::radian_t turnAbsolutePosition = 0_rad;
        units::radian_t turnPosition = 0_rad;
        units::radians_per_second_t turnVelocity = 0_rad_per_s;
        units::volt_t turnAppliedVolts = 0_V;
        units::ampere_t turnCurrent = 0_A;

        std::vector<units::second_t> odometryTimestamps;
        std::vector<units::radian_t> odometryDrivePositions;
        std::vector<units::radian_t> odometryTurnPositions;

        void LogInputs( std::string key );
    };

    virtual void UpdateInputs(Inputs &inputs) =0;

    virtual void SetDriveOpenLoop( double percent ) =0;

    virtual void SetTurnOpenLoop( double percent ) =0;

    virtual void SetDriveWheelVelocity( units::radians_per_second_t velocity ) =0;

    virtual void SetTurnPosition( units::radian_t position ) =0;

    virtual void SetDriveBrakeMode( bool enable ) =0;

    virtual void SetTurnBrakeMode( bool enable ) =0;

    virtual int GetIndex() =0;
};