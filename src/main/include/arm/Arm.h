#pragma once

#include <units/angle.h>

#include <frc2/command/SubsystemBase.h>

#include "ArmIO.h"

#include "util/Utility.h"

class Arm : public frc2::SubsystemBase {
public:
    Arm();
    void Periodic() override;

    units::degree_t GetElbowAngle();
    void SetElbowGoal( units::degree_t goal );
    void NudgeElbow( units::degree_t nudge );

    ArmIO::WristPosition GetWristGoal();
    void SetWristGoal( ArmIO::WristPosition pos );

    bool AllAtGoal();
    bool WristAtGoal();
    bool ElbowAtGoal();
    
    bool isArmBackward();
    void AdjustToHoming( bool isClimberHoming );

    frc2::CommandPtr ChangeElbowAngle( units::degree_t goal );
    frc2::CommandPtr ChangeWristPosition(ArmIO::WristPosition pos );
    frc2::CommandPtr ChangeElbowAndWrist( units::degree_t elbow_goal, ArmIO::WristPosition pos );
    frc2::CommandPtr GotoElbowRest();
    frc2::CommandPtr SetClimberHoming( bool isClimberHoming );

private:
    std::unique_ptr<ArmIO> io;
    ArmIO::Metrics metrics;

    util::MotorHomer elbowHomer;

    units::degree_t back_rest_angle;
    const units::degree_t ELBOW_GOAL_TOLERANCE = 4_deg;
    const units::degree_t WRIST_GOAL_TOLERANCE = 15_deg;
};
