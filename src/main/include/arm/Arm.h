#pragma once

#include <units/angle.h>

#include <frc2/command/SubsystemBase.h>

#include "ArmIO.h"

#include "util/Utility.h"

class Arm : public frc2::SubsystemBase {
public:
    Arm();
    void Periodic() override;

    void SetElbowGoal( units::degree_t goal );
    void NudgeElbow( units::degree_t nudge );
    void SetWristPosition( ArmIO::WristPosition pos );
    bool AllAtGoal();
    bool WristAtGoal();
    bool ElbowAtGoal();
    bool isArmBackward();

    frc2::CommandPtr ChangeElbowAngle( units::degree_t goal );
    frc2::CommandPtr ChangeWristPosition(ArmIO::WristPosition pos );
    frc2::CommandPtr ChangeElbowAndWrist( units::degree_t elbow_goal, ArmIO::WristPosition pos );

private:
    std::unique_ptr<ArmIO> io;
    ArmIO::Metrics metrics;

    util::MotorHomer elbowHomer;

    const units::degree_t ELBOW_GOAL_TOLERANCE = 4_deg;
    const units::degree_t WRIST_GOAL_TOLERANCE = 15_deg;
};
