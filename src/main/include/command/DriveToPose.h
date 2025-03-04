
#pragma once

#include <functional>

#include <units/velocity.h>
#include <units/acceleration.h>
#include <units/angular_velocity.h>
#include <units/angular_acceleration.h>

#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>

#include <frc/controller/PIDController.h>
#include <frc/trajectory/TrapezoidProfile.h>
#include <frc/kinematics/ChassisSpeeds.h>

#include "util/LoggedCommand.h"

class Drive;

/**
 * Drive the Robot to a Field pose.
 * 
 * Use trapezoidal profiles for each of the x-axis, y-axis, and rotation.
 */
class DriveToPose : public frc2::CommandHelper<LoggedCommand, DriveToPose> {
public:
    DriveToPose( Drive *drive, std::function<frc::Pose2d()> poseFunc, double fractionFullSpeed=1.0 );
    
    void Init() override;

    void Execute() override;

    void Ending(bool interrupted) override;

    bool IsFinished() override;

private:
    Drive *m_drive;

    std::function<frc::Pose2d()> m_poseFunc;

    frc::Pose2d m_targetPose;
    frc::Pose2d m_profileStartPose;

    frc::ChassisSpeeds m_speeds;

    units::second_t m_ProfileTime{0_s};

    frc::TrapezoidProfile<units::meters> m_XProfile;
    frc::TrapezoidProfile<units::meters> m_YProfile;
    frc::TrapezoidProfile<units::radians> m_RProfile;

    frc::TrapezoidProfile<units::meters>::State m_XSetpoint;
    frc::TrapezoidProfile<units::meters>::State m_YSetpoint;
    frc::TrapezoidProfile<units::radians>::State m_RSetpoint;

    frc::TrapezoidProfile<units::meters>::State m_XGoal;
    frc::TrapezoidProfile<units::meters>::State m_YGoal;
    frc::TrapezoidProfile<units::radians>::State m_RGoal;

    frc::PIDController m_Xpid{ 4.0, 0.0, 0.0 };
    frc::PIDController m_Ypid{ 4.0, 0.0, 0.0 };
    frc::PIDController m_Rpid{ 4.0, 0.0, 0.0 };

    static const frc::TrapezoidProfile<units::meters>::Constraints XY_constraints;
    static const frc::TrapezoidProfile<units::radians>::Constraints R_constraints;
};
