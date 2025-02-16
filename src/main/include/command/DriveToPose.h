
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
    DriveToPose( Drive *drive, std::function<frc::Pose2d()> poseFunc );

    void Init() override;

    void Execute() override;

    void Ending(bool interrupted) override;

    bool IsFinished() override;

private:
    Drive *m_drive;

    std::function<frc::Pose2d()> m_poseFunc;

    frc::Pose2d m_targetPose;
    frc::Pose2d m_profileStartPose;

    // units::degree_t m_profileDirection;

    frc::ChassisSpeeds m_speeds;

    frc::TrapezoidProfile<units::meters> m_XProfile{{12_fps, 10_fps_sq}};
    frc::TrapezoidProfile<units::meters> m_YProfile{{12_fps, 10_fps_sq}};
    frc::TrapezoidProfile<units::degrees> m_RProfile{{360_deg_per_s, 900_deg_per_s_sq}};

    frc::TrapezoidProfile<units::meters>::State m_XSetpoint;
    frc::TrapezoidProfile<units::meters>::State m_YSetpoint;
    frc::TrapezoidProfile<units::degrees>::State m_RSetpoint;

    frc::TrapezoidProfile<units::meters>::State m_XGoal;
    frc::TrapezoidProfile<units::meters>::State m_YGoal;
    frc::TrapezoidProfile<units::degrees>::State m_RGoal;

    frc::PIDController m_Xpid{ 4.0, 0.0, 0.0 };
    frc::PIDController m_Ypid{ 4.0, 0.0, 0.0 };
    frc::PIDController m_Rpid{ 4.0, 0.0, 0.0 };
};
