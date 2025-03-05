
#include <numbers>

#include "swerve/Drive.h"

#include "command/DriveToPose.h"

#include "util/DataLogger.h"


const frc::TrapezoidProfile<units::meters>::Constraints DriveToPose::XY_constraints = {6_fps, 5_fps_sq};
const frc::TrapezoidProfile<units::radians>::Constraints DriveToPose::R_constraints = {120_deg_per_s, 100_deg_per_s_sq};


DriveToPose::DriveToPose( Drive *drive, std::function<frc::Pose2d()> poseFunc, double fractionFullSpeed ) 
    : m_drive{drive}, m_poseFunc{poseFunc},
    m_XProfile{ {XY_constraints.maxVelocity * fractionFullSpeed, XY_constraints.maxAcceleration * fractionFullSpeed} },
    m_YProfile{ {XY_constraints.maxVelocity * fractionFullSpeed, XY_constraints.maxAcceleration * fractionFullSpeed} },
    m_RProfile{ {R_constraints.maxVelocity * fractionFullSpeed, R_constraints.maxAcceleration * fractionFullSpeed} }
{
    SetName( "DriveToPose" );

    m_Rpid.EnableContinuousInput( -std::numbers::pi, std::numbers::pi );

    AddRequirements({m_drive});
}

void DriveToPose::Init() 
{
    m_targetPose = m_poseFunc();

    frc::Pose2d currentPose = m_drive->GetPose();
    frc::ChassisSpeeds currentSpeed = 
        frc::ChassisSpeeds::FromRobotRelativeSpeeds( m_drive->GetChassisSpeeds(), currentPose.Rotation() );

    m_Xpid.Reset();
    m_Ypid.Reset();
    m_Rpid.Reset();

    frc::Translation2d moveTranslation = m_targetPose.Translation() - currentPose.Translation();

    m_profileStartPose = currentPose;

    m_XSetpoint.position = 0_m;
    m_XSetpoint.velocity = currentSpeed.vx;
    m_XGoal.position = moveTranslation.X();
    m_XGoal.velocity = 0_mps;

    m_YSetpoint.position = 0_m;
    m_YSetpoint.velocity = currentSpeed.vy;
    m_YGoal.position = moveTranslation.Y();
    m_YGoal.velocity = 0_mps;

    m_RSetpoint.position = currentPose.Rotation().Radians();
    m_RSetpoint.velocity = currentSpeed.omega;
    m_RGoal.position = m_targetPose.Rotation().Radians();
    m_RGoal.velocity = 0_rpm;

    // Check for 180 degree wrap around
    if( m_RSetpoint.position - m_RGoal.position > 180_deg ) {
        m_RGoal.position += 360_deg;
    } else if( m_RSetpoint.position - m_RGoal.position < -180_deg ) {
        m_RGoal.position -= 360_deg;
    }

    // m_ProfileTime = 0_s;
    // m_XProfile.Calculate(20_ms, m_XSetpoint, m_XGoal);
    // m_YProfile.Calculate(20_ms, m_YSetpoint, m_YGoal);
    // m_RProfile.Calculate(20_ms, m_RSetpoint, m_RGoal);
    // fmt::print( "XProfile Total Time = {}\n", m_XProfile.TimeLeftUntil( m_XGoal.position ) );
    // fmt::print( "YProfile Total Time = {}\n", m_YProfile.TimeLeftUntil( m_YGoal.position ) );
    // fmt::print( "RProfile Total Time = {}\n", m_RProfile.TimeLeftUntil( m_RGoal.position ) );

    DataLogger::Log( "DriveToPose/StartPose", m_profileStartPose );
    DataLogger::Log( "DriveToPose/TargetPose", m_targetPose );
}

// Called repeatedly when this Command is scheduled to run
void DriveToPose::Execute() 
{
    frc::Pose2d currentPose = m_drive->GetPose();

    m_XSetpoint = m_XProfile.Calculate(20_ms, m_XSetpoint, m_XGoal);
    m_YSetpoint = m_YProfile.Calculate(20_ms, m_YSetpoint, m_YGoal);
    m_RSetpoint = m_RProfile.Calculate(20_ms, m_RSetpoint, m_RGoal);

    units::meter_t setptXField = m_XSetpoint.position + m_profileStartPose.Translation().X();
    units::meter_t setptYField = m_YSetpoint.position + m_profileStartPose.Translation().Y();

    units::meters_per_second_t xFeedback = m_Xpid.Calculate( currentPose.X().value(), setptXField.value() ) * 1_mps;
    units::meters_per_second_t yFeedback = m_Ypid.Calculate( currentPose.Y().value(), setptYField.value() ) * 1_mps;

    units::radians_per_second_t rotFeedback = 
        m_Rpid.Calculate( currentPose.Rotation().Radians().value(), m_RSetpoint.position.value() ) * 1_rad_per_s;

    frc::Pose2d trajectoryPose{ setptXField, setptYField, m_RSetpoint.position };
    m_speeds = frc::ChassisSpeeds::FromFieldRelativeSpeeds(
        m_XSetpoint.velocity + xFeedback,
		m_YSetpoint.velocity + yFeedback, 
        m_RSetpoint.velocity + rotFeedback,
		currentPose.Rotation()
    );

    m_drive->RunVelocity( m_speeds );

    DataLogger::Log( "DriveToPose/TrajectoryPose", trajectoryPose );

    DataLogger::Log( "DriveToPose/setptXField", setptXField );
    DataLogger::Log( "DriveToPose/setptYField", setptYField );
    DataLogger::Log( "DriveToPose/setptRot", m_RSetpoint.position );
    DataLogger::Log( "DriveToPose/xFF", m_XSetpoint.velocity );
    DataLogger::Log( "DriveToPose/yFF", m_YSetpoint.velocity );
    DataLogger::Log( "DriveToPose/rFF", m_RSetpoint.velocity );
    DataLogger::Log( "DriveToPose/xFeedback", xFeedback );
    DataLogger::Log( "DriveToPose/yFeedback", yFeedback );
    DataLogger::Log( "DriveToPose/rFeedback", rotFeedback );
    DataLogger::Log( "DriveToPose/xTotalTime", m_XProfile.TotalTime() );
    DataLogger::Log( "DriveToPose/yTotalTime", m_YProfile.TotalTime() );
    DataLogger::Log( "DriveToPose/rTotalTime", m_RProfile.TotalTime() );
    DataLogger::Log( "DriveToPose/velocity", units::math::hypot( m_speeds.vx, m_speeds.vy ) );
    DataLogger::Log( "DriveToPose/omega", m_speeds.omega );
}

// Called once the command ends or is interrupted.
void DriveToPose::Ending(bool interrupted) 
{
    m_drive->Stop();
    DataLogger::Log( "DriveToPose/finalError", m_targetPose - m_drive->GetPose() );
}

// Returns true when the command should end.
bool DriveToPose::IsFinished() 
{
    // NOTE::
    //
    //   IsFinished() returned true at the halfway point of the trajectory.
    //   Changed to use TotalTime() which goes to zero at the end of the trajectory.
    //

    bool atTargetLocation = m_XProfile.TotalTime() < 0.001_s && 
                            m_YProfile.TotalTime() < 0.001_s && 
                            m_RProfile.TotalTime() < 0.001_s;
   
    return atTargetLocation;
}
