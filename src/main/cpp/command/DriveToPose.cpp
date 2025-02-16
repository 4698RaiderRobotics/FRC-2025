
#include "swerve/Drive.h"

#include "command/DriveToPose.h"

#include "util/DataLogger.h"

DriveToPose::DriveToPose( Drive *drive, std::function<frc::Pose2d()> poseFunc ) 
    : m_drive{drive}, m_poseFunc{poseFunc} 
{
    SetName( "DriveToPose" );

    m_Rpid.EnableContinuousInput( -180, 180 );

    AddRequirements({m_drive});
}

void DriveToPose::Init() {
    m_targetPose = m_poseFunc();

    frc::Pose2d currentPose = m_drive->GetPose();
    frc::ChassisSpeeds currentSpeed = m_drive->GetChassisSpeeds();

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

    m_RSetpoint.position = currentPose.Rotation().Degrees();
    m_RSetpoint.velocity = currentSpeed.omega;
    m_RGoal.position = m_targetPose.Rotation().Degrees();
    m_RGoal.velocity = 0_rpm;

    // Check for 180 degree wrap around
    if( m_RSetpoint.position - m_RGoal.position > 180_deg ) {
        m_RGoal.position += 360_deg;
    } else if( m_RSetpoint.position - m_RGoal.position < -180_deg ) {
        m_RGoal.position -= 360_deg;
    }

    DataLogger::Log( "DriveToPose/StartPose", m_profileStartPose );
    DataLogger::Log( "DriveToPose/TargetPose", m_targetPose );
}

// Called repeatedly when this Command is scheduled to run
void DriveToPose::Execute() {

    frc::Pose2d currentPose = m_drive->GetPose();

    m_XSetpoint = m_XProfile.Calculate(20_ms, m_XSetpoint, m_XGoal);
    m_YSetpoint = m_YProfile.Calculate(20_ms, m_YSetpoint, m_YGoal);
    m_RSetpoint = m_RProfile.Calculate(20_ms, m_RSetpoint, m_RGoal);

    units::meter_t setptXField = m_XSetpoint.position + m_profileStartPose.Translation().X();
    units::meter_t setptYField = m_YSetpoint.position + m_profileStartPose.Translation().Y();
    units::meters_per_second_t xFF = m_XSetpoint.velocity;
	units::meters_per_second_t yFF = m_YSetpoint.velocity;

    units::meters_per_second_t xFeedback = m_Xpid.Calculate( currentPose.X().value(), setptXField.value() ) * 1_mps;
    units::meters_per_second_t yFeedback = m_Ypid.Calculate( currentPose.Y().value(), setptYField.value() ) * 1_mps;

    units::degrees_per_second_t rotFeedback = 
        m_Rpid.Calculate( currentPose.Rotation().Degrees().value(), m_RSetpoint.position.value() ) * 1_deg_per_s;

    frc::Pose2d trajectoryPose{ setptXField, setptYField, m_RSetpoint.position };

    DataLogger::Log( "DriveToPose/TrajectoryPose", trajectoryPose );
    DataLogger::Log( "DriveToPose/setptXField", setptXField );
    DataLogger::Log( "DriveToPose/setptYField", setptYField );
    DataLogger::Log( "DriveToPose/xFF", xFF );
    DataLogger::Log( "DriveToPose/yFF", yFF );
    DataLogger::Log( "DriveToPose/xFeedback", xFeedback );
    DataLogger::Log( "DriveToPose/yFeedback", yFeedback );

    m_speeds = frc::ChassisSpeeds::FromFieldRelativeSpeeds(xFF + xFeedback,
			yFF + yFeedback, m_RSetpoint.velocity + rotFeedback,
			currentPose.Rotation());

    m_drive->RunVelocity( m_speeds );
}

// Called once the command ends or is interrupted.
void DriveToPose::Ending(bool interrupted) {
    fmt::print( "   ProfiledDriveToPose::End() interrupted {}\n", interrupted );

    m_drive->Stop();
}

// Returns true when the command should end.
bool DriveToPose::IsFinished() {

    // NOTE::
    //
    //   IsFinished() returned true at the halfway point of the trajectory.
    //   Changed to use TotalTime() which goes to zero at the end of the trajectory.
    //
    units::meters_per_second_t velocity = units::math::hypot( m_speeds.vx, m_speeds.vy );
        
    bool atTargetLocation = m_XProfile.TotalTime() < 0.001_s && 
                            m_YProfile.TotalTime() < 0.001_s && 
                            m_RProfile.TotalTime() < 0.001_s &&
                            velocity < 0.1_mps &&
                            m_speeds.omega < 5_deg_per_s;
   
    return atTargetLocation;
}
