
#include "Robot.h"

#include "command/CoralViz.h"

#include "swerve/Drive.h"

CoralViz::CoralViz( std::function<frc::Pose2d()> poseFunc, std::function<bool()> hasCoralFunc ) 
    : poseFunc{poseFunc}, hasCoralFunc{hasCoralFunc}
{
    SetName( "CoralViz" );

  // This command has no requirements
  //  AddRequirements();
}

void CoralViz::Init()
{
    ++coral_count;
    coralPoses.push_back( frc::Pose3d{} );
}

void CoralViz::Execute() {
    frc::Transform3d elevTransform, elbowTransform, wristTransform, climberTransform;
    units::radian_t elbowAngle, wristAngle;

    elbowAngle =  96_deg + elbow_lig->GetAngle() * 1_deg;
    wristAngle = asin( (39 * wrist_lig2->GetLength() - 2) / 6 ) * 1_rad;

    frc::Translation2d elevPos = { elevator_lig->GetLength() * 1_m, 0_m };
    elevPos = elevPos.RotateBy( 6_deg );
    elevTransform = { 8_in - elevPos.Y(), 0_m, elevPos.X() + 5.0_in, { 0_deg, 0_deg, 90_deg }  };

        // Rotate in the coral Y-Z plane
    frc::Translation2d elbowPos = { 17_in, 0_in };
    elbowPos = elbowPos.RotateBy( elbowAngle );
    elbowTransform = { 0_in, -elbowPos.X(), elbowPos.Y(), { -elbowAngle, 0_deg, 0_deg } };

        // Rotate in the coral X-Z plane
    wristTransform = { 0_in, 0_in, 0_in, { 0_deg, wristAngle, 0_deg } };

    coralPoses[coral_count-1] = frc::Pose3d{poseFunc()} + elevTransform + elbowTransform + wristTransform;

    DataLogger::Log( "CoralViz/coralPoses", coralPoses );
}

bool CoralViz::IsFinished() {
    return !hasCoralFunc();
}
