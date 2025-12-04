

#include "arm/Arm.h"
#include "elevator/Elevator.h"
#include "climber/Climber.h"

#include "util/DataLogger.h"

#include "RobotState.h"


RobotState::RobotState( Arm *a, Elevator *e, Climber *c )
: m_arm{a}, m_elev{e}, m_climb{c}
{

}

void RobotState::UpdateLoop()
{
    frc::Pose3d statePoses[5];
    frc::Transform3d elevator_angle, stage1Transform, stage2Transform, elbowTransform, wristTransform, climberTransform;
    units::radian_t elbowAngle, wristAngle;

    elbowAngle = m_arm->GetElbowAngle();
    wristAngle = m_arm->GetWristAngle();

    frc::Translation2d elevPos = { m_elev->GetHeight(), 0_m };
    elevPos = elevPos.RotateBy( -6_deg );

    elevator_angle = { 0_m, 0_m, 0_m, { 0_deg, -6_deg, 0_deg }  };

    stage1Transform = { 0.18_m, 0_m, 0.15_m + m_elev->GetHeight()/2, { 0_deg, 0_deg, 0_deg }  };
    stage2Transform = { 0.18_m, 0_m, 0.15_m + m_elev->GetHeight(), { 0_deg, 0_deg, 0_deg }  };

    elbowTransform = { 0.175_m, 0_m, 0.32_m + m_elev->GetHeight(), { 0_deg, -elbowAngle+6_deg, 0_deg } };

    wristTransform = { 14.5_in, 0_in, 0_in, { wristAngle, 0_deg, 0_deg } };

    climberTransform = { -12.375_in, 0_in, 17_in, { 0_deg, m_climb->GetAngle(), 0_deg } };

        // The elevator first stage Pose (transform)
    statePoses[0] = frc::Pose3d{} + elevator_angle + stage1Transform;

        // The elevator second stage Pose (transform)
    statePoses[1] =  frc::Pose3d{} + elevator_angle + stage2Transform;

        // The arm Pose
    statePoses[2] = frc::Pose3d{} + elevator_angle + elbowTransform;

        // The wrist Pose
    statePoses[3] = frc::Pose3d{} + elevator_angle + elbowTransform + wristTransform;

        // The climber Pose
    statePoses[4] = frc::Pose3d{} + climberTransform;

    DataLogger::Log( "Robot/Components", std::span<frc::Pose3d>( statePoses ) );
    // DataLogger::Log( "Robot/ZeroPose", frc::Pose3d{} );
}

