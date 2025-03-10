
#include <frc/MathUtil.h>
#include <frc/geometry/Rotation2d.h>
#include <frc/DriverStation.h>
#include <frc2/command/Commands.h>

#include <pathplanner/lib/path/PathPlannerPath.h>
#include <pathplanner/lib/auto/AutoBuilder.h>

#include "command/DriveCommands.h"
#include "command/DriveToPose.h"

#include "swerve/Drive.h"
#include "swerve/SwerveConstants.h"

const double DEADBAND = 0.1;

frc2::CommandPtr DriveCommands::JoystickDrive( 
    Drive *d, 
    std::function<double()> xSupplier, 
    std::function<double()> ySupplier, 
    std::function<double()> omegaSupplier)
{
    return frc2::cmd::Run( [d, xSupplier, ySupplier, omegaSupplier] {
        // Apply deadband
        double x_val = xSupplier();
        double y_val = ySupplier();
        double rawlinearMagnitude = std::hypot( x_val, y_val );
        // rawlinearMagnitude = util::clamp( rawlinearMagnitude, 0.0, 1.0 );
        double linearMagnitude = frc::ApplyDeadband<double>( rawlinearMagnitude, DEADBAND );
        frc::Rotation2d linearDirection;
        if( fabs( rawlinearMagnitude ) > 0.001 ) {
            linearDirection = frc::Rotation2d{ x_val, y_val };
        }
        double omega = frc::ApplyDeadband<double>( omegaSupplier(), DEADBAND );

        // Apply expo by squaring
        linearMagnitude = linearMagnitude * linearMagnitude;
        omega = std::copysign( omega*omega, omega );

        double Vx = linearMagnitude * linearDirection.Cos();
        double Vy = linearMagnitude * linearDirection.Sin();
        
        // Convert to field relative speeds
        bool isFlipped = frc::DriverStation::GetAlliance() == frc::DriverStation::kRed;

        d->RunVelocity( 
            frc::ChassisSpeeds::FromFieldRelativeSpeeds(
                Vx * swerve::physical::kDriveSpeedLimit,
                Vy * swerve::physical::kDriveSpeedLimit,
                omega * swerve::physical::kTurnSpeedLimit,
                isFlipped ? d->GetRotation() + frc::Rotation2d(180_deg) : d->GetRotation()
            )
        ); 
    },
    {d}
    ).WithName( "Joystick Drive" );
}

frc2::CommandPtr DriveCommands::DriveToPosePP_NOTGOOD( Drive *d, std::function<frc::Pose2d()> poseFunc )
{
    return frc2::cmd::Defer( [d, poseFunc] {
        frc::Pose2d targetPose = poseFunc();

        frc::Pose2d currentPose = d->GetPose();

        if( (currentPose - targetPose).Translation().Norm() < 0.04_m ) {
            return frc2::cmd::None();
        }

        std::vector<frc::Pose2d> poses{ currentPose, targetPose };

        std::vector<pathplanner::Waypoint> waypoints = pathplanner::PathPlannerPath::waypointsFromPoses(poses);

        pathplanner::PathConstraints constraints(3.0_mps, 3.0_mps_sq, 360_deg_per_s, 720_deg_per_s_sq);

        auto path = std::make_shared<pathplanner::PathPlannerPath>(
            waypoints,
            constraints,
            std::nullopt, // The ideal starting state, this is only relevant for pre-planned paths, so can be nullopt for on-the-fly paths.
            pathplanner::GoalEndState(0.0_mps, targetPose.Rotation()) // Goal end state. You can set a holonomic rotation here.
        );

        path->preventFlipping = true;

        return pathplanner::AutoBuilder::followPath(path).WithName( "AutoBuilder-pathfindToPose");
    }, {d} );
}

frc2::CommandPtr DriveCommands::DriveOpenLoop( Drive *d, frc::ChassisSpeeds speed, bool robotRelative )
{
    return frc2::cmd::Run( [d, speed, robotRelative] {
        d->RunVelocity( 
            robotRelative ? speed : frc::ChassisSpeeds::FromFieldRelativeSpeeds( speed, d->GetRotation() )
        ); },
        {d}
    ); 
}

frc2::CommandPtr DriveCommands::DriveDeltaPose( Drive *d, frc::Transform2d move, bool robotRelative )
{
    return DriveToPoseTrap( d, [d, move, robotRelative] {
        frc::Pose2d newPose;
        frc::Pose2d currentPose = d->GetPose();

        if( robotRelative ) {
            // Move the current pose in the Robot Pose coordinate system.
            // The robot coordinate system is X-forward and Y-left.
            newPose = currentPose.TransformBy( move );
        } else {
            // Move the current pose in the field coordinate system.
            newPose = {currentPose.Translation() + move.Translation(), move.Rotation() };
        }

        return newPose;
    }
    ).WithName("DriveDeltaPose");
}

