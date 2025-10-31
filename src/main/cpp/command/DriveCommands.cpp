
#include <frc/MathUtil.h>
#include <frc/geometry/Rotation2d.h>
#include <frc/DriverStation.h>
#include <frc2/command/Commands.h>

#include <pathplanner/lib/path/PathPlannerPath.h>
#include <pathplanner/lib/auto/AutoBuilder.h>

#include "command/DriveCommands.h"
#include "command/DriveToPose.h"
#include "command/ReefCommands.h"

#include "swerve/Drive.h"
#include "swerve/SwerveConstants.h"

#include "util/DataLogger.h"

const double DEADBAND = 0.1;

units::revolutions_per_minute_t DriveCommands::currentTurnSpeedLimit = swerve::physical::kTurnSpeedLimit;
units::meters_per_second_t DriveCommands::currentDriveSpeedLimit = swerve::physical::kDriveSpeedLimit;

void DriveCommands::SetDriveSpeedFunc( bool useSlowSpeed )
{
    if( useSlowSpeed ) {
        currentTurnSpeedLimit = swerve::physical::kTurnSpeedLimit * 0.15;
        currentDriveSpeedLimit = swerve::physical::kDriveSpeedLimit * 0.15;
    } else {
        currentTurnSpeedLimit = swerve::physical::kTurnSpeedLimit;
        currentDriveSpeedLimit = swerve::physical::kDriveSpeedLimit;
    }
}

frc2::CommandPtr DriveCommands::SetDriveSpeed( bool useSlowSpeed )
{
    return frc2::cmd::RunOnce( [useSlowSpeed] {
        if( useSlowSpeed ) {
            currentTurnSpeedLimit = swerve::physical::kTurnSpeedLimit * 0.15;
            currentDriveSpeedLimit = swerve::physical::kDriveSpeedLimit * 0.15;
        } else {
            currentTurnSpeedLimit = swerve::physical::kTurnSpeedLimit;
            currentDriveSpeedLimit = swerve::physical::kDriveSpeedLimit;
        }
    });
}


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

        // Apply expo 
        double m_expo = 0.75;
        linearMagnitude = m_expo * std::pow(linearMagnitude, 3.0) + (1.0 - m_expo) * linearMagnitude;

        // Use 9th degree expo...
        double omega_expo = 0.68;
        double omega_ex = omega_expo * std::pow( omega, 9.0 ) + (1.0 - omega_expo) * omega;
        // linearMagnitude = linearMagnitude * linearMagnitude;
        omega = std::copysign( omega_ex, omega );

        double Vx = linearMagnitude * linearDirection.Cos();
        double Vy = linearMagnitude * linearDirection.Sin();
        
        // Convert to field relative speeds
        bool isFlipped = frc::DriverStation::GetAlliance() == frc::DriverStation::kRed;

        d->RunVelocity( 
            frc::ChassisSpeeds::FromFieldRelativeSpeeds(
                Vx * currentDriveSpeedLimit,
                Vy * currentDriveSpeedLimit,
                omega * currentTurnSpeedLimit,
                isFlipped ? d->GetRotation() + frc::Rotation2d(180_deg) : d->GetRotation()
            )
        ); 
    },
    {d}
    ).WithName( "Joystick Drive" );
}

frc2::CommandPtr DriveCommands::DriveToPosePP( Drive *d, std::function<std::vector<frc::Pose2d>()> poseFunc, std::function<ReefPlacement ()> place_func )
{
    return frc2::cmd::Defer( [d, poseFunc, place_func] {
        std::vector<frc::Pose2d> targetPoses = poseFunc();

        frc::Pose2d currentPose = d->GetPose();
        frc::ChassisSpeeds speed = d->GetFieldRelativeChassisSpeeds();
        frc::Pose2d lookAheadPose = currentPose.Exp( speed.ToTwist2d( 100_ms ) );
        double vel_mag = std::hypot( speed.vx.value(), speed.vy.value() );

            // If we are really close to the goal point then do nothing
        units::inch_t dist_to_end_goal = (lookAheadPose - targetPoses[0]).Translation().Norm();
        DataLogger::Log( "DriveToPosePP/dist_to_end_goal", dist_to_end_goal );
        if( dist_to_end_goal < 1.5_in ) {
            return frc2::cmd::None();
        }

            // Either start towards the mid-point or in the direction the
            // robot is moving depending on current speed
        frc::Rotation2d direction = (targetPoses[1].Translation() - lookAheadPose.Translation()).Angle(); 
        if( fabs( vel_mag ) > 0.25 ) {
            direction = frc::Rotation2d{ speed.vx.value(), speed.vy.value() };
        }
        lookAheadPose = { lookAheadPose.Translation(), direction };

        std::vector<frc::Pose2d> poses;
        std::vector<pathplanner::RotationTarget> rotTarget = { { 0.5, targetPoses[0].Rotation() } };
            // If we are really close to the mid-point then omit the mid-point.
        units::inch_t dist_to_mid_pt = (lookAheadPose - targetPoses[1]).Translation().Norm();
        if( dist_to_mid_pt < 1.5_in ) {
            poses = { lookAheadPose, targetPoses[0] };
        } else {
            poses = { lookAheadPose, targetPoses[1], targetPoses[0] };
        }

        units::inch_t y_offset_dist = units::math::abs((lookAheadPose - targetPoses[0]).Translation().Y());

        DataLogger::Log( "DriveToPosePP/poses", poses );
        DataLogger::Log( "DriveToPosePP/start_speed", vel_mag );
        DataLogger::Log( "DriveToPosePP/dist_to_mid_pt", dist_to_mid_pt );
        DataLogger::Log( "DriveToPosePP/y_offset_dist", y_offset_dist );

        std::vector<pathplanner::Waypoint> waypoints = pathplanner::PathPlannerPath::waypointsFromPoses(poses);

        // Determine our constraints based on how far to the side we are and
        // what level we are placing on
        double fraction_full_speed = 1.0;
        double linear_accel_factor = 1.0;
        switch( place_func() ) {
        case ReefPlacement::PLACING_L1:
            // Keep at 1.0
            break;
        case ReefPlacement::PLACING_L2:
            if( y_offset_dist > 12_in ) {
                linear_accel_factor = 0.8;
            }
            break;
        case ReefPlacement::PLACING_L3:
            fraction_full_speed = 0.75;
            if( y_offset_dist > 12_in ) {
                linear_accel_factor = 0.8;
            }
            break;
        case ReefPlacement::PLACING_L4:
            fraction_full_speed = 0.75;
            if( y_offset_dist > 12_in ) {
                fraction_full_speed = 0.65;
                linear_accel_factor = 0.6;
            }
            break;
        default:
            break;
        }

        DataLogger::Log( "DriveToPosePP/fraction_full_speed", fraction_full_speed );
        DataLogger::Log( "DriveToPosePP/linear_accel_factor", linear_accel_factor );

        pathplanner::PathConstraints constraints(
            3.0_mps * fraction_full_speed, 
            3.0_mps_sq * fraction_full_speed * linear_accel_factor, 
            360_deg_per_s * fraction_full_speed, 
            720_deg_per_s_sq * fraction_full_speed
        );

        try {
            auto path = std::make_shared<pathplanner::PathPlannerPath>(
                waypoints, 
                rotTarget,
			    std::vector<pathplanner::PointTowardsZone>(), 
                std::vector<pathplanner::ConstraintsZone>(),
			    std::vector<pathplanner::EventMarker>(),
                constraints,
                pathplanner::IdealStartingState( vel_mag * 1_mps, lookAheadPose.Rotation() ),
                // std::nullopt,
                pathplanner::GoalEndState(0.0_mps, targetPoses[0].Rotation()), // Goal end state. You can set a holonomic rotation here.
                false // reversed?
            );

            path->preventFlipping = true;
            DataLogger::Log( "DriveToPosePP/path_gen_ok", true );
            return pathplanner::AutoBuilder::followPath(path).WithName( "AutoBuilder-pathfindToPose");
        } catch( const std::exception& ) {
            // fmt::print( "\n\n================> DriveToPosePP -- PathPlanner path creation failed <==================\n" );
            DataLogger::Log( "DriveToPosePP/path_gen_ok", false );
            return frc2::cmd::None();
        }

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

frc2::CommandPtr DriveCommands::DriveDeltaPose( Drive *d, frc::Transform2d move, bool robotRelative, double fractionFullSpeed )
{
    return DriveToPoseTrap( d, [d, move, robotRelative, fractionFullSpeed] {
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
    },
    fractionFullSpeed
    ).WithName("DriveDeltaPose");
}

