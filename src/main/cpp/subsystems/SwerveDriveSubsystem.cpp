// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include <units/math.h>

#include "subsystems/SwerveDriveSubsystem.h"
#include "SwerveConstants.h"
#include "DataLogger.h"

#include <frc/DriverStation.h>
#include <frc/Filesystem.h>
#include <frc/trajectory/TrajectoryUtil.h>

#include <frc/smartdashboard/SmartDashboard.h>

#include <pathplanner/lib/auto/AutoBuilder.h>
#include <pathplanner/lib/util/HolonomicPathFollowerConfig.h>
#include <pathplanner/lib/util/PIDConstants.h>
#include <pathplanner/lib/util/ReplanningConfig.h>

SwerveDriveSubsystem::SwerveDriveSubsystem() 
    : m_modules{ SwerveModule{ swerve::deviceIDs::kFrontLeftTurnMotorID, swerve::deviceIDs::kFrontLeftDriveMotorID, 
                               swerve::deviceIDs::kFrontLeftAbsoluteEncoderID, swerve::physical::kFrontLeftAbsoluteOffset },
                 SwerveModule{ swerve::deviceIDs::kFrontRightTurnMotorID, swerve::deviceIDs::kFrontRightDriveMotorID, 
                               swerve::deviceIDs::kFrontRightAbsoluteEncoderID, swerve::physical::kFrontRightAbsoluteOffset },
                 SwerveModule{ swerve::deviceIDs::kBackLeftTurnMotorID, swerve::deviceIDs::kBackLeftDriveMotorID, 
                               swerve::deviceIDs::kBackLeftAbsoluteEncoderID, swerve::physical::kBackLeftAbsoluteOffset },
                 SwerveModule{ swerve::deviceIDs::kBackRightTurnMotorID, swerve::deviceIDs::kBackRightDriveMotorID, 
                               swerve::deviceIDs::kBackRightAbsoluteEncoderID, swerve::physical::kBackRightAbsoluteOffset } }
    , m_gyro{swerve::deviceIDs::kPigeonIMUID, ""} 
    , m_kinematics{ frc::Translation2d{+( swerve::physical::kDriveBaseLength / 2 ), +( swerve::physical::kDriveBaseWidth / 2 )},
                    frc::Translation2d{+( swerve::physical::kDriveBaseLength / 2 ), -( swerve::physical::kDriveBaseWidth / 2 )},
                    frc::Translation2d{-( swerve::physical::kDriveBaseLength / 2 ), +( swerve::physical::kDriveBaseWidth / 2 )},
                    frc::Translation2d{-( swerve::physical::kDriveBaseLength / 2 ), -( swerve::physical::kDriveBaseWidth / 2 )} }
    , m_odometry{ m_kinematics, frc::Rotation2d{ 0_deg },
                    { m_modules[0].GetPosition(), 
                      m_modules[1].GetPosition(),
                      m_modules[2].GetPosition(), 
                      m_modules[3].GetPosition()
                    }, frc::Pose2d{ 0_ft, 0_ft, 0_deg } }
    , m_controller{ frc::PIDController{ swerve::pidf::X_Holo_kP, swerve::pidf::X_Holo_kI, swerve::pidf::X_Holo_kD }, 
                    frc::PIDController{ swerve::pidf::Y_Holo_kP, swerve::pidf::Y_Holo_kI, swerve::pidf::Y_Holo_kD },
                    frc::ProfiledPIDController<units::radian> {
                        swerve::pidf::Th_Holo_kP, swerve::pidf::Th_Holo_kI, swerve::pidf::Th_Holo_kD, 
                        frc::TrapezoidProfile<units::radian>::Constraints{
                            swerve::pidf::Th_Holo_MaxVel, swerve::pidf::Th_Holo_MaxAcc }}}

{
        // Reset the gyro
    ResetGyro(0_deg);

    frc::SmartDashboard::PutData("Field", &m_field);

    const units::meter_t kDriveBaseRadius = units::math::sqrt(
        (swerve::physical::kDriveBaseWidth)*(swerve::physical::kDriveBaseWidth) + 
        (swerve::physical::kDriveBaseLength)*(swerve::physical::kDriveBaseLength) 
    ) / 2.0;

    pathplanner::AutoBuilder::configureHolonomic(
        [this](){ return GetPose(); },
        [this](frc::Pose2d pose){ ResetGyro(pose.Rotation().Degrees()); ResetPose(pose); },
        [this](){frc::ChassisSpeeds s = GetRobotRelativeSpeeds(); return s;},
        [this](frc::ChassisSpeeds speeds){ Drive(speeds, false); },
        pathplanner::HolonomicPathFollowerConfig( // HolonomicPathFollowerConfig, this should likely live in your Constants class
            pathplanner::PIDConstants(4.0, 0.0, 0.0), // Translation PID constants
            pathplanner::PIDConstants(4.0, 0.0, 0.0), // Rotation PID constants
            swerve::physical::kMaxDriveSpeed, // Max module speed
            kDriveBaseRadius, // Drive base radius. Distance from robot center to furthest module.
            pathplanner::ReplanningConfig() // Default path replanning config. See the API for the options here
        ),
        []() {
            // Boolean supplier that controls when the path will be mirrored for the red alliance
            // This will flip the path being followed to the red side of the field.
            // THE ORIGIN WILL REMAIN ON THE BLUE SIDE
            return frc::DriverStation::GetAlliance() == frc::DriverStation::Alliance::kRed;
            // auto alliance = frc::DriverStation::GetAlliance();
            // if (alliance.has_value()) {
            //     return alliance.value() == frc::DriverStation::Alliance::kRed;
            // }
            //     return false;
        },
        this
    );
}

// ArcadeDrive drives with joystick inputs
// This takes -1 to 1 inputs
void SwerveDriveSubsystem::ArcadeDrive( double xPercent, double yPercent, double omegaPercent, bool operatorRelative ) {
    auto x = xPercent * swerve::physical::kDriveSpeedLimit;
    auto y = yPercent * swerve::physical::kDriveSpeedLimit;
    auto omega = omegaPercent * swerve::physical::kTurnSpeedLimit;

    frc::ChassisSpeeds speeds{ x, y, omega };

    if(operatorRelative) {
        speeds = speeds.FromFieldRelativeSpeeds( speeds.vx, speeds.vy, speeds.omega, m_gyro.GetYaw().GetValue() + driver_offset);
    }

    Drive( speeds, false );
}

void SwerveDriveSubsystem::Drive( frc::ChassisSpeeds speeds, bool fieldRelative ) {
    if(fieldRelative) {
        m_robotRelativeSpeeds = speeds.FromFieldRelativeSpeeds(speeds.vx, speeds.vy, speeds.omega, m_gyro.GetYaw().GetValue() + field_offset );
        // DataLogger::GetInstance().Log( "Swerve/Drive/Field Rel X", speeds.vx.value() );
        // DataLogger::GetInstance().Log( "Swerve/Drive/Field Rel Y", speeds.vy.value() );
        // DataLogger::GetInstance().Log( "Swerve/Drive/Field Rel Omega", static_cast<units::revolutions_per_minute_t>(speeds.omega).value() );
        // DataLogger::GetInstance().Log( "Swerve/Drive/Gyro Angle", m_gyro.GetYaw().GetValue().value() );
    } else {
        m_robotRelativeSpeeds = speeds;
    }

    // DataLogger::GetInstance().Log( "Swerve/Drive/Robot Rel X", m_robotRelativeSpeeds.vx.value() );
    // DataLogger::GetInstance().Log( "Swerve/Drive/Robot Rel Y", m_robotRelativeSpeeds.vy.value() );
    // DataLogger::GetInstance().Log( "Swerve/Drive/Robot Rel Omega", static_cast<units::revolutions_per_minute_t>(m_robotRelativeSpeeds.omega).value() );


    // An array of SwerveModuleStates computed from the ChassisSpeeds object
    // m_desiredStates = m_kinematics.ToSwerveModuleStates( fieldRelative ? speeds.FromFieldRelativeSpeeds( 
    //                 speeds.vx, speeds.vy, speeds.omega, m_gyro.GetYaw().GetValue() ) :
    //                 speeds );
    m_desiredStates = m_kinematics.ToSwerveModuleStates( m_robotRelativeSpeeds );
    m_kinematics.DesaturateWheelSpeeds( &m_desiredStates, swerve::physical::kMaxDriveSpeed );
}

// Drives a path given a trajectory state
void SwerveDriveSubsystem::DriveTrajectory( frc::Trajectory::State trajState, const frc::Rotation2d &robotHeading ) {
    // A ChassisSpeeds objects based on the current position on the trajectory
    auto adjustedSpeeds = m_controller.Calculate( m_odometry.GetEstimatedPosition(), trajState, robotHeading.Degrees() );

    Drive( adjustedSpeeds, false );
}

void SwerveDriveSubsystem::Periodic( void ) {

    // Sets each SwerveModule to the correct SwerveModuleState
    for( int i=0; i<4; ++i ) {
        m_modules[i].SetDesiredState( m_desiredStates[i] );
    }

    if(frc::DriverStation::IsDisabled() && !m_have_driver_offset ) {
        auto pose = m_odometry.GetEstimatedPosition();
        field_offset = pose.Rotation().Degrees();
        if(frc::DriverStation::GetAlliance() == frc::DriverStation::Alliance::kRed) {
            driver_offset = field_offset + 180_deg;
        } else {
            driver_offset = field_offset;
        }
    } else if( !frc::DriverStation::IsDisabled() && !m_have_driver_offset ) {
            // Only get a driver offset on the first enabling..
        m_have_driver_offset = true;
         fmt::print( "SwerveDriveSubsystem::Periodic -- Stop getting offset has {} = {:.5}\n", m_have_driver_offset, driver_offset.value() );
    }

    // Updates the odometry of the robot given the SwerveModules' states
    //needs to be an array

    DataLogger::Log( "Swerve/Gyro Angle", m_gyro.GetYaw().GetValueAsDouble(), true );
    DataLogger::Log( "Swerve/Driver Offset", driver_offset.value(), true );
    DataLogger::Log( "Swerve/Field Offset", field_offset.value(), true );

    m_odometry.Update( m_gyro.GetYaw().GetValue(),
    {
         m_modules[0].GetPosition(),  m_modules[1].GetPosition(), 
         m_modules[2].GetPosition(),  m_modules[3].GetPosition() 
    });

    //m_vision->UpdateVisionPose( m_odometry );

    if( frc::DriverStation::IsEnabled() ) {
        // Log the swerve states
        for( int i=0; i<4; ++i ) {
            m_actualStates[i] = m_modules[i].GetState();
        }
        DataLogger::Log( "Swerve/Actual States", m_actualStates );
        DataLogger::Log( "Swerve/Desired States", m_desiredStates );

        // Log each module's info
        for(int i = 0; i < 4; i++) {
            DataLogger::Log( m_modules[i].m_name + "/Turn Setpoint", m_modules[i].state.angle.Degrees().value() );
            DataLogger::Log( m_modules[i].m_name + "/Turn Position", 
                    units::degree_t{m_modules[i].m_turnAbsEncoder.GetPosition().GetValueAsDouble()  * 1_tr}.value() );
            DataLogger::Log( m_modules[i].m_name + "/Turn Raw Position", m_modules[i].m_turnAbsEncoder.GetAbsolutePosition().GetValueAsDouble() );
            DataLogger::Log( m_modules[i].m_name + "/Turn pidoutput", m_modules[i].pidOutput );

            DataLogger::Log( m_modules[i].m_name + "/Delta Theta", m_modules[i].dTheta.value() );
            DataLogger::Log( m_modules[i].m_name + "/Desired RPM", static_cast<units::revolutions_per_minute_t>(m_modules[i].speed).value() );
            DataLogger::Log( m_modules[i].m_name + "/Optimized RPM", static_cast<units::revolutions_per_minute_t>(m_modules[i].opSpeed).value() );
            DataLogger::Log( m_modules[i].m_name + "/Drive Current", m_modules[i].m_driveMotor.GetSupplyCurrent().GetValueAsDouble() );
            DataLogger::Log( m_modules[i].m_name + "/Turn Current", m_modules[i].m_turnMotor.GetSupplyCurrent().GetValueAsDouble() );
            DataLogger::Log( m_modules[i].m_name + "/Drive Position (m)",
                    (units::turn_t{m_modules[i].m_driveMotor.GetPosition().GetValueAsDouble()} * swerve::physical::kDriveMetersPerRotation).value() );
            DataLogger::Log( m_modules[i].m_name + "/Drive Motor RPM", m_modules[i].m_driveMotor.GetVelocity().GetValueAsDouble() * 60 );
            DataLogger::Log( m_modules[i].m_name + "/Turn Motor RPM", m_modules[i].m_turnMotor.GetVelocity().GetValueAsDouble() * 60 );
        }
    }

    m_field.SetRobotPose( m_odometry.GetEstimatedPosition() );
}

frc::ChassisSpeeds SwerveDriveSubsystem::GetRobotRelativeSpeeds() {
    return m_robotRelativeSpeeds;
}

// Returns the pose2d of the robot
frc::Pose2d SwerveDriveSubsystem::GetPose( void ) {
    return m_odometry.GetEstimatedPosition();
}

// Resets the gyro to an angle
void SwerveDriveSubsystem::ResetGyro( units::degree_t angle ) {
    DataLogger::Log( "Swerve/Status", fmt::format("Reseting Gyro to {}..", angle ) );
    driver_offset -= angle;
    field_offset -= angle;
    m_gyro.SetYaw(angle);
}

// Resets the gyro to an angle
void SwerveDriveSubsystem::ResetDriverOrientation( units::degree_t angle ) {
    DataLogger::Log( "Swerve/Status", fmt::format("Reseting Driver Orientation to {}..", angle ) );
    driver_offset = 0_deg;
    ResetGyro(angle);
}

// Resets the pose to a position
void SwerveDriveSubsystem::ResetPose( frc::Pose2d pose ) {
    DataLogger::Log( "Swerve/Status", fmt::format("Reseting Pose to <{},{},{}> with GyroYaw {}..", 
                    pose.X(), pose.Y(), pose.Rotation().Degrees(), m_gyro.GetYaw().GetValue() ) );
    m_odometry.ResetPosition(
        m_gyro.GetYaw().GetValue(),
        {
            m_modules[0].GetPosition(),  m_modules[1].GetPosition(), 
            m_modules[2].GetPosition(),  m_modules[3].GetPosition() 
        },
        pose
    );
}

