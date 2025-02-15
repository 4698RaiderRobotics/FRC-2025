//
//      Base Class for a logged robot to write basic data to a log file.
//

#include <frc/RobotController.h>
#include <frc/DataLogManager.h>
#include <frc/DriverStation.h>
#include <frc/livewindow/LiveWindow.h>
#include <frc2/command/CommandScheduler.h>
#include <frc2/command/Command.h>

#include "util/DataLogger.h"
#include "util/LoggedRobot.h"

LoggedRobot::LoggedRobot() {
        // Disable LiveWindow Telemetry
    frc::LiveWindow::DisableAllTelemetry();
    
        // Start the log manager
    frc::DataLogManager::Start();

        // Record both DS control and joystick data
    frc::DriverStation::StartDataLog( frc::DataLogManager::GetLog() );

        // Send the metadata from the buildinfo.txt file
    DataLogger::LogMetadata();

        // Determine the number of PDP channels
    m_pdpChannels = (m_pdp.GetType() == frc::PowerDistribution::ModuleType::kRev) ? 24 : 16;

    frc2::CommandScheduler::GetInstance().OnCommandInitialize(  
        [](const frc2::Command& command) {
            DataLogger::Log( "Command/Log", "Command " + command.GetName() + 
                                        " starting..." );
            DataLogger::Log( "Command/" + command.GetName(), true );
        }
        );
    frc2::CommandScheduler::GetInstance().OnCommandFinish(  
        [](const frc2::Command& command) {
            DataLogger::Log( "Command/Log", "Command " + command.GetName() + 
                                        " finished." );
            DataLogger::Log( "Command/" + command.GetName(), false );
        }
    );
    frc2::CommandScheduler::GetInstance().OnCommandInterrupt(  
        [](const frc2::Command& command, const std::optional<frc2::Command*>& int_cmd) {
            DataLogger::Log( "Command/" + command.GetName(), false );
            DataLogger::Log( "Command/Log", "Command <" + command.GetName() + 
                   "> interrupted by <" + (int_cmd.has_value() ? int_cmd.value()->GetName() : "<DISABLED>") + ">" );
        }
    );
}

void LoggedRobot::RobotPeriodic() {
    static double currents[24];
    frc::PowerDistribution::Faults faults;

    frc::CANStatus cs;

        // Log the RoboRIO Information
    DataLogger::Log( "RoboRIO/Input Voltage", frc::RobotController::GetInputVoltage() );
    DataLogger::Log( "RoboRIO/Input Current", frc::RobotController::GetInputCurrent() );
    DataLogger::Log( "RoboRIO/BrownedOut", frc::RobotController::IsBrownedOut() );
    DataLogger::Log( "RoboRIO/3V3 Volts", frc::RobotController::GetVoltage3V3() );
    DataLogger::Log( "RoboRIO/3V3 Amps", frc::RobotController::GetCurrent3V3() );
    DataLogger::Log( "RoboRIO/3V3 Fault Count", int64_t(frc::RobotController::GetFaultCount3V3()) );
    DataLogger::Log( "RoboRIO/5V Volts", frc::RobotController::GetVoltage5V() );
    DataLogger::Log( "RoboRIO/5V Amps", frc::RobotController::GetCurrent5V() );
    DataLogger::Log( "RoboRIO/5V Fault Count", int64_t(frc::RobotController::GetFaultCount5V()) );

        // Log the RoboRIO CANBus Stats
    cs = frc::RobotController::GetCANStatus();
    DataLogger::Log( "RoboRIO/CAN Percent Utilization", cs.percentBusUtilization );
    DataLogger::Log( "RoboRIO/CAN OffCount", int64_t(cs.busOffCount) );
    DataLogger::Log( "RoboRIO/CAN receiveErrorCount", int64_t(cs.receiveErrorCount) );
    DataLogger::Log( "RoboRIO/CAN transmitErrorCount", int64_t(cs.transmitErrorCount) );
    DataLogger::Log( "RoboRIO/CAN txFullCount", int64_t(cs.txFullCount) );

        // Log the PDP Information
    faults = m_pdp.GetFaults();

     DataLogger::Log( "PDP/Bus Voltage", m_pdp.GetVoltage() );
     DataLogger::Log( "PDP/Total Current", m_pdp.GetTotalCurrent() );
     DataLogger::Log( "PDP/Temperature", m_pdp.GetTemperature() );
     DataLogger::Log( "PDP/Total Power", m_pdp.GetTotalPower() );
     DataLogger::Log( "PDP/Total Energy", m_pdp.GetTotalEnergy() );
     DataLogger::Log( "PDP/Brown Out", (bool) faults.Brownout );
     DataLogger::Log( "PDP/Can Warning", (bool) faults.CanWarning );
     DataLogger::Log( "PDP/Hardware Fault", (bool) faults.HardwareFault );

    for( int i=0; i<m_pdpChannels; ++i ) {
        currents[i] = m_pdp.GetCurrent( i );
    }
    DataLogger::Log( "PDP/Currents", std::span<double> ( currents, m_pdpChannels) );
}
