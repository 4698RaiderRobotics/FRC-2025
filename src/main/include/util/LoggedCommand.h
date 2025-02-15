// Log data to the Data Log file on the RoboRIO
//

#pragma once

#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>

#include "DataLogger.h"

// LoggedCommand is derived from frc2::Command to provide automatic logging of the start
// and end of a command.  Use LoggedCommand in place of frc2::Command to derive a new
// command.  You should use SetName() in the constructor of your command.  Use Init()
// and Ending() in place of Initialize() and End(). 
class LoggedCommand : public frc2::Command {
public:
  

  void Initialize() final {
    DataLogger::Log( "Command/" + this->GetName(), true );
    DataLogger::Log( "Command/Log", "   Command " +  this->GetName() + " initialized" );
    Init();
  }
  void End( bool interrupted ) final {
    Ending( interrupted );
    DataLogger::Log( "Command/" + this->GetName(), false );
    DataLogger::Log( "Command/Log", "   Command " +  this->GetName() + " ended" );
  }

  virtual void Init() {}
  virtual void Ending( bool interrupted ) {}
};
