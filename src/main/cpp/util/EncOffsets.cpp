

#include "util/EncOffsets.h"

#include <frc/Preferences.h>
#include <frc/DriverStation.h>
#include <frc/smartdashboard/SmartDashboard.h>

EncOffsets *EncOffsets::singleton = nullptr;
#define UpdateNT "{}_Pref/Update Offset"
#define OffsetNT "{}_Pref/Offset"
#define AngleNT "{}_Pref/Angle"

void EncOffsets::Listen( const std::string& name, std::function<void()> CB_func )
{
    cb_map[ name ] = CB_func;
}

double EncOffsets::Get( const std::string& name )
{
    return frc::Preferences::GetDouble( name + "Offset" );
}

void EncOffsets::Set( const std::string& name, double value )
{
    frc::Preferences::SetDouble( name + "Offset", value );
    frc::SmartDashboard::PutNumber( fmt::format( OffsetNT, name), 0.0);
}

void EncOffsets::UpdateAngle( const std::string& name, double angle )
{
    if( !frc::DriverStation::IsFMSAttached() && frc::DriverStation::IsDisabled() ) {
        if( cb_map.contains( name ) ) {
            frc::SmartDashboard::PutNumber( fmt::format( AngleNT, name), angle);
        }
    }
}

void EncOffsets::SetupUI()
{
    if( !frc::DriverStation::IsFMSAttached() ) {
        for( auto &m : cb_map ) {
            frc::SmartDashboard::PutBoolean( fmt::format( UpdateNT, m.first), false);
            frc::SmartDashboard::PutNumber( fmt::format( OffsetNT, m.first), 0.0);
            frc::SmartDashboard::PutNumber( fmt::format( AngleNT, m.first), 0.0);
        }
    }
}

void EncOffsets::UpdateUI()
{
    if( !frc::DriverStation::IsFMSAttached() ) {
        for( auto &m : cb_map ) {
            std::string label = fmt::format( UpdateNT, m.first );
            if( frc::SmartDashboard::GetBoolean(label, false) ) {
                m.second();
                frc::SmartDashboard::PutBoolean(label, false);
            }
        }
    }
}

EncOffsets& EncOffsets::GetInstance() 
{
    if( singleton == nullptr ) {
        singleton = new EncOffsets();
    }

    return *singleton;
}
