

#include "util/EncOffsets.h"

#include <frc/Preferences.h>
#include <frc/DriverStation.h>
#include <frc/smartdashboard/SmartDashboard.h>

EncOffsets *EncOffsets::singleton = nullptr;

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
}

void EncOffsets::Update( const std::string& name )
{
    if( cb_map.contains( name ) ) {
        cb_map[name]();
    }
}

void EncOffsets::SetupUI()
{
    if( !frc::DriverStation::IsFMSAttached() ) {
        for( auto &m : cb_map ) {
            frc::SmartDashboard::PutBoolean( fmt::format("Update {} Offset" , m.first), false);
        }
    }
}

void EncOffsets::UpdateUI()
{
    if( !frc::DriverStation::IsFMSAttached() ) {
        for( auto &m : cb_map ) {
            std::string label = fmt::format( "Update {} Offset", m.first );
            if( frc::SmartDashboard::GetBoolean(label, false) ) {
                Update( m.first );
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
