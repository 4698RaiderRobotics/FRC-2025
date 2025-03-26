#pragma once

#include <functional>

#include <frc/filter/Debouncer.h>

namespace util {

template<typename T>
T clamp( const T &val, const T min, const T max ) {
    if( val < min ) return min;
    if( val > max ) return max;
    return val;
}

class MotorHomer {
public:
    MotorHomer( ) {}
    MotorHomer( std::function<void()> start, std::function<void()> stopAndReset, std::function<bool()> homeCondition, units::second_t debounce=100_ms )
        : startFunc{ start }, stopResetFunc{ stopAndReset }, isHomedFunc{ homeCondition }
    {
        isHomedDebounced = std::unique_ptr<frc::Debouncer>( new frc::Debouncer(debounce) );
    }

    void Home( ) 
    {
        if( homing_is_done || isHomedDebounced == nullptr ) return;

        if( !homing_is_active ) {
            startFunc();
            homing_is_active = true;
                // Reset the debouncer.
            isHomedDebounced->Calculate(false);
        }

        if( isHomedDebounced->Calculate( isHomedFunc() ) ) {
            stopResetFunc();
            homing_is_active = false;
            homing_is_done = true;
        }
    }

    void Reset( ) 
    {
        homing_is_done = false;
        homing_is_active = false;
    }

    bool isHomingActive() { return homing_is_active; }
    bool isHomingDone() { return homing_is_done; }
    
private:
    std::function<void()> startFunc;
    std::function<void()> stopResetFunc;
    std::function<bool()> isHomedFunc;
    bool homing_is_done = false;
    bool homing_is_active = false;
    std::unique_ptr<frc::Debouncer> isHomedDebounced;
};

}