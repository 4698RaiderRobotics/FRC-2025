#pragma once

#include <string>
#include <functional>
#include <map>

/*
 *  Manage the encoder offsets stored in the RoboRIO preferences.
 */
class EncOffsets {
public:
    // delete copy constructor
    EncOffsets(const EncOffsets& obj) = delete; 

    void Listen( const std::string& name, std::function<void()> CB_func );
    double Get( const std::string& name );
    void Set( const std::string& name, double value );
    void UpdateAngle( const std::string& name, double angle );

    void SetupUI();
    void UpdateUI();
    static EncOffsets& GetInstance();

private:
    // This class is a singleton.
    static EncOffsets *singleton;

    // Constructor is private
    EncOffsets() {}

    std::map<std::string, std::function<void()>> cb_map;
};
