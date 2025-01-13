#pragma once

#include <array>

#include <frc/AddressableLED.h>
#include <frc2/command/SubsystemBase.h>

#include "Constants.h"

/// @brief modes for the LED string.
enum LedMode
{
    Off,
    SolidGreen,
    SolidRed,
    HvaColors,
    Strobe,
    ShootingAnimation,
    Rainbow
};

class Leds : public frc2::SubsystemBase
{
    public:

        explicit Leds();

        void     Periodic() override;
        void     SetMode(LedMode ledMode);

    private:

        void Off();
        void SolidColor(int red, int green, int blue);
        void Rainbow();
        void HvaColors();
        void Strobe();
        void ShootingAnimation();
        
        LedMode m_ledMode;                         // The LED mode

        int     m_firstPixelHue = 0;               // Store the hue of the first pixel for rainbow mode
        int     m_cycleCounter  = 0;               // Counter for dynamic LED modes
        int     m_liveCounter   = 0;               // Counter to indicate that the periodic method is being called

        frc::AddressableLED m_led{LedConstants::kPwmPort};
        
        std::array<frc::AddressableLED::LEDData, LedConstants::kLength> m_ledBuffer;  // Instatntiate the LED data buffer
};
