#pragma once

#include "Constants.h"

#include <array>

#include <frc/AddressableLED.h>
#include <frc2/command/SubsystemBase.h>

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

        // Class constructor
        Leds();

        // Will be called periodically whenever the CommandScheduler runs.
        void Periodic() override;

        void SetMode(LedMode ledMode);

    private:

        static constexpr int kLength = 410;  // The length of the LED string

        LedMode m_ledMode;                   // The LED mode

        int firstPixelHue = 0;               // Store the hue of the first pixel for rainbow mode
        int cycleCounter  = 0;               // Counter for dynamic LED modes

        frc::AddressableLED m_led{LED_PWM_PORT};
        
        std::array<frc::AddressableLED::LEDData, kLength> m_ledBuffer;  // Instatntiate the LED data buffer

        void Off();
        void SolidColor(int red, int green, int blue);
        void Rainbow();
        void HvaColors();
        void Strobe();
        void ShootingAnimation();
};
