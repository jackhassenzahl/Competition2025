#include "subsystems/Leds.h"

#include <iostream>

#include <frc/smartdashboard/SmartDashboard.h>

using namespace std;

/// @brief Class to support an addressable LED string.
Leds::Leds()
{
    // Length is expensive to set, so only set it once, then just update data
    m_led.SetLength(LedConstants::kLength);

    // Set the default mode
    SetMode(LedMode::Off);

    // Intialize the LED data
    m_led.SetData(m_ledBuffer);

    // Start the addressable LED communications
    m_led.Start();
}

/// @brief This method will be called once periodically.
void Leds::Periodic()
{
    frc::SmartDashboard::PutNumber("LED Counter", m_liveCounter++);

    switch (m_ledMode)
    {
    case LedMode::Off:
    case LedMode::SolidGreen:
    case LedMode::SolidRed:
    case LedMode::HvaColors:
        return;

    case LedMode::Strobe:
        Strobe();
        break;

    case LedMode::ShootingAnimation:
        ShootingAnimation();
        break;

    case LedMode::Rainbow:
        Rainbow();
        break;
    }

    // Set the LEDs
    m_led.SetData(m_ledBuffer);
}

/// @brief Setting the Led's mode to the given parameter.
/// @param ledMode mode to set the Leds.
void Leds::SetMode(LedMode ledMode)
{
    // Remember the LED mode
    m_ledMode = ledMode;

    // Set the LEDs based on the LED mode
    switch (m_ledMode)
    {
    case LedMode::Off:
        SolidColor(0, 0, 0);
        break;

    case LedMode::SolidGreen:
        SolidColor(0, 255, 0);
        break;

    case LedMode::SolidRed:
        SolidColor(255, 0, 0);
        break;

    case LedMode::HvaColors:
        m_cycleCounter = 0;
        HvaColors();
        break;

    case LedMode::Strobe:
        m_cycleCounter = 0;
        Strobe();
        break;

    case LedMode::ShootingAnimation:
        m_cycleCounter = 0;
        ShootingAnimation();
        break;

    case LedMode::Rainbow:
        Rainbow();
        break;

    default:
        break;
    }

    // Set the LEDs
    m_led.SetData(m_ledBuffer);
}

/// @brief Method to support setting the LED string to the specified solid color.
/// @param red The red component of the LED color.
/// @param green The green component of the LED color.
/// @param blue The blue component of the LED color.
void Leds::SolidColor(int red, int green, int blue)
{
    // Set the value for every pixel
    for (int ledIndex = 0; ledIndex < LedConstants::kLength; ledIndex++)
        m_ledBuffer[ledIndex].SetRGB(red * LedConstants::kBrightness, green * LedConstants::kBrightness, blue * LedConstants::kBrightness);
}

/// @brief Method to support setting the LED string to HVA alternating color.
void Leds::HvaColors()
{
    // For every pixel
    for (int ledIndex = 0; ledIndex < LedConstants::kLength; ledIndex++)
    {
        if (ledIndex % 2 == 0)
        {
            // Set the value
            m_ledBuffer[ledIndex].SetRGB(0, 0, 255 * LedConstants::kBrightness);
        }
        else
        {
            // Set the value
            m_ledBuffer[ledIndex].SetRGB(0, 0, 100 * LedConstants::kBrightness);
        }
    }

    // Update the cycle counter
    m_cycleCounter++;
}

/// @brief Method to strobe the LED string.
void Leds::Strobe()
{
    if (m_cycleCounter % 20 == 0)
        SolidColor(255, 255, 255);
    else
        SolidColor(0, 0, 0);

    // Update the cycle counter
    m_cycleCounter++;
}

/// @brief Methdo to set the LED stdring to shooting animation.
void Leds::ShootingAnimation()
{

}

/// @brief 
void Leds::Rainbow()
{
    // For every pixel
    for (int ledIndex = 0; ledIndex < LedConstants::kLength; ledIndex++)
    {
        // Calculate the hue - hue is easier for rainbows because the color
        // shape is a circle so only one value needs to precess
        const auto pixelHue = (m_firstPixelHue + (ledIndex * 45 / LedConstants::kLength)) % 360;

        // Set the value
        m_ledBuffer[ledIndex].SetHSV(pixelHue, 255, (int)(255 * LedConstants::kBrightness));
    }

    // Increase by to make the rainbow "move"
    m_firstPixelHue += LedConstants::kRainbowRate;

    // Check bounds
    m_firstPixelHue %= 180;
}
