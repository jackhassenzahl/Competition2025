#pragma once

#include "Constants.h"
#include "SwerveModule.h"

#include <frc2/command/SubsystemBase.h>
#include <frc2/command/CommandPtr.h>

#include <cmath>

class Drivetrain : public frc2::SubsystemBase
{
    public:

        Drivetrain();

        // Robot centric, therefore no need for gyro
        void Drive(double forward, double strafe, double angle);

        // Field centric, so use gyro
        void Drive(double forward, double strafe, double angle, double gyro);

        void GetSwerveModuleWheelVector(int swerveModuleIndex, WheelVector* wheelVector);

    private:

        double PI = acos(-1.0);
        double R  = sqrt((CHASSIS_LENGTH * CHASSIS_LENGTH) + (CHASSIS_WIDTH * CHASSIS_WIDTH));

        SwerveModule *m_swerveModule[NUMBER_OF_SWERVE_MODULES];  // Pointers to the four swerve modules

        // Private methods
        void FieldCentricAngleConversion(double *forward, double *strafe, double angle);
        void CalculateSwerveModuleDriveAndAngle(double forward, double strafe, double rotate, WheelVector wheelVector[]);
        void NormalizeSpeed(WheelVector wheelVector[]);
};
