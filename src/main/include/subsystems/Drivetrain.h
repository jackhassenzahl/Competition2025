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

        // Private methods
        void FieldCentricAngleConversion(double *forward, double *strafe, double angle);
        void CalculateSwerveModuleDriveAndAngle(double forward, double strafe, double rotate, WheelVector wheelVector[]);
        void NormalizeSpeed(WheelVector wheelVector[]);

        double PI = acos(-1.0);
        double R  = sqrt((ChassisConstants::kChassisLength * ChassisConstants::kChassisLength) + (ChassisConstants::kChassisWidth * ChassisConstants::kChassisWidth));

        SwerveModule *m_swerveModule[ChassisConstants::kNumberOfSwerveModules];  // Pointers to the four swerve modules
};
