#pragma once

#include "studica/AHRS.h"

#include <frc2/command/SubsystemBase.h>
#include <frc2/command/CommandPtr.h>

#include "SwerveModule.h"

#include "Constants.h"

class Drivetrain : public frc2::SubsystemBase
{
    public:

        explicit        Drivetrain();

        void            Periodic() override;

        void            Drive(double forward, double strafe, double angle);

        void            SetFieldCentricity(bool fieldCentric);
        bool            GetFieldCentricity();

        void            SetWheelAnglesToZero();

        units::degree_t GetHeading();

        WheelVector*    GetSwerveModuleWheelVector(int swerveModuleIndex);

    private:

        // Private methods
        void FieldCentricAngleConversion(double *forward, double *strafe, double angle);
        void CalculateSwerveModuleDriveAndAngle(double forward, double strafe, double rotate, WheelVector wheelVector[]);
        void NormalizeSpeed(WheelVector wheelVector[]);

        double R = sqrt((ChassisConstants::ChassisLength * ChassisConstants::ChassisLength) +
                        (ChassisConstants::ChassisWidth  * ChassisConstants::ChassisWidth));

        SwerveModule *m_swerveModule[ChassisConstants::NumberOfSwerveModules];  // Pointers to the four swerve modules

        bool          m_fieldCentricity = false;                                // Field centricity flag

        studica::AHRS m_navx{studica::AHRS::NavXComType::kMXP_SPI};             // navX MXP using SPI
};
