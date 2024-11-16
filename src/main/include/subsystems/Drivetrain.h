#pragma once

#include "Constants.h"
#include "SwerveModule.h"

#include <frc2/command/SubsystemBase.h>
#include <frc2/command/CommandPtr.h>

#include <cmath>

struct MathSwerveModule
{
    double Drive[4];
    double Angle[4];
};

class Drivetrain : public frc2::SubsystemBase
{
    public:
    
        Drivetrain();
        
        // Robot centric, therefore no need for gyro
        void Drive(double forward, double strafe, double angle);

        // Field centric, so use gyro
        void Drive(double forward, double strafe, double angle, double gyro);

    private:

        // Private method prototypes
        void FieldCentricAngleConversion(double *forward, double *strafe, double angle);
        void CalculateSwerveModuleDriveAndAngle(double forward, double strafe, double rotate, MathSwerveModule *swerveModule);
        void OptimizeWheelAngle(MathSwerveModule pastSwerveModule, MathSwerveModule desiredSwerveModule, MathSwerveModule *newSwerveModule);
        void NormalizeSpeed(MathSwerveModule *swerveModule);

        double PI = acos(-1.0);
        double R  = sqrt((ChassisLength * ChassisLength) + (ChassisWidth * ChassisWidth));

        std::unique_ptr<SwerveModule> swerveModule[4];

        MathSwerveModule m_mathSwerveModule;
};
