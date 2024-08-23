/// \file Parameters.h
/// \brief Configuration for the main robot servos.
/// \author MiAM Robotique, Matthieu Vigne
/// \copyright GNU GPLv3

#ifndef PARAMETERS_H
    #define PARAMETERS_H
        #include <math.h>

        // Dimensions of the robot
        namespace robotdimensions
        {
            float const wheelRadius = 49.1; ///< Wheel radius, in mm - identified during open loop experiments.
            float const wheelSpacing = 100.5; ///< Wheel spacing from robot center, in mm - identified during open loop experiments.
            float const encoderWheelRadius = 25.3; ///< Radius of encoder wheels, in mm.
            float const encoderWheelSpacing = 139.0; ///< Encoder wheel spacing from robot center, in mm.

            float const stepSize = 2 * M_PI / 600.0; ///< Size of a motor step, in rad.

            float const maxWheelSpeed = 400; ///< Maximum wheel speed, in mm/s.
            float const maxWheelAcceleration = 1000; ///< Maximum wheel acceleration, in mm/s^2.
        }

 #endif