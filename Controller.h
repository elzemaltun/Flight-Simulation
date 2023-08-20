#pragma once

#include <iostream>
#include <cmath>

class Controller {
private:
    // Controller gains
    double kp_pitch;
    double kd_pitch;
    double kp_velocity;
    double kd_velocity;
    double ki_pitch;
    double ki_velocity;

    // Integral terms and previous errors
    double integral_altitude;
    double integral_velocity;
    double prev_altitude_error;
    double prev_velocity_error;

    // Thrust and elevator deflections
    double delta_elevator;
    double delta_thrust;

public:
    Controller(double kp_p, double kd_p, double ki_p, double kp_v, double kd_v, double ki_v);

    void calculateControlInputs(double altitude_error, double velocity_error, double dt);

    // Getter methods 
    double getElevatorDeflection();
    double getThrustAdjustment();
};