#include "Controller.h"

Controller::Controller(double kp_p, double kd_p, double ki_p, double kp_v, double kd_v, double ki_v)
    : kp_pitch(kp_p), kd_pitch(kd_p), ki_pitch(ki_p), kp_velocity(kp_v), kd_velocity(kd_v), ki_velocity(ki_v),
    integral_altitude(0.0), integral_velocity(0.0), prev_altitude_error(0.0), prev_velocity_error(0.0) {
}

// PID controlller design
void Controller::calculateControlInputs(double altitude_error, double velocity_error, double dt) {

    // Calculate integral terms
    integral_altitude += altitude_error * dt;
    integral_velocity += velocity_error * dt;

    // delta_thrust is for controlling the velocity via thrust
    // delta_elevator is for controlling altitude by changing pitch angle via elevator
    delta_elevator = altitude_error * kp_pitch + ki_pitch * integral_altitude - (altitude_error - prev_altitude_error) / dt * kd_pitch;
    delta_thrust = velocity_error * kp_velocity + ki_velocity * integral_velocity - (velocity_error - prev_velocity_error) / dt * kd_velocity;
}

// Getter methods 
double Controller::getElevatorDeflection() {
    return delta_elevator;
}

double Controller::getThrustAdjustment() {
    return delta_thrust;
}