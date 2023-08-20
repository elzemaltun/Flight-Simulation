#pragma once

#include <iostream>
#include <cmath>

class PlaneModel {
private:
    // constant variables
    static const double tail_distance;// tail's aerodynamic center from cg
    static const double wing_distance;// wing's aerodynamic center from cg
    static const double pi;
    static const double m; // mass in kg
    static const double g; // gravity in m/s^2
    static const double rho; // air density at sea level in kg/m^3
    static const double Iyy; // pitch moment of inertia (assumed)

    // wing and tail properties
    static const double wing_span;
    static const double chord_length;
    static const double tail_span;
    static const double aspect_ratio;
    static const double wing_area;
    static const double tail_area;

    // airfoil properties
    static const double Cl_0_wing; //lift coefficient when aoa = 0 for E176 airfoil
    static const double Cl_aoa_wing; //slope of CL vs aoa graph for wing
    static const double Cd_0; //drag coefficient when aoa = 0
    static const double Cl_0_tail; // lift coefficient at the tail for NACA0015
    static const double Cl_aoa_tail;

    static const double max_throttle;
    static const double delta_max_elevator;

    // State variables
    double velocity;
    double angle_of_attack;
    double q;
    double theta;
    double h;
    double gamma;

public:

    PlaneModel(double initial_velocity, double initial_angle_of_attack, double initial_q, double initial_theta, double initial_h, double initial_gamma);
    
    void update(double dt, double delta_elevator, double delta_thrust);

    // Getter methods
    
    double getVelocity();
    double getAngleOfAttack();
    double getQ();
    double getTheta(); 
    double getH();
    double getGamma();
};
