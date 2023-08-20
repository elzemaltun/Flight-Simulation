#include "PlaneModel.h"

// AC distances of wing and tail
const double PlaneModel::tail_distance = 1.0; // tail's aerodynamic center from cg
const double PlaneModel::wing_distance = 0.5; // wing's aerodynamic center from cg

// Define wing and tail properties
const double PlaneModel::wing_span = 3.1;
const double PlaneModel::chord_length = 1.0;
const double PlaneModel::tail_span = 1.0;
const double PlaneModel::aspect_ratio = wing_span / chord_length;
const double PlaneModel::wing_area = wing_span * chord_length;
const double PlaneModel::tail_area = tail_span * chord_length;
const double PlaneModel::Cl_0_wing = 0.2; //lift coefficient when aoa = 0 for E176 airfoil
const double PlaneModel::Cl_aoa_wing = 0.1; //slope of CL vs aoa graph for wing
const double PlaneModel::Cd_0 = 0.02; //drag coefficient when aoa = 0
const double PlaneModel::Cl_0_tail = 0.0; //lift coefficient when aoa = 0 at the tail for NACA0015
const double PlaneModel::Cl_aoa_tail = 0.4;

// Define and initialize other constants
const double PlaneModel::pi = 2.0 * acos(0.0);
const double PlaneModel::m = 12.0; // mass in kg
const double PlaneModel::g = 9.81; // gravity in m/s^2
const double PlaneModel::rho = 1.225; // air density at sea level in kg/m^35;
const double PlaneModel::Iyy = 24.0; // pitch moment of inertia (assumed);
const double PlaneModel::max_throttle = 100.0;
const double PlaneModel::delta_max_elevator = 100.0;

PlaneModel::PlaneModel(double initial_velocity, double initial_angle_of_attack, double initial_q, double initial_theta, double initial_h, double initial_gamma)
    : velocity(initial_velocity), angle_of_attack(initial_angle_of_attack), q(initial_q), theta(initial_theta), h(initial_h), gamma(initial_gamma) {

}

void PlaneModel::update(double dt, double delta_elevator, double delta_thrust) {

    // Calculate Lift and drag coefficients
    double CL_wing = Cl_0_wing + Cl_aoa_wing * angle_of_attack * (pi / 180.0); //lift coefficient at the wing
    double CL_tail = Cl_0_tail + Cl_aoa_tail * angle_of_attack * (pi / 180.0) + 0.2 * (delta_elevator / delta_max_elevator); //lift coefficient at the tail
    double CD = Cd_0 + pow(Cl_0_wing, 2.0) / (pi * aspect_ratio * 0.7);

    // Calculate moment, moment is essential for the longitudinal motion equations.
    double moment = 1 / 2 * rho * pow(velocity, 2.0) * ((wing_area * CL_wing * wing_distance) - (tail_area * CL_tail * tail_distance));

    // Model the forces on the plane
    double weight = m * g;
    double thrust = max_throttle * delta_thrust;
    double lift = 1 / 2 * rho * pow(velocity, 2.0) * wing_area * CL_wing;
    double drag = 1 / 2 * rho * pow(velocity, 2.0) * wing_area * CD;

    // Longitudinal motion equations
    // There made some modificationd in 6DOF equations since we only simulate the longitudinal motion. p = r = phi = ksi = Z = N = 0.
    // Since the a/c is symmetrical, product of inertas become 0. 
    double v_dot = (thrust * cos(angle_of_attack) - drag - weight * sin(gamma)) / m;
    double h_dot = velocity * sin(gamma);
    double alpha_dot;
    if (velocity == 0.0) {
        alpha_dot = q;
    }
    else {
        alpha_dot = (weight * cos(gamma) - lift - thrust * sin(angle_of_attack)) / (m * velocity) + q;
    }
    double q_dot = moment / Iyy;
    double theta_dot = q;
    double gamma_dot = q - alpha_dot;

    // Update state variables using Euler's method
    // In order to solve longitudinal motion equations, Euler's Method must be used.
    velocity += v_dot * dt;
    angle_of_attack += alpha_dot * dt;
    q += q_dot * dt;
    theta += theta_dot * dt;
    gamma += gamma_dot * dt;
    h += h_dot * dt;
}

// Getter methods to access the plane's state variables
double PlaneModel::getVelocity() {
    return velocity;
}

double PlaneModel::getAngleOfAttack() {
    return angle_of_attack;
}

double PlaneModel::getQ() {
    return q;
}

double PlaneModel::getTheta() {
    return theta;
}

double PlaneModel::getH() {
    return h;
}

double PlaneModel::getGamma() {
    return gamma;
}