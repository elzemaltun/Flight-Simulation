// simulation_model.cpp : This file contains the 'main' function. Program execution begins and ends there.
//

#include <iostream>
#include <fstream>
#include "PlaneModel.h"
#include "Controller.h"

using namespace std;


int main()
{
    const double dt = 0.2; // Time step for simulation

    // User inputs
    double velocity_reference;
    double altitude_reference;
    cout << "Please enter a reference velocity and altitude: ";
    cin >> velocity_reference >> altitude_reference;

    
    // Create instances of PlaneModel and Controller
    PlaneModel plane(0, 0, 0, 0, 0, 0); // Initial values for state variables
    // Give small gains
    // Determine gains by trial-error
    // controller(kp_pitch, kd_pitch, ki_pitch, kp_velocity, kd_velocity, ki_velocity,)
    Controller controller(-0.005, -0.0001, 0.0002, 0.0002, -0.005, 0.0001); // Controller gains


    // Open a CSV file
    ofstream csvFile("simulation_results.csv");

    // Simulation loop
    for (double t = 0; t <= 10; t += dt) {
        // Calculate altitude error and velocity error
        double altitude_error = altitude_reference - plane.getH();
        double velocity_error = velocity_reference - plane.getVelocity();

        // Calculate control inputs using altitude_error and velocity_error
        controller.calculateControlInputs(altitude_error, velocity_error, dt);

        // Update state variables using PlaneModel's update method
        plane.update(dt, controller.getElevatorDeflection(), controller.getThrustAdjustment());


        // Write simulation results to CSV file
        csvFile << t << ", " << plane.getVelocity() << ", " << velocity_reference << ", "
            << plane.getH() << ", " << altitude_reference << ", "
            << plane.getGamma() << ", " << plane.getAngleOfAttack() << "\n";
    }

    // Close the CSV file
    csvFile.close();

    return 0;

}