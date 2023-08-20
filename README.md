# Flight Simulation Project

This project simulates the longitudinal flight behavior of an aircraft weights 12 kg using a simple model. The goal is to control the velocity and altitude behavior of the aircraft with a PID controller by giving it reference altitude and velocity. Ideally, the response signal should be fast with no oscillations. 

In the model, for the wing, E176 airfoil and for the tail NACA0010 airfoil have been chosen. For the physical qualifications, Boeing's ScanEagle UAV has been chosen. Some of the values for modelling the plane are assumed according to the general sense. In the code, those assumed values are written. Air density is assumed to be constant and at sea level. 

6 DOF equations are used to model the aircraft's motion in the longitudinal axis. Since only the longitudinal motion is modelled, some simplifications are made. Euler's Method is used for solving those equations. 

To control the velocity and altitude, PID controller is used. The gains of the PID controller are determined by the trial-error method.

Data collection is done at 5Hz.


## Project Structure

main.cpp: This is the main program that orchestrates the simulation and controls the interaction between the PlaneModel and Controller.

PlaneModel.cpp and PlaneModel.h: These files contain the implementation and class definition for the PlaneModel, which simulates the behavior of the aircraft.

Controller.cpp and Controller.h: These files contain the implementation and class definition for the Controller, which computes control inputs for the aircraft based on altitude and velocity errors.


## Compilation and Execution

To compile and run the simulation, first, download the project into a folder. Then, follow these steps:

1. Open a terminal.
2. Navigate to the project directory:
    cd FileName

3. Compile the source files using g++:
    g++ -o simulation main.cpp PlaneModel.cpp Controller.cpp

4. Run the compiled executable:
    simulation.exe

## How to Use the Program

The user should enter desired velocity and altitude respectively. The results will be written in a .csv file.

## Simulation Output

This simulation will collect the following information and write it into a CSV file in the same order: time, velocity, reference velocity, altitude, reference altitude, pitch angle and angle of attack.


## Known Issues
The results are not correct. According to the results, the angle of attack and altitude become lower than 0. However, normally, the aircraft's altitude must increase as the velocity increases. This error might be caused by most probably PID controller. However, no matter how much I try to fix it, I was never been able to receive logical results. The 6DOF equations and Euler's method are correct.


## Contact Me
If you have any questions, please do not hesitate to contact me.
email: elzemaltunn@gmail.com
