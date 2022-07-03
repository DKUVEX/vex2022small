#ifndef API_H
#define API_H

#include <iostream>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "v5.h"
#include "v5_vcs.h"
#include <eigen-3.4.0/Eigen/Dense>

using namespace vex;
using signature = vision::signature;
using code = vision::code;

brain Brain;
controller Controller1 = controller(primary);
competition Competition;

// Motorjh
motor LA = motor(PORT8, ratio18_1, 1); 
motor LB = motor(PORT10, ratio18_1, 1); 
motor RA = motor(PORT2, ratio18_1, 1); 
motor RB = motor(PORT1,  ratio18_1, 1);

motor fw1 = motor(PORT11,  ratio6_1, true); 
motor fw2 = motor(PORT12,  ratio6_1, true);

motor itk = motor(PORT20, ratio18_1, false); 
motor ind = motor(PORT3, ratio18_1, false);

// Encoder
encoder Hor = encoder(Brain.ThreeWirePort.G);
encoder Ver = encoder(Brain.ThreeWirePort.A);

digital_out sol = digital_out(Brain.ThreeWirePort.C);

// Sensor
gps GPS = gps(PORT13, 0.00, 0.00, mm, 0.00);

inertial Gyro = inertial(PORT3);
gyro v4gyro = gyro(Brain.ThreeWirePort.D);

#endif